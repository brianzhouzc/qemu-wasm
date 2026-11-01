/*
 * QEMU TCG Single Threaded vCPUs implementation
 *
 * Copyright (c) 2003-2008 Fabrice Bellard
 * Copyright (c) 2014 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/lockable.h"
#include "sysemu/tcg.h"
#include "sysemu/replay.h"
#include "sysemu/cpu-timers.h"
#include "qemu/main-loop.h"
#include "qemu/notify.h"
#include "qemu/guest-random.h"
#include "exec/exec-all.h"
#include "tcg/startup.h"
#include "tcg-accel-ops.h"
#include "tcg-accel-ops-rr.h"
#include "tcg-accel-ops-icount.h"

/*
 * The Wasm backend keeps per-thread state in Module.__wasm32_tb, set up by
 * init_wasm32() from the vCPU thread. The ktock tree only did that in the
 * MTTCG thread function, and MTTCG refuses -icount, so the single-thread path
 * (the one the iPod machine needs) hit a null __wasm32_tb on its first TB.
 */
#if defined(EMSCRIPTEN) && !defined(CONFIG_TCG_INTERPRETER)
#include "../../tcg/wasm32.h"
#endif

#ifdef EMSCRIPTEN
/*
 * Longest the vCPU thread keeps a halted guest's warps to itself before it
 * lets the main loop have a turn.  See rr_warp_here().
 */
#define RR_WARP_HANDOFF_NS SCALE_MS

static int64_t rr_last_handoff_ns;

/*
 * Advance the virtual clock from this thread, and say whether that made a timer
 * due, in which case the caller goes straight back and runs it.
 *
 * Under -icount sleep=off a warp is otherwise a round trip: the idle vCPU wakes
 * the main loop, main_loop_wait() calls icount_start_warp_timer(), that moves
 * the clock and queues work to wake the vCPU, and the vCPU then runs the timers
 * in icount_handle_deadline().  The main loop's part is that one call, and
 * nothing in it needs that thread.  The virtual timers already run here under
 * the BQL, and a timer callback that rearms while the guest is halted already
 * warps from here, through timerlist_rearm().  MEASURED on stock 2.0.5: 23,000
 * round trips in a boot, 22 us each, 2% of it, and 10,000 a second when idle.
 *
 * THE VCPU MUST STILL LET GO OF THE BQL.  A guest that runs between warps
 * releases it in tcg_cpus_exec(), but a timer whose interrupt is masked warps
 * again with no guest code in between, and everything on host time lives on
 * the main loop.  So the round trip is still taken once a millisecond, which
 * releases the lock until the main loop has been round.  It costs 9% of the old
 * round trips, and tests/warp-spin.test.js in iPodWasm is that guest.
 *
 * A stopped VM must not get here: a timer already due would then spin on this
 * path instead of waiting for the resume.  all_cpu_threads_idle() calls a
 * stopped vCPU idle, which is right for everyone else.
 */
static bool rr_warp_here(void)
{
    if (!cpu_can_run(first_cpu) ||
        get_clock() - rr_last_handoff_ns >= RR_WARP_HANDOFF_NS) {
        return false;
    }
    icount_start_warp_timer();
    return qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                      ~QEMU_TIMER_ATTR_EXTERNAL) == 0;
}
#endif

/* Kick all RR vCPUs */
void rr_kick_vcpu_thread(CPUState *unused)
{
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        cpu_exit(cpu);
    };
}

/*
 * TCG vCPU kick timer
 *
 * The kick timer is responsible for moving single threaded vCPU
 * emulation on to the next vCPU. If more than one vCPU is running a
 * timer event we force a cpu->exit so the next vCPU can get
 * scheduled.
 *
 * The timer is removed if all vCPUs are idle and restarted again once
 * idleness is complete.
 */

static QEMUTimer *rr_kick_vcpu_timer;
static CPUState *rr_current_cpu;

static inline int64_t rr_next_kick_time(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TCG_KICK_PERIOD;
}

/* Kick the currently round-robin scheduled vCPU to next */
static void rr_kick_next_cpu(void)
{
    CPUState *cpu;
    do {
        cpu = qatomic_read(&rr_current_cpu);
        if (cpu) {
            cpu_exit(cpu);
        }
        /* Finish kicking this cpu before reading again.  */
        smp_mb();
    } while (cpu != qatomic_read(&rr_current_cpu));
}

static void rr_kick_thread(void *opaque)
{
    timer_mod(rr_kick_vcpu_timer, rr_next_kick_time());
    rr_kick_next_cpu();
}

static void rr_start_kick_timer(void)
{
    if (!rr_kick_vcpu_timer && CPU_NEXT(first_cpu)) {
        rr_kick_vcpu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                           rr_kick_thread, NULL);
    }
    if (rr_kick_vcpu_timer && !timer_pending(rr_kick_vcpu_timer)) {
        timer_mod(rr_kick_vcpu_timer, rr_next_kick_time());
    }
}

static void rr_stop_kick_timer(void)
{
    if (rr_kick_vcpu_timer && timer_pending(rr_kick_vcpu_timer)) {
        timer_del(rr_kick_vcpu_timer);
    }
}

static void rr_wait_io_event(void)
{
    CPUState *cpu;

    while (all_cpu_threads_idle()) {
        rr_stop_kick_timer();
        /*
         * Q9: the bounded emscripten_thread_sleep() poll used by
         * qemu_wait_io_event() was tried here as well, and it stalls the
         * icount boot: the guest halts in its idle loop at about 38 M
         * instructions and never wakes (the virtual clock keeps advancing).
         * The condition variable works on this path, so it stays; the
         * bounded poll is only for the non-RR vCPU threads.
         */
        qemu_cond_wait_iothread(first_cpu->halt_cond);
    }

    rr_start_kick_timer();

    CPU_FOREACH(cpu) {
        qemu_wait_io_event_common(cpu);
    }
}

/*
 * Destroy any remaining vCPUs which have been unplugged and have
 * finished running
 */
static void rr_deal_with_unplugged_cpus(void)
{
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        if (cpu->unplug && !cpu_can_run(cpu)) {
            tcg_cpus_destroy(cpu);
            break;
        }
    }
}

static void rr_force_rcu(Notifier *notify, void *data)
{
    rr_kick_next_cpu();
}

/*
 * Calculate the number of CPUs that we will process in a single iteration of
 * the main CPU thread loop so that we can fairly distribute the instruction
 * count across CPUs.
 *
 * The CPU count is cached based on the CPU list generation ID to avoid
 * iterating the list every time.
 */
static int rr_cpu_count(void)
{
    static unsigned int last_gen_id = ~0;
    static int cpu_count;
    CPUState *cpu;

    QEMU_LOCK_GUARD(&qemu_cpu_list_lock);

    if (cpu_list_generation_id_get() != last_gen_id) {
        cpu_count = 0;
        CPU_FOREACH(cpu) {
            ++cpu_count;
        }
        last_gen_id = cpu_list_generation_id_get();
    }

    return cpu_count;
}

/*
 * In the single-threaded case each vCPU is simulated in turn. If
 * there is more than a single vCPU we create a simple timer to kick
 * the vCPU and ensure we don't get stuck in a tight loop in one vCPU.
 * This is done explicitly rather than relying on side-effects
 * elsewhere.
 */

static void *rr_cpu_thread_fn(void *arg)
{
    Notifier force_rcu;
    CPUState *cpu = arg;

#if defined(EMSCRIPTEN) && !defined(CONFIG_TCG_INTERPRETER)
    init_wasm32();
#endif
    assert(tcg_enabled());
    rcu_register_thread();
    force_rcu.notify = rr_force_rcu;
    rcu_add_force_rcu_notifier(&force_rcu);
    tcg_register_thread();

    qemu_mutex_lock_iothread();
    qemu_thread_get_self(cpu->thread);

    cpu->thread_id = qemu_get_thread_id();
    cpu->neg.can_do_io = true;
    cpu_thread_signal_created(cpu);
    qemu_guest_random_seed_thread_part2(cpu->random_seed);

    /* wait for initial kick-off after machine start */
    while (first_cpu->stopped) {
        qemu_cond_wait_iothread(first_cpu->halt_cond);

        /* process any pending work */
        CPU_FOREACH(cpu) {
            current_cpu = cpu;
            qemu_wait_io_event_common(cpu);
        }
    }

    rr_start_kick_timer();

    cpu = first_cpu;

    /* process any pending work */
    cpu->exit_request = 1;

    while (1) {
        /* Only used for icount_enabled() */
        int64_t cpu_budget = 0;
        bool idle;

        qemu_mutex_unlock_iothread();
        replay_mutex_lock();
        qemu_mutex_lock_iothread();

        if (icount_enabled()) {
            int cpu_count = rr_cpu_count();

            /* Account partial waits to QEMU_CLOCK_VIRTUAL.  */
            icount_account_warp_timer();
            /*
             * Run the timers here.  This is much more efficient than
             * waking up the I/O thread and waiting for completion.
             */
            icount_handle_deadline();

            cpu_budget = icount_percpu_budget(cpu_count);
        }

        replay_mutex_unlock();

        if (!cpu) {
            cpu = first_cpu;
        }

        while (cpu && cpu_work_list_empty(cpu) && !cpu->exit_request) {
            /* Store rr_current_cpu before evaluating cpu_can_run().  */
            qatomic_set_mb(&rr_current_cpu, cpu);

            current_cpu = cpu;

            qemu_clock_enable(QEMU_CLOCK_VIRTUAL,
                              (cpu->singlestep_enabled & SSTEP_NOTIMER) == 0);

            if (cpu_can_run(cpu)) {
                int r;

                qemu_mutex_unlock_iothread();
                if (icount_enabled()) {
                    icount_prepare_for_run(cpu, cpu_budget);
                }
                r = tcg_cpus_exec(cpu);
                if (icount_enabled()) {
                    icount_process_data(cpu);
                }
                qemu_mutex_lock_iothread();

                if (r == EXCP_DEBUG) {
                    cpu_handle_guest_debug(cpu);
                    break;
                } else if (r == EXCP_ATOMIC) {
                    qemu_mutex_unlock_iothread();
                    cpu_exec_step_atomic(cpu);
                    qemu_mutex_lock_iothread();
                    break;
                }
            } else if (cpu->stop) {
                if (cpu->unplug) {
                    cpu = CPU_NEXT(cpu);
                }
                break;
            }

            cpu = CPU_NEXT(cpu);
        } /* while (cpu && !cpu->exit_request).. */

        /* Does not need a memory barrier because a spurious wakeup is okay.  */
        qatomic_set(&rr_current_cpu, NULL);

        if (cpu && cpu->exit_request) {
            qatomic_set_mb(&cpu->exit_request, 0);
        }

        idle = icount_enabled() && all_cpu_threads_idle();
        if (idle) {
#ifdef EMSCRIPTEN
            if (rr_warp_here()) {
                continue;
            }
#endif
            /*
             * When all cpus are sleeping (e.g in WFI), to avoid a deadlock
             * in the main_loop, wake it up in order to start the warp timer.
             */
            qemu_notify_event();
        }

        rr_wait_io_event();
#ifdef EMSCRIPTEN
        if (idle) {
            rr_last_handoff_ns = get_clock();
        }
#endif
        rr_deal_with_unplugged_cpus();
    }

    rcu_remove_force_rcu_notifier(&force_rcu);
    rcu_unregister_thread();
    return NULL;
}

void rr_start_vcpu_thread(CPUState *cpu)
{
    char thread_name[VCPU_THREAD_NAME_SIZE];
    static QemuCond *single_tcg_halt_cond;
    static QemuThread *single_tcg_cpu_thread;

    g_assert(tcg_enabled());
    tcg_cpu_init_cflags(cpu, false);

    if (!single_tcg_cpu_thread) {
        cpu->thread = g_new0(QemuThread, 1);
        cpu->halt_cond = g_new0(QemuCond, 1);
        qemu_cond_init(cpu->halt_cond);

        /* share a single thread for all cpus with TCG */
        snprintf(thread_name, VCPU_THREAD_NAME_SIZE, "ALL CPUs/TCG");
        qemu_thread_create(cpu->thread, thread_name,
                           rr_cpu_thread_fn,
                           cpu, QEMU_THREAD_JOINABLE);

        single_tcg_halt_cond = cpu->halt_cond;
        single_tcg_cpu_thread = cpu->thread;
    } else {
        /* we share the thread */
        cpu->thread = single_tcg_cpu_thread;
        cpu->halt_cond = single_tcg_halt_cond;
        cpu->thread_id = first_cpu->thread_id;
        cpu->neg.can_do_io = 1;
        cpu->created = true;
    }
}
