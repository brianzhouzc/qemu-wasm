# Code review: qemu-wasm, branch `wasm`

Scope: the 24 commits on `wasm` on top of ktock's `master` (`0ef7b4e..df2b8ed`),
all yours. That covers the Wasm TCG backend fixes, the Emscripten main-loop and
vCPU changes, savevm-to-file and the snapshot API, the browser exports (LCD
frame, keys), the vmstate additions, and the imported iPod Classic machine
(`2d384bf`: `hw/arm/ipod_classic/*`, `hw/intc/pl192.c`, the PL080 changes,
`hw/arm/ipod_touch_sha1.c`). The imported device models (about 11,000 lines)
were read for memory safety and guest-reachable crashes rather than line by
line for modelling accuracy. The front end has its own review on the
`review/code-review` branch of iPodWasm; findings that cross the boundary point
at it as `iPodWasm Wn`.

Each finding gives a location, what is wrong, how it goes wrong, and a
suggested fix. Tick the box when it is done.

Severity:

- **High**: memory corruption, or wrong code generated, on a path that exists
  today.
- **Medium**: a real bug on a reachable but less common path (restore, reset,
  a guest that programs a register oddly), or a fix that does not cover the
  path it claims to.
- **Low**: cleanup, performance, latent issues, stale comments.

| ID | Sev | Summary |
|---|---|---|
| Q1 | High | Wheel: the browser fields sit before `parent_obj` and overwrite the QOM header |
| Q2 | High | SHA-1 model: guest-controlled heap overflow |
| Q3 | High | PL192: the priority stack overflows into the struct |
| Q4 | High | Wasm backend: the `add2_i32` fix breaks `add2_i64` |
| Q5 | High | Wasm backend: `sextract_i32` emits an invalid module |
| Q6 | Medium | ATA vmstate leaves out the IDE drive state |
| Q7 | Medium | PL192 vmstate drops `rawintr`, so pending interrupts are lost on restore |
| Q8 | Medium | The JIT can never run under `-icount`; `wasm-instantiate` is dead |
| Q9 | Medium | The vCPU wake-up fix is not on the `-icount` (RR) path |
| Q10 | Medium | Main-loop futex: lost-wakeup window |
| Q11 | Medium | Browser keys run QEMU code and arm timers on the page thread |
| Q12 | Medium | Key ack is never set for the wheel and clears itself for buttons |
| Q13 | Medium | A failed snapshot load exits QEMU; dead `qemu_loadvm_state_from_file` |
| Q14 | Medium | Timer: guest-controlled divide by zero |
| Q15 | Medium | Exynos UART `post_load` change affects every Exynos board |
| Q16 | Low | Snapshot request handoff is not atomic |
| Q17 | Low | LCD export: the frame number advances when nothing changed; torn reads |
| Q18 | Low | `getenv()` in interrupt and MMIO hot paths |
| Q19 | Low | `WHEELTRACE` now prints every edge, against the file's own rule |
| Q20 | Low | Browser key state, `iicds_new` and `paced_pending` are not migrated |
| Q21 | Low | Semihosting is on by default |
| Q22 | Low | Guest-triggerable `hw_error()`/`assert()` aborts |
| Q23 | Low | Build files: unchecked downloads, `wasm32`-only fiber |
| Q24 | Low | Dead code, debug leftovers, small inconsistencies |

---

## High

### Q1. Wheel: the browser fields sit before `parent_obj` and overwrite the QOM header

- [x] Fixed

**Where:** `include/hw/arm/ipod_classic/ipod_classic_wheel.h:298-308`; used by
`hw/arm/ipod_classic/ipod_classic_wheel.c:142-147`, `:261-268` and `:591-631`.

**Problem.** The `#ifdef EMSCRIPTEN` block is the first thing in
`struct IPodClassicWheelState`, ahead of `SysBusDevice parent_obj`. QOM
requires the parent to be the first member: `OBJECT(s)`, `DEVICE(s)` and
`SYS_BUS_DEVICE(s)` are plain pointer casts, and `object_initialize()` writes
the `Object`/`DeviceState` header at offset 0 of the allocation. On wasm32 the
fields alias the header like this:

| wheel field | offset | overlaps |
|---|---|---|
| `wasm_keycode` | 0 | `Object.class` |
| `wasm_release_at` | 8 | `Object.properties`, `Object.ref` |
| `wasm_release_cap` | 16 | `Object.parent`, `DeviceState.id` |
| `wasm_press_mask` | 24 | `DeviceState.canonical_path` |
| `wasm_ack` | 28 | `DeviceState.realized`, `pending_deleted_event` |

**What goes wrong.**
- On the very first `ipod_wasm_key_press`, `wasm_release_at` is read as
  "a release is pending" (it holds the `properties` pointer and the refcount),
  so the code injects a release of `wasm_keycode | 0x80`, a keycode made from
  the low bits of the class pointer. It then zeroes `properties` and `ref`.
- From then on, `Object.class` holds the last keycode (e.g. `0x1c`), `ref`
  holds the high word of a virtual-time deadline (and 0 after each release),
  and `realized` follows the ack.
- The next QOM access to the wheel dereferences garbage. A guest software
  reset is enough: `SWRCON` → `qemu_system_reset_request`
  (`ipod_classic_clk.c:142`) → the resettable walk calls `object_get_class()`
  on the wheel. Any `object_ref`/`object_unref` on the wheel (for example from
  a FlatView rebuild) would free it once `ref` reads 0.

Native builds are unaffected; the block is compiled only for Emscripten.

**Fix.** Move the `#ifdef EMSCRIPTEN` block below `parent_obj`, ideally to the
end of the struct.

### Q2. SHA-1 model: guest-controlled heap overflow

- [x] Fixed

**Where:** `hw/arm/ipod_touch_sha1.c:129-135` (`flush_hw_buffer`), `:297-311`
(memory mode), `:325-327` (`SHA_INSIZE`), `:170` and `:277` (digest reads).

**Problem.** `buffer_ind` only grows; nothing checks it against
`SHA1_BUFFER_SIZE` (5 MiB).
- Each `SHA_CONFIG` write of 0x2 or 0xa in memory mode appends another
  `insize` bytes, and each flush of the hardware buffer appends 64, with no
  bound on the total.
- The only check is `assert(value <= SHA1_BUFFER_SIZE)` on `SHA_INSIZE`. The
  wasm build compiles it out (`-DNDEBUG`), and in a native build it lets the
  guest abort QEMU.

**What goes wrong.** A guest that hashes more than 5 MiB without writing
`SHA_RESET` (for example several large hashes in a row, or one 6 MiB
`insize` in the wasm build) writes past `buffer` into `hw_buffer`,
`buffer_ind` and `hashout`, then off the end of the object into the heap. That
is a host memory corruption driven by guest register writes. Separately, the
`case 0x20 ... 0x34` read returns 4 bytes from `hashout[offset - 0x20]`, so
offsets 0x31-0x34 (and any unaligned offset, since the ops set no `.valid`
sizes) read past the 20-byte `hashout`.

**Fix.** Before every append, check
`buffer_ind + n <= SHA1_BUFFER_SIZE`; on overflow, log `LOG_GUEST_ERROR` and
drop the data. Replace the `assert` with the same. Restrict the digest case to
`0x20 ... 0x33` with 4-byte aligned access (`.valid.min_access_size = 4`,
`.valid.max_access_size = 4`). While here, `#define SHA1_BUFFER_SIZE` needs
parentheses.

### Q3. PL192: the priority stack overflows into the struct

- [x] Fixed

**Where:** `hw/intc/pl192.c:144-157` (`pl192_mask_priority`), `:173-193`
(`pl192_irq_ack`); `include/hw/intc/pl192.h:60-61`.

**Problem.** `priority_stack` and `irq_stack` hold `PL192_PRIO_LEVELS + 1`
(17) entries, but the guard is `stack_i >= PL192_INT_SOURCES` (32). Every read
of `VECTADDR` pushes, and so does a read of `SOFTINTCLEAR`, which falls through
to the ack. Also, an ack with nothing pending sets
`current = current_highest = PL192_NO_IRQ` (33) and then reads
`vect_priority[33]` from a 32-entry array.

**What goes wrong.** Seventeen `VECTADDR` reads without a matching write
(nested handlers, a dispatcher that reads it twice, a read-only probe) write
past both arrays into `priority`, `daisy_vectaddr`, `daisy_priority`, the
`daisy_callback` pointer and the `irq`/`fiq` `qemu_irq` pointers. The next
interrupt then calls through a corrupted pointer. `pl192.c` came in with the
import from the qemu-ios lineage, but it is new to this tree.

**Fix.** Guard with `s->stack_i >= PL192_PRIO_LEVELS` and log
`LOG_GUEST_ERROR` instead of calling `hw_error`. In `pl192_irq_ack`, if
`current_highest >= PL192_INT_SOURCES` and it is not the daisy line, return the
default vector without pushing.

### Q4. Wasm backend: the `add2_i32` fix breaks `add2_i64`

- [x] Fixed (reopened in the second pass)

**Where:** `tcg/wasm32/tcg-target.c.inc:1387-1432` (`tcg_wasm_out_add2`),
called for both opcodes from `:3309-3314` and `:3727-3730`.

**Problem.** `tcg_wasm_out_add2` serves `INDEX_op_add2_i32` and
`INDEX_op_add2_i64` (`TCG_TARGET_HAS_add2_i64` is 1 and
`TCG_TARGET_REG_BITS` is 64). The fix in `9893dbf` masks every half to 32
bits and takes the carry from bit 32, which is correct for i32 and wrong for
i64. The old code, which took the carry with a 64-bit unsigned compare, was
right for i64.

**What goes wrong.** Every `add2_i64` now truncates both halves to 32 bits.
The 32-bit ARM front end does not emit `add2_i64`, which is why the iPod boots.
The backend is shared, though: any 64-bit guest (x86_64 `ADCX`/`ADOX` through
`tcg_gen_add2_tl`, AArch64 `ADCS`, 128-bit helpers) gets wrong arithmetic, and
the bug would ship with an upstream PR of this fix.

**Fix.** Pass the type in (as `tcg_wasm_out_extract` does). Keep the new
masked sequence for `TCG_TYPE_I32` and restore the previous 64-bit sequence for
`TCG_TYPE_I64`.

### Q5. Wasm backend: `sextract_i32` emits an invalid module

- [x] Fixed

**Where:** `tcg/wasm32/tcg-target.c.inc:1591-1601` (`tcg_wasm_out_extract`).

**Problem.** For a signed i32 extract the emitted sequence is:

```
global.get arg; i64.const sl; i64.shl; i64.const 0xffffffff; i64.and
i64.const rs           ; pushed here
i64.extend_i32_s       ; 0xAC: takes an i32, but the top of stack is rs (i64)
i64.shr_s
```

`i64.extend_i32_s` consumes the i64 shift amount, so the module fails
validation (type mismatch). Even with an `i64.extend32_s` instead, it would
extend the shift amount, not the value.

**What goes wrong.** Any block containing `sextract_i32` cannot be compiled
into a module. For ARM it comes from `SBFX` (v6T2 and later,
`target/arm/tcg/translate.c:7230`) and the EL2 path at `:2812`. The ARM926
guest never executes either, so this is latent for the iPod but real for any
v7 guest.

**Fix.** Sign-extend before pushing the shift amount:
`... i64.and; i32.wrap_i64; i64.extend_i32_s; i64.const rs; i64.shr_s`. The
`i64.and` is then redundant on that path.

---

## Medium

### Q6. ATA vmstate leaves out the IDE drive state

- [x] Fixed (reopened in the second pass)

**Where:** `hw/arm/ipod_classic/ipod_classic_ata.c:534-549`.

**Problem.** The vmstate has `VMSTATE_IDE_BUS(bus, ...)`, which saves only
`cmd`, `unit` and the error-status subsection (`hw/ide/core.c:3010-3022`).
Every other IDE controller pairs it with
`VMSTATE_IDE_DRIVES(bus.ifs, ...)`, for example `hw/ide/isa.c:62-63` and
`hw/ide/mmio.c:114-115`.

**What goes wrong.** After a restore the drive is in its reset state: the
task-file registers, `mult_sectors` (SET MULTIPLE), `identify_data` changes
from SET FEATURES (write cache, transfer mode), LBA48 HOB registers and any
PIO position are gone. `ide_drive_post_load` (which re-applies the write-cache
setting) never runs. A guest that changed any of these during boot sees a
different disk after a restore.

**Fix.** Add `VMSTATE_IDE_DRIVES(bus.ifs, IPodClassicATAState)` and bump
`version_id`.

### Q7. PL192 vmstate drops `rawintr`, so pending interrupts are lost on restore

- [x] Fixed

**Where:** `hw/intc/pl192.c:389-436`.

**Problem.** The comment says devices re-assert their lines from their own
`post_load`. In this machine only the wheel has a `post_load` that does that
(`ipod_classic_wheel.c:696-702`). ATA, timer, I2C, LCD, the PL080s and GPIO
do not, and QEMU does not migrate `qemu_irq` levels: the interrupt
controller's record of its input lines is the only copy.

**What goes wrong.** Any level interrupt asserted at the save point (timer
status not yet cleared, ATA INTRQ, a DMA terminal count) is gone after the
restore while the device still believes it is asserted. The guest waits for an
interrupt that never comes. The UART storm that motivated dropping `rawintr`
(`7e20022`) is already fixed at its source by `21812f3`.

**Fix.** Migrate `rawintr` (the `post_load` that recomputes `irq_status` and
`fiq_status` stays). If a device line really is not re-driven after restore,
fix that device rather than the controller.

### Q8. The JIT can never run under `-icount`; `wasm-instantiate` is dead

- [x] Fixed (or documented)

**Where:** `tcg/wasm32.c:274-287`; `hw/arm/ipod_classic/ipod_classic.c:772-775`.

**Problem.** `init_wasm32()` runs on the vCPU thread, which starts after
machine init has applied `wasm-instantiate`, and it sets
`instantiate_num = INT32_MAX` whenever `icount_enabled()`. The harness always
passes `-icount`.

**What goes wrong.** With the committed tree the default "JIT" build never
compiles a block: `wasm-instantiate=100` from the page is overwritten, and the
heartbeat's `wasm_tb` stays 0. The 57.5 s JIT boot recorded in `5e8a7ff` and
iPodWasm's `docs/architecture.md` cannot be reproduced from these sources (it
was presumably measured with the guard edited out locally, which
`tools/mirror.sh` carries). The comment says the JIT and icount "do not mix
yet", while the docs say the JIT is fixed and merely slower.

**Fix.** Pick one:
- Remove the guard now that `a6bd842` and `9893dbf` are in, and let
  `wasm-instantiate` (0 = default) decide.
- Keep it, but let an explicit `wasm-instantiate` override it, and update the
  comment, README and `architecture.md` to say icount turns the JIT off.

### Q9. The vCPU wake-up fix is not on the `-icount` (RR) path

- [x] Fixed

**Where:** `system/cpus.c:435-448`; `accel/tcg/tcg-accel-ops-rr.c:118-125`.

**Problem.** The Emscripten change replaces `qemu_cond_wait` in
`qemu_wait_io_event()`. That function is used by the MTTCG, KVM, HVF and dummy
vCPU threads. `-icount` forces single-threaded RR TCG, whose thread waits in
`rr_wait_io_event()` on `qemu_cond_wait_iothread(first_cpu->halt_cond)`,
unchanged.

**What goes wrong.** The machine this branch is for never executes the new
code. If the lost condition-variable wake-up described in `9c371fe` is real,
it can still happen on the iPod's vCPU; if it was really the main-loop issue
(Q10's futex fixed that), the `cpus.c` change and its comment describe a
behaviour that is not what fixed the boot.

**Fix.** Apply the same bounded poll in `rr_wait_io_event()`, or drop the
`cpus.c` change and correct the commit's claim in the research notes.

### Q10. Main-loop futex: lost-wakeup window

- [x] Fixed

**Where:** `util/main-loop.c:55-72` (`qemu_em_notify_wait`), `:349-355`.

**Problem.** `expected` is read inside `qemu_em_notify_wait()`, after
`qemu_poll_ns(..., 0)` has returned 0. A `qemu_notify_event()` between the
poll and that read bumps the counter before `expected` is sampled, so the wait
does not see it.

**What goes wrong.** The main loop sleeps the full bound (up to 10 ms) with
work pending. The comment accepts this as the cost of a missed notification,
but it is avoidable, and under `-icount sleep=off` the extra main-loop latency
is one of the nondeterminism sources the research notes chase. Also, a BH
scheduled from another thread wakes the loop through `aio_notify()`, which
does not touch the futex, so it waits the same 10 ms.

**Fix.** Sample the counter before the non-blocking poll (before releasing the
BQL is simplest) and pass it to the wait:
`int seen = qatomic_read(&qemu_em_notify); ... poll ...; if (ret == 0) futex_wait(&qemu_em_notify, seen, ms)`.
Bumping the futex from `aio_notify()` on the main context would cover BHs
too.

### Q11. Browser keys run QEMU code and arm timers on the page thread

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic_wheel.c:571-648`.

**Problem.** `ipod_wasm_key_press()` (and `ipod_wasm_key()`) run on the
browser's main thread. They take the BQL there and call `wheel_key_event()`,
which calls `wheel_kick()` → `timer_mod()` → `qemu_timer_notify_cb()` →
`async_run_on_cpu()`. The comment at `:586-589` says the page thread must not
create or arm QEMU timers because "Emscripten pthread atomics trap there".

**What goes wrong.** It works today, but against the stated rule:
- The browser main thread cannot `Atomics.wait`, so `qemu_mutex_lock_iothread`
  busy-waits for as long as the main loop holds the BQL. A save writes about
  19 MB with the BQL held, so a key press during a save freezes the tab for
  that long.
- QEMU code runs on a thread QEMU did not create: no RCU registration, no
  `current_cpu`, and none of the thread-locals the timer and CPU code expect.

**Fix.** Do what `ipod_snapshot_request()` does: record the key in a small
lock-free mailbox (atomic stores), schedule a BH or notify the main loop, and
call `wheel_key_event()` from the main loop with the BQL held normally.

### Q12. Key ack is never set for the wheel and clears itself for buttons

- [x] Fixed (with iPodWasm W4)

**Where:** `hw/arm/ipod_classic/ipod_classic_wheel.c:142-147` (pop),
`:261-268` (tick), `:606-616` (press), `:631-634` (`ipod_wasm_key_ack`).

**Problem.**
- `wasm_press_mask = buttons` after the key event. For `q`/`e` (the wheel)
  `buttons` does not change and is normally 0, so
  `(word >> BTN_SHIFT) & wasm_press_mask` is never non-zero and `wasm_ack`
  is never set.
- For buttons, the tick clears `wasm_ack` in the same step that performs the
  release, so the ack is visible only between consumption and the next tick.
  The page polls every 20 ms of wall time.

**What goes wrong.** The page's `turn()` waits its 2 s fallback for every
detent, and a button press can be missed by the poll and also fall back to
2 s. See iPodWasm W4.

**Fix.** Export a monotonic counter instead of a flag
(`uint32_t wasm_consumed`, read with `qatomic_read`). Increment it when the
guest pops a packet carrying the pressed button, and for wheel keys when the
tick sends the packet that pays out the last pending step. The page records
the counter before the press and waits for it to change.

### Q13. A failed snapshot load exits QEMU; dead `qemu_loadvm_state_from_file`

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic.c:671-690` and `:700-714`;
`migration/migration.c:712-721`; `migration/savevm.c:1794-1816`.

**Problem.** Loads go through `qmp_migrate_incoming("file:...")`. In 8.2,
`process_incoming_migration_co()` ends a failed load with
`exit(EXIT_FAILURE)`. `ipod_snapshot_poll()`'s failure branch
(`migration_is_idle()` after 8 s) can therefore only catch a load that never
started, never one that failed.

**What goes wrong.** A snapshot from an older build (any vmstate change, Q6 or
Q7 included) kills the emulator. The page does not learn this from
`ipod_snapshot_status()` and times out after 120 s (iPodWasm W2).

Separately, `qemu_loadvm_state_from_file()` is unused. If anything called it,
it would call `migration_incoming_state_destroy()` without the yank instance
ever being registered, so `yank_unregister_instance()` asserts natively and
dereferences NULL in the NDEBUG wasm build.

**Fix.**
- Write a small header or sidecar with the build id when saving, and refuse
  (state 3) a file from another build before calling
  `qmp_migrate_incoming`.
- Optionally, for this machine, turn the `exit()` into a failed state the page
  can see. QEMU 9.1 added `exit-on-error=false` to `migrate-incoming` for
  exactly this; backporting that is the clean way.
- Delete `qemu_loadvm_state_from_file()` and its declaration.

### Q14. Timer: guest-controlled divide by zero

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic_timer.c:140` (`chan_freq`) and
`:199` (`ptimer_set_freq`).

**Problem.** `return base / div / (c->pre + 1);` is computed in 32 bits.
- `PRE = 0xffffffff` makes `pre + 1` zero, so this divides by zero.
- Any `PRE >= base / div` makes the result 0, and `ptimer_set_freq(..., 0)`
  divides by zero computing the period.

**What goes wrong.** One guest write to a timer's `PRE` register crashes QEMU:
SIGFPE natively, a trap that kills the worker in wasm. `chan_reload()` runs on
every `PRE` write, so the value does not even need to be used.

**Fix.** Compute in 64 bits; if the result is 0, log `LOG_GUEST_ERROR` and
clamp to 1 Hz (or stop the ptimer).

### Q15. Exynos UART `post_load` change affects every Exynos board

- [x] Fixed

**Where:** `hw/char/exynos4210_uart.c:619-633`.

**Problem.** `21812f3` replaces the upstream `exynos4210_uart_rx_timeout_set()`
with an unconditional `timer_del()`. This model is shared by every Exynos4
board in the tree (`smdkc210`, `nuri`).

**What goes wrong.** On those boards, a guest migrated with bytes in the RX
FIFO below the trigger level never gets the RX timeout interrupt, so the tail
of a transfer stalls until more data arrives. That is the case the upstream
re-arm exists for.

**Fix.** Narrow it: re-arm only when the RX FIFO holds data
(`fifo_elements_number(&s->rx) > 0`), otherwise `timer_del()`. That keeps
the upstream behaviour and still avoids the empty-FIFO storm on the iPod.

---

## Low

### Q16. Snapshot request handoff is not atomic

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic.c:660-665` and `:730-746`.

`ipod_snapshot_request()` writes `ipod_snapshot_path` and `ipod_snapshot_op`
with plain stores on the browser thread, then `ipod_snapshot_state`
(`volatile`). The BH reads them on the main-loop thread. `volatile` orders
volatile accesses only, so the compiler may sink the `op` or `path` stores
past the `state` store. The BH could then see `op == 0` and return, leaving
`state == 1` forever (the page times out after 120 s). Use `qatomic_set()` for
`op`/`path` followed by `qatomic_store_release(&state, 1)`, and
`qatomic_load_acquire()` in the BH. Re-requests while `state == 1` are
silently dropped; returning an error code would let the page tell that apart
from success.

### Q17. LCD export: the frame number advances when nothing changed; torn reads

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic_lcd.c:648-663`; comments at
`:637-640` and `:756-759`.

- `wasm_lcd_frame_tick()` sets `s->dirty = true` before composing, so
  `ipod_classic_lcd_compose()` always returns true and `wasm_seq` advances 30
  times a second regardless of the picture. The page's "convert only when the
  sequence changed" optimisation never skips anything (76,800 pixels per
  frame).
- The page reads `s->fb` from the browser thread while the main loop may be
  composing into it, so it can see half of one frame and half of the next. A
  torn frame looks like a change to `waitStable`.
- The block comment says 60 Hz; the first arm is 1/60 s and the tick is 1/30 s.

Compose into a back buffer, `memcmp` it with the published one, and only on a
difference copy it over (or swap pointers) and bump `wasm_seq` with a release
store.

### Q18. `getenv()` in interrupt and MMIO hot paths

- [x] Fixed

**Where:** `hw/intc/pl192.c:179` and `:360`; `hw/arm/ipod_touch_sha1.c:235`
and `:274`; `hw/arm/ipod_classic/ipod_classic_ata.c:140` and `:248`;
`hw/arm/ipod_classic/ipod_classic_timer.c:184`.

`pl192_irq_handler()` runs on every `qemu_set_irq()` into the VIC, and
`pl192_irq_ack()` on every acknowledge; both call `getenv()`, which scans the
environment each time. Under TCI in the browser that is measurable. Cache them
the way `wheel_trace_on()` and `pl080_trace_on()` do. Also,
`static int dbg_n` in the `VICACK` trace is signed and incremented without
bound.

### Q19. `WHEELTRACE` now prints every edge, against the file's own rule

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic_wheel.c:101-104` and `:362-364`;
the rule at `:24-27` and `:41-46`.

The header says the trace announces only the first of each event "because
stderr is where the firmware's own semihosting output goes and it has to stay
readable". The wasm-branch changes print every IRQ level change and every
keycode. Either restore first-only announcements or update the rule (for
example `WHEELTRACE=2` for the verbose form).

### Q20. Browser key state, `iicds_new` and `paced_pending` are not migrated

- [x] Fixed (or accepted)

- The wheel's `wasm_*` release state is not in its vmstate (`:705-735`). A
  snapshot taken while a browser press is held restores with the button bit
  set in `buttons` and no release scheduled: a stuck key.
- `iicds_new` (`ipod_classic_i2c.c:159-178`) is not in the I2C vmstate
  (`:299-316`). A save between the guest's `IICDS` write and its go bit drops
  the byte after restore.
- PL080 `paced_pending` (`include/hw/dma/pl080.h:66-84`) is not in
  `vmstate_pl080`. A paced channel saved in the completed-pending state
  ignores the guest's relink after restore.

Today's snapshots are taken at the end of boot, so none of these is likely,
but every user-initiated save (if one is added later) can hit them. Add them as
subsections.

### Q21. Semihosting is on by default

- [x] Fixed (or accepted)

**Where:** `hw/arm/ipod_classic/ipod_classic.c:368-389`.

The default gives the guest the whole ARM semihosting interface, not just the
console writes the comment needs: `SYS_OPEN`, `SYS_WRITE`, `SYS_REMOVE`,
`SYS_RENAME` and `SYS_SYSTEM` on the host. In the browser that is MEMFS and
harmless. In the native emulator, running a firmware build someone else made
lets it read and write the user's files and run shell commands. Consider
defaulting it on only for Emscripten, or leaving it on but documenting
`-semihosting-config` and the risk next to the property.

### Q22. Guest-triggerable `hw_error()`/`assert()` aborts

- [x] Fixed (reopened in the second pass)

**Where:** `hw/intc/pl192.c:147`, `:285-291`, `:309-312`;
`hw/dma/pl080.c:202-205`; `hw/arm/ipod_touch_sha1.c:326`.

An unaligned VIC write, a write to `RAWINTR`/`FIQSTATUS`/the ID registers, a
PL080 flow-control value of 4 or more, or an oversized `SHA_INSIZE` all
abort QEMU from a guest register write. QEMU's convention is
`qemu_log_mask(LOG_GUEST_ERROR, ...)` and ignore the access. The PL080 one is
upstream code; the others came in with this branch.

### Q23. Build files: unchecked downloads, `wasm32`-only fiber

- [x] Fixed

- `Dockerfile:39`: `curl -Ls ... | tar xJ` has no `-f`, so a 404 page is piped
  into `tar` and the error is confusing; and nothing verifies the tarball.
  Use `curl -fsSL -o` plus a `sha256sum -c`. (The glib download a few lines
  later uses `-k`, which disables TLS verification; that one is upstream but
  worth fixing while you are there.)
- `meson.build:326`: fiber is selected only for `cpu_family() == 'wasm32'`.
  ktock maintains wasm64 branches; `host_machine.cpu_family().startswith('wasm')`
  keeps the native fix and does not break them.

### Q24. Dead code, debug leftovers, small inconsistencies

- [x] Fixed

- `hw/dma/pl080.c:602-605`: a second MMIO region (`iomem2`) is registered on
  every PL080 in the tree (Versatile, RealView and the rest), but the iPod
  machine never maps it. Map it or drop it; the rename `iomem` to `iomem1`
  also changes a struct other boards include.
- `hw/dma/pl080.c`: commented-out `printf`/`fprintf` lines at the transfer,
  interrupt and register-write sites.
- `hw/arm/ipod_classic/ipod_classic_ata.c:140-200`: the `ATALBA` trace
  comments are in Spanish; the rest of the machine is in English.
- `hw/arm/ipod_classic/ipod_classic.c:607`: the heartbeat comment mentions
  `IPOD_HEARTBEAT_MS`, which only `node-run.js` reads; QEMU takes the
  `heartbeat=` machine property. `heartbeat` is a `uint32` property passed to
  `ipod_heartbeat_start(int ms)` (`:638`), so values of 2^31 or more turn it
  off. The heartbeat also reads `env.regs[15]` and the CPSR from the
  main-loop thread while the vCPU runs; fine for diagnostics, but the numbers
  can be torn.
- `hw/arm/ipod_classic/ipod_classic.c:341-351`: the `bootrom` setter calls
  `exit(1)` instead of setting `errp`.

---

# Second pass: review of the fixes

Scope: the 25 commits from `2ff5cc8` to `2e153d4`. Most fixes are right
(Q1, Q2, Q3, Q5, Q7's intent, Q10, Q11, Q12, Q14, Q15, Q16, Q17). Three fixes
do not do what they say, and Q4, Q6 and Q22 are reopened above. The front
end's second pass is in the iPodWasm `REVIEW.md` (items `R1`-`R12`). Its R2
matters here: the page passes `wasm-instantiate=100`, so after `5824acb` the
JIT is on under icount by default.

| ID | Sev | Summary | Reopens |
|---|---|---|---|
| S1 | High | The IDE drives are still not migrated: the field is version-gated out | Q6 |
| S2 | Medium | `add2_i64` reads `al` after writing `retl`, which may be the same register | Q4 |
| S3 | Medium | The PL080 flow-control `return` wedges the controller for good | Q22 |
| S4 | Medium | New vmstate fields are not version-gated; PL080 changes upstream's wire format | Q7, Q20 |
| S5 | Medium | The snapshot sidecar is written after "done" is published | Q13, Q16 |
| S6 | Low | The key mailbox has one slot | Q11 |
| S7 | Low | The SHA-1 block now rejects every non-word access | Q2 |
| S8 | Low | `static char mode = -1` turns the ATALBA trace on for Arm hosts | Q18 |
| S9 | Low | The browser-only wheel fields change the stream layout at the same version | Q20 |
| S10 | Low | The "unknown" build id matches every other "unknown" | Q13 |

## High

### S1. The IDE drives are still not migrated: the field is version-gated out

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic_ata.c:546` and `:556`;
`include/hw/ide/internal.h:588-589`; `migration/vmstate.c:31-49`.

`VMSTATE_IDE_DRIVES` expands to `VMSTATE_STRUCT_ARRAY(..., 2, 3, ...)`: its
field `version_id` is 3. `vmstate_field_exists()` saves and loads a field only
when `field->version_id <= version_id`, and the ATA description is version 2.
So the drives are skipped on save and on load, exactly as before the fix.
This is why the ISA, MMIO and PCI IDE descriptions are all at version 3.

**Fix.** Set `.version_id = 3` (keep `.minimum_version_id = 1`: older streams
then simply lack the drives, which is what they contain). Then check with a
save, a load and `info qtree` (or a trace of `vmstate_load_field`) that the
`ifs` section is actually in the stream.

## Medium

### S2. `add2_i64` reads `al` after writing `retl`, which may be the same register

- [x] Fixed

**Where:** `tcg/wasm32/tcg-target.c.inc:1404-1429`; constraint at `:148-152`.

The new i64 path stores the low sum into `retl` (`:1417`) and then compares
it against `al` (`:1420-1422`) for the carry. `add2` is constrained
`C_O2_I4(r, r, r, r, r, r)` with no early clobber, so the register allocator
may give `retl` the same register as `al`. The compare then reads `low < low`
and the carry is always 0. The old code handled exactly this case
(`al == retl`).

**Fix.** Compute the carry from the locals before any global is written:
`TMP1 < al` while `al` is intact, keep it on the stack or in `TMP64_2`, then
store `retl` and `reth`.

### S3. The PL080 flow-control `return` wedges the controller for good

- [x] Fixed

**Where:** `hw/dma/pl080.c:200-209`; the `running` handshake at `:174-180`
and `:333-335`.

`pl080_run()` sets `s->running = 1` and clears it only at the bottom of its
loop. The new `return` leaves `running` at 1. Every later call sees
`running != 0`, increments it and returns, so no channel on that DMAC moves
again until reset, and `pl080_update()` is skipped too. That is worse than the
abort it replaced, and silent after the first log line.

**Fix.** `continue` (skip that channel) instead of `return`, and log once per
channel configuration rather than on every pass.

### S4. New vmstate fields are not version-gated; PL080 changes upstream's wire format

- [x] Fixed

**Where:** `hw/intc/pl192.c:463` (description still version 1);
`hw/arm/ipod_classic/ipod_classic_i2c.c:301` and `:312` (version 2, minimum
1); `hw/dma/pl080.c:54-71` (version 2, minimum 1).

Each description gained a field without a field version, while still
accepting older streams (or, for PL192, without a version bump at all). A
stream in the old layout is then read with every later field shifted, instead
of being refused. With the sidecar fallback accepting id-less files (S10),
that is reachable.
- PL080 also drops the three duplicate `tc_int` entries. They are upstream's
  wire format, shared by every PL080 board (Versatile, RealView, ...), and
  removing them breaks migration from any upstream build while still claiming
  version 1 compatibility.

**Fix.** Use `VMSTATE_UINT32_V(rawintr, PL192State, 2)` with PL192 at version
2, `VMSTATE_BOOL_V(iicds_new, ..., 2)`, and
`VMSTATE_UINT32_V(paced_pending, ..., 2)`. Keep PL080's duplicate entries (or
put `paced_pending` in a subsection). Alternatively, raise
`minimum_version_id` to the new version wherever old streams are not worth
reading.

### S5. The snapshot sidecar is written after "done" is published

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic.c:806-810`.

The save path sets `ipod_snapshot_state = 2` and only then writes
`state.bin.id`. The page and `node-steps.js` poll the state and read the files
as soon as they see 2. `node-steps.js:369-377` swallows a missing sidecar
("older build"), so a save can silently lose its build id. The store is also a
plain one, while everything else now uses atomics.

**Fix.** Write the sidecar first, then `qatomic_store_release(&ipod_snapshot_state, ret == 0 ? 2 : 3)`.

## Low

### S6. The key mailbox has one slot

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic_wheel.c:623-678`.

`wasm_key_queue()` overwrites `op`/`arg`/`hold`. Two calls before the BH runs
lose the first, for example quick clicks on the page's buttons (which call
`tapKey` without awaiting), or `ipod_wasm_key()` press-then-release. The three
fields can also be read torn if a new request arrives while the BH reads
them. A small ring (eight entries, atomic head/tail) fixes both. Separately,
for buttons the counter also advances on trailing packets of the previous,
still-held press, so a back-to-back press can be "acknowledged" early.
Returning a press id from `ipod_wasm_key_press()`, and reporting the last
consumed id, would make the acknowledgement exact.

### S7. The SHA-1 block now rejects every non-word access

- [x] Fixed (or accepted)

**Where:** `hw/arm/ipod_touch_sha1.c:380-387`.

`.valid.min_access_size = 4` applies to the whole 0x100-byte region,
including the 0x40-0x7c input buffer. A byte or halfword access is now a
decode error, which the guest takes as a data abort; before, it was accepted.
If the firmware ever does a `strb` there (a `memcpy` tail, say), this changes
behaviour. Checking `size == 4` in the digest case alone keeps Q2's fix
without that risk.

### S8. `static char mode = -1` turns the ATALBA trace on for Arm hosts

- [x] Fixed

**Where:** `hw/arm/ipod_classic/ipod_classic_ata.c:29-37`.

`char` is unsigned on Arm hosts, so `mode == -1` is never true. `mode` stays
255 and `ata_lba_mode()` is truthy, so a native build on an arm64 machine
prints the ATALBA trace for every DMA read. Use `int`.

### S9. The browser-only wheel fields change the stream layout at the same version

- [x] Fixed (or accepted)

**Where:** `hw/arm/ipod_classic/ipod_classic_wheel.c:792-803`.

The `#ifdef EMSCRIPTEN` fields sit in the main field list, so native and wasm
streams differ at version 3. Put them in a subsection that is only compiled
(and `needed`) in the wasm build. A cross load then fails on an unknown
subsection instead of misparsing.

### S10. The "unknown" build id matches every other "unknown"

- [x] Fixed (or accepted)

**Where:** `hw/arm/ipod_classic/ipod_classic.c:686-693` and `:708-733`.

A build without the generated header writes "unknown", and so does every
other such build. A sidecar saying "unknown" therefore passes the check
against any other unknown build, and a missing sidecar is accepted outright.
Refusing (or at least warning on) "unknown" on either side would keep the
check meaningful. The dirty-tree id collision is iPodWasm R11.
