#ifndef HW_ARM_IPOD_TOUCH_SHA1_H
#define HW_ARM_IPOD_TOUCH_SHA1_H

#include "qemu/osdep.h"
#include "hw/platform-bus.h"
#include "hw/hw.h"
#include "exec/hwaddr.h"
#include "exec/memory.h"

/*
 * SHA-1 state for the S5L8702's hash block. This used to be OpenSSL's
 * SHA_CTX; OpenSSL is not part of the WebAssembly build and nothing else here
 * needs it, so hw/arm/ipod_touch_sha1.c implements the algorithm itself.
 */
typedef struct IPodTouchSHA1Ctx {
    uint32_t h[5];
    uint64_t bytes;
    uint8_t block[64];
} IPodTouchSHA1Ctx;

#define TYPE_IPOD_TOUCH_SHA1                "ipodtouch.sha1"
OBJECT_DECLARE_SIMPLE_TYPE(IPodTouchSHA1State, IPOD_TOUCH_SHA1)

#define SHA1_BUFFER_SIZE 1024 * 1024 * 5

#define SHA_CONFIG         0x0
#define SHA_RESET          0x4
#define SHA_MEMORY_MODE    0x80 // whether we read from the memory
#define SHA_MEMORY_START   0x84
#define SHA_INSIZE         0x8c

typedef struct IPodTouchSHA1State {
	SysBusDevice busdev;
    MemoryRegion iomem;
    uint32_t config;
    uint32_t memory_start;
    uint32_t memory_mode;
    uint32_t insize;
    uint8_t buffer[SHA1_BUFFER_SIZE];
    uint32_t hw_buffer[0x10]; // hardware buffer
    uint32_t buffer_ind;
    uint8_t hashout[0x14];
    bool hw_buffer_dirty;
    bool hash_computed;
    IPodTouchSHA1Ctx ctx;
} IPodTouchSHA1State;

#endif