/*
 * NV-Center Controller - Main Header
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-timed pulse sequencer for NV-center quantum sensing
 * on Teensy 4.1 (i.MX RT1062) with Zephyr RTOS.
 */

#ifndef NV_CONTROLLER_H
#define NV_CONTROLLER_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Version information
 */
#define NV_CONTROLLER_VERSION_MAJOR  1
#define NV_CONTROLLER_VERSION_MINOR  0
#define NV_CONTROLLER_VERSION_PATCH  0

/*
 * TTL Channel Definitions
 *
 * Each channel corresponds to a physical output for controlling
 * the NV-center sensing apparatus:
 *
 * - MW_I:   Microwave I (in-phase) component for spin manipulation
 * - MW_Q:   Microwave Q (quadrature) component for IQ modulation
 * - LASER:  Green laser (532nm) for initialization and readout
 * - MASTER: Master pulse for lock-in amplifier reference
 */
typedef enum {
    NV_CHANNEL_MW_I      = 0,  /* Microwave In-phase */
    NV_CHANNEL_MW_Q      = 1,  /* Microwave Quadrature */
    NV_CHANNEL_LASER     = 2,  /* Laser control */
    NV_CHANNEL_MASTER    = 3,  /* Master pulse (lock-in ref) */
    NV_CHANNEL_TRIG_OUT  = 4,  /* External trigger output */
    NV_CHANNEL_AUX_0     = 5,  /* Auxiliary channel 0 */
    NV_CHANNEL_AUX_1     = 6,  /* Auxiliary channel 1 */
    NV_CHANNEL_AUX_2     = 7,  /* Auxiliary channel 2 */
    NV_CHANNEL_COUNT     = 8   /* Total number of channels */
} nv_channel_t;

/* Channel bitmask type for simultaneous channel operations */
typedef uint8_t nv_channel_mask_t;

#define NV_MASK_MW_I      (1U << NV_CHANNEL_MW_I)
#define NV_MASK_MW_Q      (1U << NV_CHANNEL_MW_Q)
#define NV_MASK_LASER     (1U << NV_CHANNEL_LASER)
#define NV_MASK_MASTER    (1U << NV_CHANNEL_MASTER)
#define NV_MASK_TRIG_OUT  (1U << NV_CHANNEL_TRIG_OUT)
#define NV_MASK_ALL       ((1U << NV_CHANNEL_COUNT) - 1)

/*
 * System Status
 */
typedef enum {
    NV_STATUS_IDLE       = 0,  /* Ready, not running */
    NV_STATUS_ARMED      = 1,  /* Sequence loaded, awaiting trigger */
    NV_STATUS_RUNNING    = 2,  /* Sequence executing */
    NV_STATUS_PAUSED     = 3,  /* Sequence paused */
    NV_STATUS_ERROR      = 4,  /* Error condition */
    NV_STATUS_OVERFLOW   = 5,  /* Event buffer overflow */
} nv_status_t;

/*
 * Error Codes
 */
typedef enum {
    NV_OK                = 0,
    NV_ERR_INVALID_ARG   = -1,
    NV_ERR_NO_MEMORY     = -2,
    NV_ERR_BUSY          = -3,
    NV_ERR_NOT_READY     = -4,
    NV_ERR_OVERFLOW      = -5,
    NV_ERR_UNDERFLOW     = -6,
    NV_ERR_TIMEOUT       = -7,
    NV_ERR_HARDWARE      = -8,
    NV_ERR_PROTOCOL      = -9,
    NV_ERR_SEQUENCE      = -10,
} nv_error_t;

/*
 * Trigger Modes
 */
typedef enum {
    NV_TRIGGER_IMMEDIATE = 0,  /* Start immediately when armed */
    NV_TRIGGER_SOFTWARE  = 1,  /* Wait for software trigger command */
    NV_TRIGGER_EXTERNAL  = 2,  /* Wait for external trigger input */
    NV_TRIGGER_ADC_SYNC  = 3,  /* Synchronize with ADC_ETC */
} nv_trigger_mode_t;

/*
 * Timing Constants
 *
 * Based on i.MX RT1062 GPT timer at 150MHz input clock
 */
#define NV_TIMER_FREQ_HZ          150000000UL  /* 150 MHz */
#define NV_TIMER_TICK_NS          7            /* ~6.67ns per tick */
#define NV_MIN_PULSE_TICKS        2            /* Minimum pulse width */
#define NV_MAX_SEQUENCE_DURATION  0xFFFFFFFFUL /* 32-bit timer */

/* Convert nanoseconds to timer ticks */
static inline uint32_t nv_ns_to_ticks(uint32_t ns)
{
    return (uint32_t)(((uint64_t)ns * NV_TIMER_FREQ_HZ) / 1000000000ULL);
}

/* Convert timer ticks to nanoseconds */
static inline uint32_t nv_ticks_to_ns(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000000000ULL) / NV_TIMER_FREQ_HZ);
}

/* Convert microseconds to timer ticks */
static inline uint32_t nv_us_to_ticks(uint32_t us)
{
    return (uint32_t)(((uint64_t)us * NV_TIMER_FREQ_HZ) / 1000000ULL);
}

/*
 * System Information Structure
 */
typedef struct {
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
    uint8_t channel_count;
    uint32_t timer_freq_hz;
    uint32_t min_pulse_ns;
    uint32_t max_events;
    uint32_t dma_buffer_size;
} nv_system_info_t;

/*
 * Runtime Statistics
 */
typedef struct {
    uint32_t sequences_completed;
    uint32_t events_executed;
    uint32_t trigger_count;
    uint32_t overflow_count;
    uint32_t underrun_count;
    uint32_t max_latency_ticks;
    uint32_t last_sequence_duration_us;
} nv_statistics_t;

/*
 * Callback function types
 */
typedef void (*nv_sequence_callback_t)(nv_status_t status, void *user_data);
typedef void (*nv_trigger_callback_t)(uint32_t timestamp, void *user_data);
typedef void (*nv_error_callback_t)(nv_error_t error, void *user_data);

/*
 * Global system functions (implemented in main.c)
 */
nv_error_t nv_controller_init(void);
nv_error_t nv_controller_start(void);
nv_error_t nv_controller_stop(void);
nv_status_t nv_controller_get_status(void);
void nv_controller_get_info(nv_system_info_t *info);
void nv_controller_get_statistics(nv_statistics_t *stats);
void nv_controller_reset_statistics(void);

#ifdef __cplusplus
}
#endif

#endif /* NV_CONTROLLER_H */
