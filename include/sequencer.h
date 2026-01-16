/*
 * NV-Center Controller - Pulse Sequencer
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-timed pulse sequencer using:
 * - GPT timer for precise timing (6.67ns resolution @ 150MHz)
 * - DMA for deterministic GPIO updates (no CPU jitter)
 * - XBAR for hardware signal routing
 *
 * Architecture:
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    Event Table (RAM)                        │
 * │  [t0, mask0] [t1, mask1] [t2, mask2] ... [tn, maskn]       │
 * └─────────────────────┬───────────────────────────────────────┘
 *                       │ DMA Transfer
 *                       ▼
 * ┌─────────────────────────────────────────────────────────────┐
 * │                   DMA Ping-Pong Buffer                      │
 * │  Buffer A ◄─────► Buffer B (double buffering)              │
 * └─────────────────────┬───────────────────────────────────────┘
 *                       │ GPT Compare Match Trigger
 *                       ▼
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    GPIO Data Register                       │
 * │  DR = [MW_I | MW_Q | LASER | MASTER | ...]                 │
 * └─────────────────────────────────────────────────────────────┘
 */

#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "nv_controller.h"
#include "event_table.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Sequencer Configuration
 */
#ifndef CONFIG_NV_SEQUENCER_MAX_EVENTS
#define CONFIG_NV_SEQUENCER_MAX_EVENTS 4096
#endif

#ifndef CONFIG_NV_SEQUENCER_DMA_BUFFER_SIZE
#define CONFIG_NV_SEQUENCER_DMA_BUFFER_SIZE 256
#endif

/*
 * Sequencer Operating Modes
 */
typedef enum {
    SEQ_MODE_ONESHOT   = 0,  /* Run sequence once and stop */
    SEQ_MODE_LOOP      = 1,  /* Loop sequence indefinitely */
    SEQ_MODE_N_REPEAT  = 2,  /* Repeat sequence N times */
    SEQ_MODE_GATED     = 3,  /* Run while external gate high */
} seq_mode_t;

/*
 * Sequencer Configuration Structure
 */
typedef struct {
    seq_mode_t mode;               /* Operating mode */
    uint32_t repeat_count;         /* Number of repeats (for N_REPEAT mode) */
    nv_trigger_mode_t trigger;     /* Trigger mode */
    uint32_t trigger_delay_ticks;  /* Delay after trigger before start */
    bool auto_rearm;               /* Automatically rearm after completion */
    bool sync_adc;                 /* Enable ADC_ETC synchronization */
    uint32_t adc_trigger_offset;   /* ADC trigger offset from master pulse */
} seq_config_t;

/*
 * Sequencer State (read-only from outside)
 */
typedef struct {
    nv_status_t status;
    uint32_t current_event_index;
    uint32_t events_remaining;
    uint32_t loops_completed;
    uint32_t current_timestamp;
    nv_channel_mask_t current_output;
} seq_state_t;

/*
 * DMA Buffer Entry
 *
 * Each entry contains:
 * - gpio_value: Bitmask to write to GPIO DR register
 * - compare_value: Timer compare value for next update
 */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t gpio_value;      /* GPIO output state */
    uint32_t compare_value;   /* Timer compare match value */
} seq_dma_entry_t;

/*
 * Sequencer Hardware Abstraction
 *
 * Low-level interface to i.MX RT1062 hardware:
 * - GPT: General Purpose Timer
 * - XBAR: Crossbar Switch
 * - DMA: Enhanced Direct Memory Access
 */
typedef struct {
    volatile uint32_t *gpt_base;    /* GPT register base */
    volatile uint32_t *gpio_base;   /* GPIO register base */
    volatile uint32_t *xbar_base;   /* XBAR register base */
    volatile uint32_t *dma_base;    /* DMA register base */
    uint8_t dma_channel;            /* DMA channel number */
    uint8_t gpt_irq;                /* GPT IRQ number */
} seq_hal_t;

/*
 * Sequencer API Functions
 */

/**
 * @brief Initialize the pulse sequencer hardware
 *
 * Sets up GPT timer, DMA channels, XBAR routing, and GPIO pins.
 * Must be called before any other sequencer functions.
 *
 * @return NV_OK on success, error code otherwise
 */
nv_error_t seq_init(void);

/**
 * @brief Deinitialize and release sequencer resources
 *
 * @return NV_OK on success
 */
nv_error_t seq_deinit(void);

/**
 * @brief Configure sequencer operating parameters
 *
 * @param config Pointer to configuration structure
 * @return NV_OK on success, NV_ERR_INVALID_ARG if config invalid
 */
nv_error_t seq_configure(const seq_config_t *config);

/**
 * @brief Load an event table into the sequencer
 *
 * Validates the event table and prepares DMA buffers.
 * The sequencer must be stopped before loading.
 *
 * @param table Pointer to event table
 * @return NV_OK on success, error code otherwise
 */
nv_error_t seq_load(const event_table_t *table);

/**
 * @brief Arm the sequencer for execution
 *
 * Prepares hardware and waits for trigger condition.
 *
 * @return NV_OK on success, NV_ERR_NOT_READY if no sequence loaded
 */
nv_error_t seq_arm(void);

/**
 * @brief Disarm the sequencer
 *
 * Cancels armed state without executing.
 *
 * @return NV_OK on success
 */
nv_error_t seq_disarm(void);

/**
 * @brief Issue software trigger to start sequence
 *
 * Only effective in TRIGGER_SOFTWARE mode.
 *
 * @return NV_OK on success, NV_ERR_NOT_READY if not armed
 */
nv_error_t seq_trigger(void);

/**
 * @brief Abort currently running sequence
 *
 * Immediately stops execution and sets all outputs low.
 *
 * @return NV_OK on success
 */
nv_error_t seq_abort(void);

/**
 * @brief Pause sequence execution
 *
 * Temporarily halts execution while maintaining state.
 *
 * @return NV_OK on success, NV_ERR_NOT_READY if not running
 */
nv_error_t seq_pause(void);

/**
 * @brief Resume paused sequence
 *
 * @return NV_OK on success, NV_ERR_NOT_READY if not paused
 */
nv_error_t seq_resume(void);

/**
 * @brief Get current sequencer state
 *
 * @param state Pointer to state structure to fill
 */
void seq_get_state(seq_state_t *state);

/**
 * @brief Set all output channels immediately
 *
 * Direct GPIO write bypassing sequencer (for manual control).
 *
 * @param mask Channel bitmask to set
 */
void seq_set_outputs(nv_channel_mask_t mask);

/**
 * @brief Get current output channel states
 *
 * @return Current channel bitmask
 */
nv_channel_mask_t seq_get_outputs(void);

/**
 * @brief Set single channel state immediately
 *
 * @param channel Channel to modify
 * @param state true = high, false = low
 */
void seq_set_channel(nv_channel_t channel, bool state);

/**
 * @brief Toggle single channel
 *
 * @param channel Channel to toggle
 */
void seq_toggle_channel(nv_channel_t channel);

/**
 * @brief Register callback for sequence completion
 *
 * @param callback Function to call on sequence end
 * @param user_data User context pointer
 */
void seq_set_completion_callback(nv_sequence_callback_t callback, void *user_data);

/**
 * @brief Register callback for trigger events
 *
 * @param callback Function to call on trigger
 * @param user_data User context pointer
 */
void seq_set_trigger_callback(nv_trigger_callback_t callback, void *user_data);

/**
 * @brief Get hardware timer value
 *
 * @return Current 32-bit timer count
 */
uint32_t seq_get_timer_count(void);

/**
 * @brief Wait for sequence completion (blocking)
 *
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return NV_OK if completed, NV_ERR_TIMEOUT if timeout
 */
nv_error_t seq_wait_complete(uint32_t timeout_ms);

/*
 * Low-level hardware functions (internal use)
 */
void seq_hal_init(void);
void seq_hal_start_timer(void);
void seq_hal_stop_timer(void);
void seq_hal_setup_dma(const seq_dma_entry_t *buffer, size_t count);
void seq_hal_enable_dma(void);
void seq_hal_disable_dma(void);

/*
 * ISR handler (called from IRQ context)
 */
void seq_isr_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* SEQUENCER_H */
