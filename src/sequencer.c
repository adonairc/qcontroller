/*
 * NV-Center Controller - Pulse Sequencer Implementation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-timed pulse sequencer using GPT timer and DMA.
 * Provides sub-10ns jitter pulse generation for quantum sensing.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <string.h>

#include "sequencer.h"
#include "event_table.h"
#include "xbar_routing.h"

LOG_MODULE_REGISTER(sequencer, CONFIG_LOG_DEFAULT_LEVEL);

/* i.MX RT1062 GPT1 IRQ number (from reference manual) */
#define GPT1_IRQn   100
#define GPT2_IRQn   101

/*
 * i.MX RT1062 Register Definitions
 *
 * Direct hardware access for deterministic timing.
 * These bypass Zephyr drivers for minimal latency.
 */

/* GPT Register Offsets */
#define GPT_CR      0x00    /* Control Register */
#define GPT_PR      0x04    /* Prescaler Register */
#define GPT_SR      0x08    /* Status Register */
#define GPT_IR      0x0C    /* Interrupt Register */
#define GPT_OCR1    0x10    /* Output Compare Register 1 */
#define GPT_OCR2    0x14    /* Output Compare Register 2 */
#define GPT_OCR3    0x18    /* Output Compare Register 3 */
#define GPT_ICR1    0x1C    /* Input Capture Register 1 */
#define GPT_ICR2    0x20    /* Input Capture Register 2 */
#define GPT_CNT     0x24    /* Counter Register */

/* GPT Control Register Bits */
#define GPT_CR_EN           (1 << 0)
#define GPT_CR_ENMOD        (1 << 1)
#define GPT_CR_DBGEN        (1 << 2)
#define GPT_CR_WAITEN       (1 << 3)
#define GPT_CR_DOZEEN       (1 << 4)
#define GPT_CR_STOPEN       (1 << 5)
#define GPT_CR_CLKSRC_SHIFT 6
#define GPT_CR_CLKSRC_MASK  (0x7 << GPT_CR_CLKSRC_SHIFT)
#define GPT_CR_FRR          (1 << 9)
#define GPT_CR_EN_24M       (1 << 10)
#define GPT_CR_SWR          (1 << 15)

/* GPT Status Register Bits */
#define GPT_SR_OF1          (1 << 0)
#define GPT_SR_OF2          (1 << 1)
#define GPT_SR_OF3          (1 << 2)
#define GPT_SR_IF1          (1 << 3)
#define GPT_SR_IF2          (1 << 4)
#define GPT_SR_ROV          (1 << 5)

/* GPT Interrupt Register Bits */
#define GPT_IR_OF1IE        (1 << 0)
#define GPT_IR_OF2IE        (1 << 1)
#define GPT_IR_OF3IE        (1 << 2)
#define GPT_IR_IF1IE        (1 << 3)
#define GPT_IR_IF2IE        (1 << 4)
#define GPT_IR_ROVIE        (1 << 5)

/* GPIO Register Offsets */
#define GPIO_DR             0x00    /* Data Register */
#define GPIO_GDIR           0x04    /* Direction Register */
#define GPIO_PSR            0x08    /* Pad Status Register */
#define GPIO_DR_SET         0x84    /* Data Register SET */
#define GPIO_DR_CLEAR       0x88    /* Data Register CLEAR */
#define GPIO_DR_TOGGLE      0x8C    /* Data Register TOGGLE */

/* Base Addresses (i.MX RT1062) */
#define GPT1_BASE           0x401EC000
#define GPT2_BASE           0x401F0000
#define GPIO1_BASE          0x401B8000
#define GPIO2_BASE          0x401BC000
#define GPIO3_BASE          0x401C0000
#define GPIO4_BASE          0x401C4000
#define GPIO5_BASE          0x400C0000

/* DMA (eDMA) Registers */
#define DMA_BASE            0x400E8000
#define DMA_TCD_BASE        0x400E9000
#define DMA_TCD_SIZE        0x20

/* DMAMUX */
#define DMAMUX_BASE         0x400EC000
#define DMAMUX_CHCFG(n)     (DMAMUX_BASE + (n) * 4)

/*
 * Sequencer State
 */
typedef struct {
    /* Configuration */
    seq_config_t config;

    /* State */
    volatile nv_status_t status;
    volatile uint32_t current_event;
    volatile uint32_t events_total;
    volatile uint32_t loops_completed;
    volatile uint32_t loops_remaining;

    /* Event data */
    const event_table_t *event_table;

    /* DMA ping-pong buffers */
    seq_dma_entry_t dma_buffer_a[CONFIG_NV_SEQUENCER_DMA_BUFFER_SIZE];
    seq_dma_entry_t dma_buffer_b[CONFIG_NV_SEQUENCER_DMA_BUFFER_SIZE];
    volatile uint8_t active_buffer;  /* 0 = A, 1 = B */
    volatile uint32_t buffer_events; /* Events in current buffer */

    /* Current output state */
    volatile nv_channel_mask_t current_output;

    /* Hardware pointers */
    volatile uint32_t *gpt_base;
    volatile uint32_t *gpio_base;

    /* Callbacks */
    nv_sequence_callback_t completion_callback;
    void *completion_user_data;
    nv_trigger_callback_t trigger_callback;
    void *trigger_user_data;

    /* Statistics */
    uint32_t sequences_completed;
    uint32_t events_executed;
    uint32_t overflow_count;
    uint32_t underrun_count;
    uint32_t max_latency_ticks;

    /* Semaphore for completion waiting */
    struct k_sem completion_sem;

    /* DMA channel */
    uint8_t dma_channel;

} seq_context_t;

static seq_context_t seq_ctx;

/* GPIO pin mapping for channels */
static const struct {
    uint32_t gpio_base;
    uint8_t pin;
} channel_pins[NV_CHANNEL_COUNT] = {
    [NV_CHANNEL_MW_I]     = { GPIO4_BASE, 4 },   /* Pin 2 */
    [NV_CHANNEL_MW_Q]     = { GPIO4_BASE, 5 },   /* Pin 3 */
    [NV_CHANNEL_LASER]    = { GPIO4_BASE, 6 },   /* Pin 4 */
    [NV_CHANNEL_MASTER]   = { GPIO4_BASE, 8 },   /* Pin 5 */
    [NV_CHANNEL_TRIG_OUT] = { GPIO2_BASE, 10 },  /* Pin 6 */
    [NV_CHANNEL_AUX_0]    = { GPIO2_BASE, 17 },  /* Pin 7 */
    [NV_CHANNEL_AUX_1]    = { GPIO2_BASE, 16 },  /* Pin 8 */
    [NV_CHANNEL_AUX_2]    = { GPIO2_BASE, 18 },  /* Pin 9 */
};

/*
 * Direct register access macros
 */
#define REG32(addr)         (*(volatile uint32_t *)(addr))
#define GPT_REG(offset)     REG32((uint32_t)seq_ctx.gpt_base + (offset))
#define GPIO_REG(base, off) REG32((base) + (off))

/*
 * Forward declarations
 */
static void seq_fill_dma_buffer(seq_dma_entry_t *buffer, uint32_t *start_idx, uint32_t max_entries);
static void seq_start_dma(void);
static void seq_setup_next_compare(void);

/*
 * GPT ISR - handles compare match events
 */
static void gpt1_isr(const void *arg)
{
    ARG_UNUSED(arg);

    uint32_t status = GPT_REG(GPT_SR);

    /* Clear interrupt flags */
    GPT_REG(GPT_SR) = status;

    if (status & GPT_SR_OF1) {
        /* Output compare 1 matched - update GPIO */
        if (seq_ctx.status == NV_STATUS_RUNNING) {
            if (seq_ctx.current_event < seq_ctx.events_total) {
                const pulse_event_t *event =
                    &seq_ctx.event_table->events[seq_ctx.current_event];

                /* Update GPIO directly */
                seq_ctx.current_output = event->mask;
                seq_set_outputs(event->mask);

                /* Check for ADC trigger flag */
                if (event->flags & EVENT_FLAG_ADC_TRIGGER) {
                    /* ADC trigger is handled by hardware XBAR routing */
                }

                seq_ctx.current_event++;
                seq_ctx.events_executed++;

                /* Set up next compare value */
                if (seq_ctx.current_event < seq_ctx.events_total) {
                    GPT_REG(GPT_OCR1) =
                        seq_ctx.event_table->events[seq_ctx.current_event].timestamp_ticks;
                } else {
                    /* Sequence complete or loop */
                    if (seq_ctx.config.mode == SEQ_MODE_LOOP ||
                        (seq_ctx.config.mode == SEQ_MODE_N_REPEAT &&
                         seq_ctx.loops_remaining > 0)) {

                        /* Loop back */
                        seq_ctx.current_event = seq_ctx.event_table->header.loop_start_idx;
                        seq_ctx.loops_completed++;
                        if (seq_ctx.config.mode == SEQ_MODE_N_REPEAT) {
                            seq_ctx.loops_remaining--;
                        }

                        /* Reset timer for loop */
                        GPT_REG(GPT_CR) &= ~GPT_CR_EN;
                        GPT_REG(GPT_CNT) = 0;
                        GPT_REG(GPT_OCR1) =
                            seq_ctx.event_table->events[seq_ctx.current_event].timestamp_ticks;
                        GPT_REG(GPT_CR) |= GPT_CR_EN;

                    } else {
                        /* Sequence complete */
                        seq_ctx.status = NV_STATUS_IDLE;
                        seq_ctx.sequences_completed++;

                        /* Stop timer */
                        GPT_REG(GPT_CR) &= ~GPT_CR_EN;

                        /* All outputs low */
                        seq_set_outputs(0);

                        /* Signal completion */
                        k_sem_give(&seq_ctx.completion_sem);

                        if (seq_ctx.completion_callback) {
                            seq_ctx.completion_callback(NV_STATUS_IDLE,
                                                       seq_ctx.completion_user_data);
                        }

                        /* Auto-rearm if configured */
                        if (seq_ctx.config.auto_rearm) {
                            seq_arm();
                        }
                    }
                }
            }
        }
    }

    if (status & GPT_SR_IF1) {
        /* Input capture 1 - external trigger received */
        uint32_t capture_time = GPT_REG(GPT_ICR1);

        if (seq_ctx.trigger_callback) {
            seq_ctx.trigger_callback(capture_time, seq_ctx.trigger_user_data);
        }

        if (seq_ctx.status == NV_STATUS_ARMED &&
            seq_ctx.config.trigger == NV_TRIGGER_EXTERNAL) {
            /* Start sequence on external trigger */
            seq_trigger();
        }
    }
}

/*
 * Initialize the pulse sequencer
 */
nv_error_t seq_init(void)
{
    LOG_INF("Initializing pulse sequencer");

    memset(&seq_ctx, 0, sizeof(seq_ctx));

    /* Initialize hardware pointers */
    seq_ctx.gpt_base = (volatile uint32_t *)GPT1_BASE;
    seq_ctx.gpio_base = (volatile uint32_t *)GPIO4_BASE;
    seq_ctx.dma_channel = 0;

    /* Initialize synchronization */
    k_sem_init(&seq_ctx.completion_sem, 0, 1);

    /* Reset GPT1 */
    GPT_REG(GPT_CR) = GPT_CR_SWR;
    while (GPT_REG(GPT_CR) & GPT_CR_SWR) {
        /* Wait for reset complete */
    }

    /* Configure GPT1:
     * - Clock source: ipg_clk_highfreq (150MHz)
     * - Free-run mode
     * - Enable in debug/wait/stop modes for consistent timing
     */
    GPT_REG(GPT_CR) = (1 << GPT_CR_CLKSRC_SHIFT) |  /* Peripheral clock */
                      GPT_CR_FRR |                   /* Free-run mode */
                      GPT_CR_DBGEN |                 /* Debug enable */
                      GPT_CR_WAITEN |                /* Wait enable */
                      GPT_CR_ENMOD;                  /* Reset on enable */

    /* No prescaler (divide by 1) */
    GPT_REG(GPT_PR) = 0;

    /* Configure GPIO pins as outputs */
    for (int i = 0; i < NV_CHANNEL_COUNT; i++) {
        volatile uint32_t *gpio = (volatile uint32_t *)channel_pins[i].gpio_base;
        gpio[GPIO_GDIR / 4] |= (1 << channel_pins[i].pin);
        gpio[GPIO_DR_CLEAR / 4] = (1 << channel_pins[i].pin);  /* Start low */
    }

    /* Connect GPT1 ISR */
    IRQ_CONNECT(GPT1_IRQn, 0, gpt1_isr, NULL, 0);
    irq_enable(GPT1_IRQn);

    /* Initialize XBAR routing */
    xbar_init();
    xbar_setup_sequencer_trigger();

    seq_ctx.status = NV_STATUS_IDLE;

    LOG_INF("Sequencer initialized, timer @ %u Hz", NV_TIMER_FREQ_HZ);

    return NV_OK;
}

nv_error_t seq_deinit(void)
{
    /* Disable timer */
    GPT_REG(GPT_CR) &= ~GPT_CR_EN;
    GPT_REG(GPT_IR) = 0;

    irq_disable(GPT1_IRQn);

    seq_ctx.status = NV_STATUS_IDLE;

    return NV_OK;
}

nv_error_t seq_configure(const seq_config_t *config)
{
    if (!config) {
        return NV_ERR_INVALID_ARG;
    }

    if (seq_ctx.status == NV_STATUS_RUNNING) {
        return NV_ERR_BUSY;
    }

    memcpy(&seq_ctx.config, config, sizeof(seq_config_t));

    LOG_DBG("Sequencer configured: mode=%d, trigger=%d, repeats=%u",
            config->mode, config->trigger, config->repeat_count);

    return NV_OK;
}

nv_error_t seq_load(const event_table_t *table)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    if (seq_ctx.status == NV_STATUS_RUNNING) {
        return NV_ERR_BUSY;
    }

    /* Validate table */
    nv_error_t err = event_table_validate(table);
    if (err != NV_OK) {
        LOG_ERR("Event table validation failed: %d", err);
        return err;
    }

    seq_ctx.event_table = table;
    seq_ctx.events_total = table->header.event_count;
    seq_ctx.current_event = 0;
    seq_ctx.loops_completed = 0;
    seq_ctx.loops_remaining = seq_ctx.config.repeat_count;

    LOG_INF("Loaded sequence with %u events, duration %u ticks",
            table->header.event_count, table->header.duration_ticks);

    return NV_OK;
}

nv_error_t seq_arm(void)
{
    if (!seq_ctx.event_table || seq_ctx.events_total == 0) {
        return NV_ERR_NOT_READY;
    }

    if (seq_ctx.status == NV_STATUS_RUNNING) {
        return NV_ERR_BUSY;
    }

    /* Reset state */
    seq_ctx.current_event = 0;
    seq_ctx.loops_completed = 0;
    seq_ctx.loops_remaining = seq_ctx.config.repeat_count;
    k_sem_reset(&seq_ctx.completion_sem);

    /* Set up first compare value */
    GPT_REG(GPT_OCR1) = seq_ctx.event_table->events[0].timestamp_ticks +
                        seq_ctx.config.trigger_delay_ticks;

    /* Enable compare interrupt */
    GPT_REG(GPT_IR) = GPT_IR_OF1IE;

    /* Enable input capture interrupt if external trigger */
    if (seq_ctx.config.trigger == NV_TRIGGER_EXTERNAL) {
        GPT_REG(GPT_IR) |= GPT_IR_IF1IE;
    }

    /* Clear any pending status */
    GPT_REG(GPT_SR) = 0x3F;

    seq_ctx.status = NV_STATUS_ARMED;

    LOG_DBG("Sequencer armed, waiting for trigger");

    /* Start immediately if configured */
    if (seq_ctx.config.trigger == NV_TRIGGER_IMMEDIATE) {
        return seq_trigger();
    }

    return NV_OK;
}

nv_error_t seq_disarm(void)
{
    if (seq_ctx.status == NV_STATUS_RUNNING) {
        seq_abort();
    }

    GPT_REG(GPT_IR) = 0;
    seq_ctx.status = NV_STATUS_IDLE;

    return NV_OK;
}

nv_error_t seq_trigger(void)
{
    if (seq_ctx.status != NV_STATUS_ARMED) {
        return NV_ERR_NOT_READY;
    }

    LOG_DBG("Triggering sequence");

    /* Reset and start timer */
    GPT_REG(GPT_CR) &= ~GPT_CR_EN;

    /* Reset counter */
    uint32_t cr = GPT_REG(GPT_CR);
    GPT_REG(GPT_CR) = cr | GPT_CR_ENMOD;

    seq_ctx.status = NV_STATUS_RUNNING;

    /* Enable timer - sequence starts now */
    GPT_REG(GPT_CR) |= GPT_CR_EN;

    if (seq_ctx.trigger_callback) {
        seq_ctx.trigger_callback(0, seq_ctx.trigger_user_data);
    }

    return NV_OK;
}

nv_error_t seq_abort(void)
{
    /* Stop timer immediately */
    GPT_REG(GPT_CR) &= ~GPT_CR_EN;

    /* All outputs low */
    seq_set_outputs(0);

    seq_ctx.status = NV_STATUS_IDLE;

    /* Signal any waiting threads */
    k_sem_give(&seq_ctx.completion_sem);

    if (seq_ctx.completion_callback) {
        seq_ctx.completion_callback(NV_STATUS_IDLE, seq_ctx.completion_user_data);
    }

    LOG_DBG("Sequence aborted");

    return NV_OK;
}

nv_error_t seq_pause(void)
{
    if (seq_ctx.status != NV_STATUS_RUNNING) {
        return NV_ERR_NOT_READY;
    }

    /* Disable timer without clearing */
    GPT_REG(GPT_CR) &= ~GPT_CR_EN;

    seq_ctx.status = NV_STATUS_PAUSED;

    return NV_OK;
}

nv_error_t seq_resume(void)
{
    if (seq_ctx.status != NV_STATUS_PAUSED) {
        return NV_ERR_NOT_READY;
    }

    seq_ctx.status = NV_STATUS_RUNNING;

    /* Re-enable timer */
    GPT_REG(GPT_CR) |= GPT_CR_EN;

    return NV_OK;
}

void seq_get_state(seq_state_t *state)
{
    if (!state) {
        return;
    }

    state->status = seq_ctx.status;
    state->current_event_index = seq_ctx.current_event;
    state->events_remaining = seq_ctx.events_total - seq_ctx.current_event;
    state->loops_completed = seq_ctx.loops_completed;
    state->current_timestamp = GPT_REG(GPT_CNT);
    state->current_output = seq_ctx.current_output;
}

void seq_set_outputs(nv_channel_mask_t mask)
{
    seq_ctx.current_output = mask;

    /* Update all channel GPIO pins based on mask */
    for (int i = 0; i < NV_CHANNEL_COUNT; i++) {
        volatile uint32_t *gpio = (volatile uint32_t *)channel_pins[i].gpio_base;
        uint32_t pin_mask = (1 << channel_pins[i].pin);

        if (mask & (1 << i)) {
            gpio[GPIO_DR_SET / 4] = pin_mask;
        } else {
            gpio[GPIO_DR_CLEAR / 4] = pin_mask;
        }
    }
}

nv_channel_mask_t seq_get_outputs(void)
{
    return seq_ctx.current_output;
}

void seq_set_channel(nv_channel_t channel, bool state)
{
    if (channel >= NV_CHANNEL_COUNT) {
        return;
    }

    volatile uint32_t *gpio = (volatile uint32_t *)channel_pins[channel].gpio_base;
    uint32_t pin_mask = (1 << channel_pins[channel].pin);

    if (state) {
        gpio[GPIO_DR_SET / 4] = pin_mask;
        seq_ctx.current_output |= (1 << channel);
    } else {
        gpio[GPIO_DR_CLEAR / 4] = pin_mask;
        seq_ctx.current_output &= ~(1 << channel);
    }
}

void seq_toggle_channel(nv_channel_t channel)
{
    if (channel >= NV_CHANNEL_COUNT) {
        return;
    }

    volatile uint32_t *gpio = (volatile uint32_t *)channel_pins[channel].gpio_base;
    uint32_t pin_mask = (1 << channel_pins[channel].pin);

    gpio[GPIO_DR_TOGGLE / 4] = pin_mask;
    seq_ctx.current_output ^= (1 << channel);
}

void seq_set_completion_callback(nv_sequence_callback_t callback, void *user_data)
{
    seq_ctx.completion_callback = callback;
    seq_ctx.completion_user_data = user_data;
}

void seq_set_trigger_callback(nv_trigger_callback_t callback, void *user_data)
{
    seq_ctx.trigger_callback = callback;
    seq_ctx.trigger_user_data = user_data;
}

uint32_t seq_get_timer_count(void)
{
    return GPT_REG(GPT_CNT);
}

nv_error_t seq_wait_complete(uint32_t timeout_ms)
{
    if (seq_ctx.status != NV_STATUS_RUNNING &&
        seq_ctx.status != NV_STATUS_ARMED) {
        return NV_OK;
    }

    int ret = k_sem_take(&seq_ctx.completion_sem, K_MSEC(timeout_ms));
    if (ret == -EAGAIN) {
        return NV_ERR_TIMEOUT;
    }

    return NV_OK;
}

/*
 * Low-level hardware initialization
 */
void seq_hal_init(void)
{
    /* Already done in seq_init() */
}

void seq_hal_start_timer(void)
{
    GPT_REG(GPT_CR) |= GPT_CR_EN;
}

void seq_hal_stop_timer(void)
{
    GPT_REG(GPT_CR) &= ~GPT_CR_EN;
}

/*
 * DMA buffer fill - prepares GPIO updates for DMA transfer
 */
static void seq_fill_dma_buffer(seq_dma_entry_t *buffer, uint32_t *start_idx, uint32_t max_entries)
{
    uint32_t idx = *start_idx;
    uint32_t count = 0;

    while (count < max_entries && idx < seq_ctx.events_total) {
        const pulse_event_t *event = &seq_ctx.event_table->events[idx];

        /* Convert channel mask to GPIO port value */
        uint32_t gpio_value = 0;
        for (int ch = 0; ch < NV_CHANNEL_COUNT; ch++) {
            if (event->mask & (1 << ch)) {
                gpio_value |= (1 << channel_pins[ch].pin);
            }
        }

        buffer[count].gpio_value = gpio_value;
        buffer[count].compare_value = event->timestamp_ticks;

        count++;
        idx++;
    }

    *start_idx = idx;
    seq_ctx.buffer_events = count;
}

void seq_hal_setup_dma(const seq_dma_entry_t *buffer, size_t count)
{
    /* Configure DMA channel for GPIO updates */
    volatile uint32_t *tcd = (volatile uint32_t *)(DMA_TCD_BASE + seq_ctx.dma_channel * DMA_TCD_SIZE);

    /* Source address - DMA buffer */
    tcd[0] = (uint32_t)buffer;  /* SADDR */

    /* Source offset - advance by entry size */
    tcd[1] = sizeof(seq_dma_entry_t) | (0 << 16);  /* SOFF | ATTR */

    /* Number of bytes per minor loop */
    tcd[2] = 4;  /* NBYTES - transfer GPIO value (4 bytes) */

    /* Source last address adjustment */
    tcd[3] = -(int32_t)(count * sizeof(seq_dma_entry_t));  /* SLAST */

    /* Destination address - GPIO DR register */
    tcd[4] = (uint32_t)seq_ctx.gpio_base + GPIO_DR;  /* DADDR */

    /* Destination offset */
    tcd[5] = 0;  /* DOFF - no increment */

    /* Current major loop count */
    tcd[6] = count | (count << 16);  /* CITER | BITER */

    /* Destination last address adjustment */
    tcd[7] = 0;  /* DLAST_SGA */
}

void seq_hal_enable_dma(void)
{
    volatile uint32_t *dma = (volatile uint32_t *)DMA_BASE;
    dma[1] = (1 << seq_ctx.dma_channel);  /* SERQ - enable request */
}

void seq_hal_disable_dma(void)
{
    volatile uint32_t *dma = (volatile uint32_t *)DMA_BASE;
    dma[2] = (1 << seq_ctx.dma_channel);  /* CERQ - clear request */
}

/*
 * ISR Handler (alternate entry point)
 */
void seq_isr_handler(void)
{
    gpt1_isr(NULL);
}
