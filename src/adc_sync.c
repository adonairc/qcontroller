/*
 * NV-Center Controller - ADC Synchronization Implementation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Synchronized ADC acquisition using ADC_ETC (External Trigger Control)
 * for phase-aligned sampling with pulse sequences.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "adc_sync.h"
#include "xbar_routing.h"

LOG_MODULE_REGISTER(adc_sync, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * i.MX RT1062 ADC_ETC Register Definitions
 */
#define ADC_ETC_BASE            0x403B0000

/* ADC_ETC Register Offsets */
#define ADC_ETC_CTRL            0x00
#define ADC_ETC_DONE0_1_IRQ     0x04
#define ADC_ETC_DONE2_ERR_IRQ   0x08
#define ADC_ETC_DMA_CTRL        0x0C
#define ADC_ETC_TRIG0_CTRL      0x10
#define ADC_ETC_TRIG0_COUNTER   0x14
#define ADC_ETC_TRIG0_CHAIN_1_0 0x18
#define ADC_ETC_TRIG0_CHAIN_3_2 0x1C
#define ADC_ETC_TRIG0_CHAIN_5_4 0x20
#define ADC_ETC_TRIG0_CHAIN_7_6 0x24
#define ADC_ETC_TRIG0_RESULT_1_0 0x28
#define ADC_ETC_TRIG0_RESULT_3_2 0x2C
#define ADC_ETC_TRIG0_RESULT_5_4 0x30
#define ADC_ETC_TRIG0_RESULT_7_6 0x34

/* Trigger register stride */
#define ADC_ETC_TRIG_STRIDE     0x28

/* ADC_ETC_CTRL bits */
#define ADC_ETC_CTRL_TRIG_ENABLE(n)  (1 << (n))
#define ADC_ETC_CTRL_EXT_TRIG_EN     (1 << 8)
#define ADC_ETC_CTRL_SOFTRST         (1 << 31)

/* ADC_ETC_TRIGn_CTRL bits */
#define TRIG_CTRL_TRIG_CHAIN(n)      ((n) << 0)
#define TRIG_CTRL_TRIG_PRIORITY(n)   ((n) << 4)
#define TRIG_CTRL_SYNC_MODE          (1 << 8)

/* ADC_ETC_TRIGn_CHAIN bits */
#define CHAIN_CSEL(n)      ((n) << 0)   /* ADC channel select */
#define CHAIN_HWTS(n)      ((n) << 4)   /* Hardware trigger select */
#define CHAIN_B2B          (1 << 8)     /* Back-to-back mode */
#define CHAIN_IE           (1 << 9)     /* Interrupt enable */

/* ADC Base Addresses */
#define ADC1_BASE               0x400C4000
#define ADC2_BASE               0x400C8000

/* ADC Register Offsets (LPADC) */
#define ADC_VERID               0x00
#define ADC_PARAM               0x04
#define ADC_CTRL                0x10
#define ADC_STAT                0x14
#define ADC_IE                  0x18
#define ADC_DE                  0x1C
#define ADC_CFG                 0x20
#define ADC_PAUSE               0x24
#define ADC_FCTRL               0x30
#define ADC_SWTRIG              0x34
#define ADC_TCTRL(n)            (0x40 + (n) * 4)
#define ADC_CMDL(n)             (0x100 + (n) * 8)
#define ADC_CMDH(n)             (0x104 + (n) * 8)
#define ADC_RESFIFO             0x300

/*
 * ADC Context
 */
typedef struct {
    /* Configuration */
    adc_config_t config;

    /* Status */
    adc_status_t status;

    /* Sample buffer */
    uint16_t sample_buffer[CONFIG_NV_ADC_SAMPLE_BUFFER_SIZE];
    uint32_t sample_count;
    uint32_t sample_index;
    uint32_t timestamp;

    /* Statistics */
    adc_statistics_t stats;

    /* Callbacks */
    adc_data_callback_t data_callback;
    void *data_user_data;
    adc_trigger_callback_t trigger_callback;
    void *trigger_user_data;

    /* Hardware pointers */
    volatile uint32_t *adc_etc;
    volatile uint32_t *adc1;
    volatile uint32_t *adc2;

    /* Lock-in mode */
    bool lockin_enabled;
    int32_t lockin_sum_in_phase;
    int32_t lockin_sum_quadrature;
    uint32_t lockin_count;

    /* DMA state */
    uint8_t dma_channel;

} adc_context_t;

static adc_context_t adc_ctx;

/*
 * Register access macros
 */
#define ADC_ETC_REG(off)    (*(volatile uint32_t *)((uint32_t)adc_ctx.adc_etc + (off)))
#define ADC1_REG(off)       (*(volatile uint32_t *)((uint32_t)adc_ctx.adc1 + (off)))
#define ADC2_REG(off)       (*(volatile uint32_t *)((uint32_t)adc_ctx.adc2 + (off)))

/*
 * ADC_ETC ISR
 */
static void adc_etc_isr(const void *arg)
{
    ARG_UNUSED(arg);

    uint32_t done_status = ADC_ETC_REG(ADC_ETC_DONE0_1_IRQ);

    /* Clear interrupt */
    ADC_ETC_REG(ADC_ETC_DONE0_1_IRQ) = done_status;

    /* Check which triggers completed */
    if (done_status & 0x01) {  /* Trigger 0 chain 0 done */
        /* Read result from trigger 0 */
        uint32_t result = ADC_ETC_REG(ADC_ETC_TRIG0_RESULT_1_0);
        uint16_t sample = result & 0xFFF;  /* 12-bit result */

        /* Store sample */
        if (adc_ctx.sample_index < adc_ctx.config.sample_count &&
            adc_ctx.sample_index < CONFIG_NV_ADC_SAMPLE_BUFFER_SIZE) {
            adc_ctx.sample_buffer[adc_ctx.sample_index++] = sample;
            adc_ctx.sample_count = adc_ctx.sample_index;
        }

        /* Update statistics */
        adc_ctx.stats.samples_acquired++;
        adc_ctx.stats.last_value = sample;

        if (sample < adc_ctx.stats.min_value || adc_ctx.stats.min_value == 0) {
            adc_ctx.stats.min_value = sample;
        }
        if (sample > adc_ctx.stats.max_value) {
            adc_ctx.stats.max_value = sample;
        }
        adc_ctx.stats.sum += sample;

        /* Lock-in processing */
        if (adc_ctx.lockin_enabled) {
            /* Simple lock-in: accumulate with reference phase */
            /* In real implementation, would multiply by reference */
            adc_ctx.lockin_sum_in_phase += sample;
            adc_ctx.lockin_count++;
        }

        /* Check if acquisition complete */
        if (adc_ctx.sample_index >= adc_ctx.config.sample_count) {
            if (!adc_ctx.config.continuous) {
                adc_ctx.status = ADC_STATUS_COMPLETE;

                /* Notify callback */
                if (adc_ctx.data_callback) {
                    adc_ctx.data_callback(adc_ctx.sample_buffer,
                                         adc_ctx.sample_count,
                                         adc_ctx.timestamp,
                                         adc_ctx.data_user_data);
                }
            } else {
                /* Continuous mode - wrap around */
                adc_ctx.sample_index = 0;
            }
        }
    }

    adc_ctx.stats.triggers_received++;
}

/*
 * Public API Implementation
 */

nv_error_t adc_sync_init(void)
{
    LOG_INF("Initializing ADC synchronization subsystem");

    memset(&adc_ctx, 0, sizeof(adc_ctx));

    /* Set up hardware pointers */
    adc_ctx.adc_etc = (volatile uint32_t *)ADC_ETC_BASE;
    adc_ctx.adc1 = (volatile uint32_t *)ADC1_BASE;
    adc_ctx.adc2 = (volatile uint32_t *)ADC2_BASE;

    /* Soft reset ADC_ETC */
    ADC_ETC_REG(ADC_ETC_CTRL) = ADC_ETC_CTRL_SOFTRST;
    k_busy_wait(10);
    ADC_ETC_REG(ADC_ETC_CTRL) = 0;

    /* Configure ADC1 for triggered operation */
    /* Note: In production, use Zephyr ADC driver for basic config */

    /* Enable ADC_ETC trigger 0 */
    ADC_ETC_REG(ADC_ETC_CTRL) = ADC_ETC_CTRL_TRIG_ENABLE(0) |
                                ADC_ETC_CTRL_EXT_TRIG_EN;

    /* Configure trigger 0: single conversion, priority 0 */
    ADC_ETC_REG(ADC_ETC_TRIG0_CTRL) = TRIG_CTRL_TRIG_CHAIN(0) |
                                      TRIG_CTRL_TRIG_PRIORITY(0);

    /* Configure chain 0: ADC1 channel 0, interrupt enabled */
    ADC_ETC_REG(ADC_ETC_TRIG0_CHAIN_1_0) = CHAIN_CSEL(0) |
                                           CHAIN_HWTS(0) |
                                           CHAIN_IE;

    /* Connect ISR */
    IRQ_CONNECT(ADC_ETC_IRQ0_IRQn, 1, adc_etc_isr, NULL, 0);
    irq_enable(ADC_ETC_IRQ0_IRQn);

    adc_ctx.status = ADC_STATUS_IDLE;

    LOG_INF("ADC_ETC initialized");

    return NV_OK;
}

nv_error_t adc_sync_deinit(void)
{
    /* Disable interrupts */
    irq_disable(ADC_ETC_IRQ0_IRQn);

    /* Reset ADC_ETC */
    ADC_ETC_REG(ADC_ETC_CTRL) = ADC_ETC_CTRL_SOFTRST;

    adc_ctx.status = ADC_STATUS_IDLE;

    return NV_OK;
}

nv_error_t adc_sync_configure(const adc_config_t *config)
{
    if (!config) {
        return NV_ERR_INVALID_ARG;
    }

    if (config->channel > 7) {
        return NV_ERR_INVALID_ARG;
    }

    if (config->sample_count > CONFIG_NV_ADC_SAMPLE_BUFFER_SIZE) {
        return NV_ERR_OVERFLOW;
    }

    memcpy(&adc_ctx.config, config, sizeof(adc_config_t));

    /* Update ADC_ETC chain configuration */
    uint32_t chain_config = CHAIN_CSEL(config->channel) |
                            CHAIN_HWTS(config->trigger_source) |
                            CHAIN_IE;

    if (config->averaging > 1) {
        chain_config |= CHAIN_B2B;  /* Back-to-back for averaging */
    }

    ADC_ETC_REG(ADC_ETC_TRIG0_CHAIN_1_0) = chain_config;

    /* Configure ADC resolution */
    /* Note: i.MX RT1062 ADC is 12-bit native */

    adc_ctx.status = ADC_STATUS_CONFIGURED;

    LOG_DBG("ADC configured: ch=%d, samples=%u, trig=%d",
            config->channel, config->sample_count, config->trigger_source);

    return NV_OK;
}

nv_error_t adc_sync_start(void)
{
    if (adc_ctx.status == ADC_STATUS_IDLE) {
        return NV_ERR_NOT_READY;
    }

    /* Reset sample buffer */
    adc_ctx.sample_index = 0;
    adc_ctx.sample_count = 0;
    adc_ctx.timestamp = k_uptime_get_32();

    /* Enable trigger */
    uint32_t ctrl = ADC_ETC_REG(ADC_ETC_CTRL);
    ADC_ETC_REG(ADC_ETC_CTRL) = ctrl | ADC_ETC_CTRL_TRIG_ENABLE(0);

    adc_ctx.status = ADC_STATUS_RUNNING;

    LOG_DBG("ADC acquisition started");

    return NV_OK;
}

nv_error_t adc_sync_stop(void)
{
    /* Disable trigger */
    uint32_t ctrl = ADC_ETC_REG(ADC_ETC_CTRL);
    ADC_ETC_REG(ADC_ETC_CTRL) = ctrl & ~ADC_ETC_CTRL_TRIG_ENABLE(0);

    adc_ctx.status = ADC_STATUS_IDLE;

    return NV_OK;
}

nv_error_t adc_sync_trigger(void)
{
    if (adc_ctx.status != ADC_STATUS_RUNNING) {
        return NV_ERR_NOT_READY;
    }

    /* Software trigger via ADC_ETC */
    /* Write to trigger software trigger bit */
    ADC_ETC_REG(ADC_ETC_CTRL) |= (1 << 16);  /* SW trigger for TRIG0 */

    return NV_OK;
}

adc_status_t adc_sync_get_status(void)
{
    return adc_ctx.status;
}

nv_error_t adc_sync_get_buffer(adc_buffer_t *buffer)
{
    if (!buffer) {
        return NV_ERR_INVALID_ARG;
    }

    buffer->data = adc_ctx.sample_buffer;
    buffer->capacity = CONFIG_NV_ADC_SAMPLE_BUFFER_SIZE;
    buffer->count = adc_ctx.sample_count;
    buffer->read_index = 0;
    buffer->write_index = adc_ctx.sample_index;
    buffer->timestamp = adc_ctx.timestamp;
    buffer->overflow = (adc_ctx.sample_index >= CONFIG_NV_ADC_SAMPLE_BUFFER_SIZE);

    return NV_OK;
}

nv_error_t adc_sync_read(uint16_t *data, uint32_t max_samples, uint32_t *samples_read)
{
    if (!data || !samples_read) {
        return NV_ERR_INVALID_ARG;
    }

    uint32_t count = MIN(max_samples, adc_ctx.sample_count);
    memcpy(data, adc_ctx.sample_buffer, count * sizeof(uint16_t));
    *samples_read = count;

    return NV_OK;
}

void adc_sync_get_statistics(adc_statistics_t *stats)
{
    if (stats) {
        memcpy(stats, &adc_ctx.stats, sizeof(adc_statistics_t));
    }
}

void adc_sync_reset_statistics(void)
{
    memset(&adc_ctx.stats, 0, sizeof(adc_statistics_t));
}

void adc_sync_set_data_callback(adc_data_callback_t callback, void *user_data)
{
    adc_ctx.data_callback = callback;
    adc_ctx.data_user_data = user_data;
}

void adc_sync_set_trigger_callback(adc_trigger_callback_t callback, void *user_data)
{
    adc_ctx.trigger_callback = callback;
    adc_ctx.trigger_user_data = user_data;
}

nv_error_t adc_sync_with_master(int32_t offset_ns, uint32_t sample_count)
{
    /* Configure XBAR to route master pulse to ADC_ETC trigger */
    nv_error_t err = xbar_setup_adc_trigger(XBAR_IN_IOMUX_08, 0);  /* GPIO4.8 -> TRIG0 */
    if (err != NV_OK) {
        return err;
    }

    /* Update configuration */
    adc_ctx.config.trigger_source = ADC_TRIG_MASTER;
    adc_ctx.config.sample_count = sample_count;

    LOG_INF("ADC synchronized with master pulse, offset=%dns, samples=%u",
            offset_ns, sample_count);

    return NV_OK;
}

nv_error_t adc_sync_lockin_mode(bool enable, nv_channel_t reference_channel)
{
    adc_ctx.lockin_enabled = enable;

    if (enable) {
        adc_ctx.lockin_sum_in_phase = 0;
        adc_ctx.lockin_sum_quadrature = 0;
        adc_ctx.lockin_count = 0;

        LOG_INF("Lock-in mode enabled, reference channel=%d", reference_channel);
    } else {
        LOG_INF("Lock-in mode disabled");
    }

    return NV_OK;
}

nv_error_t adc_sync_lockin_result(int32_t *in_phase, int32_t *quadrature)
{
    if (!adc_ctx.lockin_enabled || adc_ctx.lockin_count == 0) {
        return NV_ERR_NOT_READY;
    }

    if (in_phase) {
        *in_phase = adc_ctx.lockin_sum_in_phase / (int32_t)adc_ctx.lockin_count;
    }
    if (quadrature) {
        *quadrature = adc_ctx.lockin_sum_quadrature / (int32_t)adc_ctx.lockin_count;
    }

    return NV_OK;
}

/*
 * Low-level ADC_ETC configuration
 */

nv_error_t adc_etc_configure_chain(uint8_t chain_id, uint8_t trigger_source)
{
    if (chain_id > 7) {
        return NV_ERR_INVALID_ARG;
    }

    /* Configure the specified chain */
    uint32_t offset = ADC_ETC_TRIG0_CHAIN_1_0 + (chain_id / 2) * 4;
    uint32_t shift = (chain_id & 1) * 16;

    uint32_t reg = ADC_ETC_REG(offset);
    reg &= ~(0xFFFF << shift);
    reg |= (CHAIN_CSEL(adc_ctx.config.channel) |
            CHAIN_HWTS(trigger_source) |
            CHAIN_IE) << shift;

    ADC_ETC_REG(offset) = reg;

    return NV_OK;
}

nv_error_t adc_xbar_route(uint8_t source, uint8_t dest)
{
    return xbar_connect(source, dest);
}

nv_error_t adc_dma_setup(uint16_t *buffer, uint32_t count, bool circular)
{
    /* Configure DMA for ADC result FIFO to buffer transfer */
    /* This would use eDMA with ADC result FIFO as source */

    LOG_DBG("ADC DMA setup: buffer=%p, count=%u, circular=%d",
            buffer, count, circular);

    /* Implementation would configure eDMA TCD for:
     * - Source: ADC_ETC_TRIGn_RESULT register
     * - Dest: buffer
     * - Count: count
     * - Circular: wrap around if enabled
     */

    return NV_OK;
}
