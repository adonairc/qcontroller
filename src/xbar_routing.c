/*
 * NV-Center Controller - XBAR Signal Routing Implementation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Crossbar switch configuration for hardware signal interconnection.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "xbar_routing.h"

LOG_MODULE_REGISTER(xbar, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * i.MX RT1062 XBAR Register Definitions
 */
#define XBARA1_BASE     0x403BC000
#define XBARB2_BASE     0x403C0000
#define XBARB3_BASE     0x403C4000

/* XBARA1 has 132 inputs and 132 outputs */
/* XBARB2/B3 are smaller crossbar switches */

/* XBARA SELx registers - each controls 2 output connections */
#define XBARA_SEL(n)    (XBARA1_BASE + ((n) * 2))

/* XBARA CTRL registers - edge detection and interrupt control */
#define XBARA_CTRL0     (XBARA1_BASE + 0x100)
#define XBARA_CTRL1     (XBARA1_BASE + 0x102)

/*
 * XBAR Context
 */
typedef struct {
    volatile uint16_t *xbara1;
    bool initialized;
} xbar_context_t;

static xbar_context_t xbar_ctx;

/*
 * Register access
 */
#define XBAR_SEL_REG(n)  (*(volatile uint16_t *)(XBARA1_BASE + ((n) * 2)))

/*
 * Public API Implementation
 */

nv_error_t xbar_init(void)
{
    LOG_INF("Initializing XBAR crossbar switch");

    xbar_ctx.xbara1 = (volatile uint16_t *)XBARA1_BASE;
    xbar_ctx.initialized = true;

    /* Clear all connections initially */
    for (int i = 0; i < 66; i++) {  /* 132 outputs / 2 per register */
        XBAR_SEL_REG(i) = 0;
    }

    LOG_INF("XBAR initialized");

    return NV_OK;
}

nv_error_t xbar_connect(xbar_input_t input, xbar_output_t output)
{
    if (!xbar_ctx.initialized) {
        return NV_ERR_NOT_READY;
    }

    if (output > 131) {
        return NV_ERR_INVALID_ARG;
    }

    /* Each SELx register controls 2 outputs:
     * - Even outputs use bits [6:0]
     * - Odd outputs use bits [14:8]
     */
    uint32_t reg_idx = output / 2;
    uint16_t reg_val = XBAR_SEL_REG(reg_idx);

    if (output & 1) {
        /* Odd output - upper byte */
        reg_val = (reg_val & 0x00FF) | ((uint16_t)input << 8);
    } else {
        /* Even output - lower byte */
        reg_val = (reg_val & 0xFF00) | (input & 0x7F);
    }

    XBAR_SEL_REG(reg_idx) = reg_val;

    LOG_DBG("XBAR: connected input %d to output %d (SEL%u = 0x%04X)",
            input, output, reg_idx, reg_val);

    return NV_OK;
}

nv_error_t xbar_disconnect(xbar_output_t output)
{
    /* Connect to logic low (input 0) effectively disconnects */
    return xbar_connect(XBAR_IN_LOGIC_LOW, output);
}

nv_error_t xbar_setup_sequencer_trigger(void)
{
    /*
     * Set up GPT1 compare match to trigger DMA for GPIO updates.
     *
     * Route: GPT1_COMPARE1 -> DMA_CH_MUX_REQ30
     *
     * This allows the GPT compare match to directly trigger DMA
     * without CPU intervention, providing deterministic timing.
     */

    nv_error_t err;

    /* GPT1 Compare 1 -> DMA request 30 */
    err = xbar_connect(XBAR_IN_GPT1_COMPARE1, XBAR_OUT_DMA_CH_MUX_REQ30);
    if (err != NV_OK) {
        return err;
    }

    LOG_INF("Sequencer trigger routing configured: GPT1_CMP1 -> DMA_REQ30");

    return NV_OK;
}

nv_error_t xbar_setup_adc_trigger(xbar_input_t source, uint8_t adc_trigger)
{
    /*
     * Route a signal source to ADC_ETC trigger input.
     *
     * Available ADC_ETC triggers: TRIG0-TRIG3 (4 triggers per ADC_ETC)
     * Each trigger can initiate a chain of ADC conversions.
     */

    xbar_output_t dest;

    switch (adc_trigger) {
    case 0:
        dest = XBAR_OUT_ADC_ETC_TRIG00;
        break;
    case 1:
        dest = XBAR_OUT_ADC_ETC_TRIG01;
        break;
    case 2:
        dest = XBAR_OUT_ADC_ETC_TRIG02;
        break;
    case 3:
        dest = XBAR_OUT_ADC_ETC_TRIG03;
        break;
    default:
        return NV_ERR_INVALID_ARG;
    }

    nv_error_t err = xbar_connect(source, dest);
    if (err != NV_OK) {
        return err;
    }

    LOG_INF("ADC trigger routing configured: input %d -> ADC_ETC_TRIG%d",
            source, adc_trigger);

    return NV_OK;
}

nv_error_t xbar_setup_external_trigger(xbar_input_t gpio_xbar_input)
{
    /*
     * Route external trigger GPIO to GPT1 capture input.
     *
     * This allows external trigger signals to:
     * 1. Capture timestamp in GPT1
     * 2. Optionally start the sequencer
     */

    /* Route to GPT1 capture (via QTIMER input as intermediate) */
    nv_error_t err = xbar_connect(gpio_xbar_input, XBAR_OUT_QTIMER1_TMR0_INPUT);
    if (err != NV_OK) {
        return err;
    }

    LOG_INF("External trigger routing configured: GPIO -> GPT1_CAPTURE");

    return NV_OK;
}

nv_error_t xbar_setup_trigger_output(xbar_input_t source, xbar_output_t gpio_xbar_output)
{
    /*
     * Route internal trigger signal to external GPIO output.
     *
     * This provides a physical output signal that can be used
     * to synchronize external equipment.
     */

    nv_error_t err = xbar_connect(source, gpio_xbar_output);
    if (err != NV_OK) {
        return err;
    }

    LOG_INF("Trigger output routing configured: %d -> GPIO", source);

    return NV_OK;
}

void xbar_dump_config(void)
{
    LOG_INF("XBAR Configuration Dump:");

    for (int i = 0; i < 66; i++) {
        uint16_t reg = XBAR_SEL_REG(i);
        if (reg != 0) {
            uint8_t even_input = reg & 0x7F;
            uint8_t odd_input = (reg >> 8) & 0x7F;

            if (even_input != 0) {
                LOG_INF("  Output %d <- Input %d", i * 2, even_input);
            }
            if (odd_input != 0) {
                LOG_INF("  Output %d <- Input %d", i * 2 + 1, odd_input);
            }
        }
    }
}

nv_error_t xbar_apply_nv_config(void)
{
    /*
     * Apply standard NV-Center Controller XBAR configuration:
     *
     * 1. GPT1 Compare 1 -> DMA trigger (for GPIO updates)
     * 2. Master pulse GPIO (GPIO4.8) -> ADC_ETC trigger 0
     * 3. External trigger input (GPIO2.17) -> GPT1 capture
     * 4. GPT1 Compare 2 -> Trigger output GPIO (GPIO2.10)
     */

    nv_error_t err;

    LOG_INF("Applying NV-Center XBAR configuration");

    /* 1. Sequencer DMA trigger */
    err = xbar_setup_sequencer_trigger();
    if (err != NV_OK) {
        LOG_ERR("Failed to set up sequencer trigger: %d", err);
        return err;
    }

    /* 2. Master pulse -> ADC trigger
     * GPIO4.8 (Master pulse) is connected to IOMUX_XBAR_IO08
     */
    err = xbar_setup_adc_trigger(XBAR_IN_IOMUX_08, 0);
    if (err != NV_OK) {
        LOG_ERR("Failed to set up ADC trigger: %d", err);
        return err;
    }

    /* 3. External trigger input
     * GPIO2.17 (Trigger in) connects via IOMUX_XBAR
     */
    err = xbar_setup_external_trigger(XBAR_IN_IOMUX_07);
    if (err != NV_OK) {
        LOG_ERR("Failed to set up external trigger: %d", err);
        return err;
    }

    /* 4. Trigger output
     * Route GPT1 Compare 2 to GPIO2.10 (Trigger out)
     */
    err = xbar_setup_trigger_output(XBAR_IN_GPT1_COMPARE2, XBAR_OUT_IOMUX_10);
    if (err != NV_OK) {
        LOG_ERR("Failed to set up trigger output: %d", err);
        return err;
    }

    LOG_INF("NV-Center XBAR configuration applied successfully");

    return NV_OK;
}
