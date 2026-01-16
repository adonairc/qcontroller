/*
 * NV-Center Controller - XBAR Signal Routing
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Crossbar switch configuration for hardware signal routing.
 * The i.MX RT1062 XBAR allows interconnection of peripheral signals
 * without CPU intervention.
 *
 * Signal Flow for NV-Center Controller:
 *
 * ┌─────────┐    XBAR1    ┌──────────┐    XBAR2    ┌─────────┐
 * │  GPT1   │────────────▶│          │────────────▶│ ADC_ETC │
 * │ Compare │   Out[n]    │          │   Out[m]    │ Trigger │
 * └─────────┘             │          │             └─────────┘
 *                         │          │
 * ┌─────────┐             │   XBAR   │             ┌─────────┐
 * │  GPIO   │────────────▶│   MUX    │────────────▶│  PWM    │
 * │ Input   │             │          │             │ Trigger │
 * └─────────┘             │          │             └─────────┘
 *                         │          │
 * ┌─────────┐             │          │             ┌─────────┐
 * │ FlexPWM │────────────▶│          │────────────▶│   DMA   │
 * │  Force  │             │          │             │ Trigger │
 * └─────────┘             └──────────┘             └─────────┘
 */

#ifndef XBAR_ROUTING_H
#define XBAR_ROUTING_H

#include "nv_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * XBAR1 Input Signals (i.MX RT1062)
 * Reference: i.MX RT1060 Reference Manual, Chapter 3
 */
typedef enum {
    /* Logic signals */
    XBAR_IN_LOGIC_LOW       = 0,
    XBAR_IN_LOGIC_HIGH      = 1,

    /* IOMUX signals */
    XBAR_IN_IOMUX_00        = 2,
    XBAR_IN_IOMUX_01        = 3,
    XBAR_IN_IOMUX_02        = 4,
    XBAR_IN_IOMUX_03        = 5,
    XBAR_IN_IOMUX_04        = 6,
    XBAR_IN_IOMUX_05        = 7,
    XBAR_IN_IOMUX_06        = 8,
    XBAR_IN_IOMUX_07        = 9,
    XBAR_IN_IOMUX_08        = 10,
    XBAR_IN_IOMUX_09        = 11,
    XBAR_IN_IOMUX_10        = 12,
    XBAR_IN_IOMUX_11        = 13,
    XBAR_IN_IOMUX_12        = 14,
    XBAR_IN_IOMUX_13        = 15,
    XBAR_IN_IOMUX_14        = 16,
    XBAR_IN_IOMUX_15        = 17,
    XBAR_IN_IOMUX_16        = 18,
    XBAR_IN_IOMUX_17        = 19,
    XBAR_IN_IOMUX_18        = 20,
    XBAR_IN_IOMUX_19        = 21,

    /* ACMP outputs */
    XBAR_IN_ACMP1_OUT       = 22,
    XBAR_IN_ACMP2_OUT       = 23,
    XBAR_IN_ACMP3_OUT       = 24,
    XBAR_IN_ACMP4_OUT       = 25,

    /* QTIMER outputs */
    XBAR_IN_QTIMER3_TMR0    = 26,
    XBAR_IN_QTIMER3_TMR1    = 27,
    XBAR_IN_QTIMER3_TMR2    = 28,
    XBAR_IN_QTIMER3_TMR3    = 29,
    XBAR_IN_QTIMER4_TMR0    = 30,
    XBAR_IN_QTIMER4_TMR1    = 31,
    XBAR_IN_QTIMER4_TMR2    = 32,
    XBAR_IN_QTIMER4_TMR3    = 33,

    /* FlexPWM outputs */
    XBAR_IN_FLEXPWM1_PWM1_OUT_TRIG0 = 34,
    XBAR_IN_FLEXPWM1_PWM1_OUT_TRIG1 = 35,
    XBAR_IN_FLEXPWM1_PWM2_OUT_TRIG0 = 36,
    XBAR_IN_FLEXPWM1_PWM2_OUT_TRIG1 = 37,
    XBAR_IN_FLEXPWM1_PWM3_OUT_TRIG0 = 38,
    XBAR_IN_FLEXPWM1_PWM3_OUT_TRIG1 = 39,
    XBAR_IN_FLEXPWM1_PWM4_OUT_TRIG0 = 40,
    XBAR_IN_FLEXPWM1_PWM4_OUT_TRIG1 = 41,

    /* PIT triggers */
    XBAR_IN_PIT_TRIGGER0    = 56,
    XBAR_IN_PIT_TRIGGER1    = 57,
    XBAR_IN_PIT_TRIGGER2    = 58,
    XBAR_IN_PIT_TRIGGER3    = 59,

    /* ADC_ETC outputs */
    XBAR_IN_ADC_ETC_TRIG0_COCO0 = 60,
    XBAR_IN_ADC_ETC_TRIG0_COCO1 = 61,
    XBAR_IN_ADC_ETC_TRIG0_COCO2 = 62,
    XBAR_IN_ADC_ETC_TRIG0_COCO3 = 63,

    /* ENC outputs */
    XBAR_IN_ENC1_POS_MATCH  = 66,
    XBAR_IN_ENC2_POS_MATCH  = 67,
    XBAR_IN_ENC3_POS_MATCH  = 68,
    XBAR_IN_ENC4_POS_MATCH  = 69,

    /* DMA done */
    XBAR_IN_DMA_DONE0       = 70,
    XBAR_IN_DMA_DONE1       = 71,
    XBAR_IN_DMA_DONE2       = 72,
    XBAR_IN_DMA_DONE3       = 73,
    XBAR_IN_DMA_DONE4       = 74,
    XBAR_IN_DMA_DONE5       = 75,
    XBAR_IN_DMA_DONE6       = 76,
    XBAR_IN_DMA_DONE7       = 77,

    /* GPT outputs */
    XBAR_IN_GPT1_COMPARE1   = 78,
    XBAR_IN_GPT1_COMPARE2   = 79,
    XBAR_IN_GPT1_COMPARE3   = 80,
    XBAR_IN_GPT1_CAPTURE1   = 81,
    XBAR_IN_GPT1_CAPTURE2   = 82,

    XBAR_IN_GPT2_COMPARE1   = 83,
    XBAR_IN_GPT2_COMPARE2   = 84,
    XBAR_IN_GPT2_COMPARE3   = 85,
    XBAR_IN_GPT2_CAPTURE1   = 86,
    XBAR_IN_GPT2_CAPTURE2   = 87,

} xbar_input_t;

/*
 * XBAR1 Output Signals
 */
typedef enum {
    /* DMA triggers */
    XBAR_OUT_DMA_CH_MUX_REQ30 = 0,
    XBAR_OUT_DMA_CH_MUX_REQ31 = 1,
    XBAR_OUT_DMA_CH_MUX_REQ94 = 2,
    XBAR_OUT_DMA_CH_MUX_REQ95 = 3,

    /* IOMUX outputs */
    XBAR_OUT_IOMUX_00       = 4,
    XBAR_OUT_IOMUX_01       = 5,
    XBAR_OUT_IOMUX_02       = 6,
    XBAR_OUT_IOMUX_03       = 7,
    XBAR_OUT_IOMUX_04       = 8,
    XBAR_OUT_IOMUX_05       = 9,
    XBAR_OUT_IOMUX_06       = 10,
    XBAR_OUT_IOMUX_07       = 11,
    XBAR_OUT_IOMUX_08       = 12,
    XBAR_OUT_IOMUX_09       = 13,
    XBAR_OUT_IOMUX_10       = 14,
    XBAR_OUT_IOMUX_11       = 15,
    XBAR_OUT_IOMUX_12       = 16,
    XBAR_OUT_IOMUX_13       = 17,

    /* ACMP sample */
    XBAR_OUT_ACMP1_SAMPLE   = 18,
    XBAR_OUT_ACMP2_SAMPLE   = 19,
    XBAR_OUT_ACMP3_SAMPLE   = 20,
    XBAR_OUT_ACMP4_SAMPLE   = 21,

    /* FlexPWM fault/force */
    XBAR_OUT_FLEXPWM1_FAULT0 = 28,
    XBAR_OUT_FLEXPWM1_FAULT1 = 29,
    XBAR_OUT_FLEXPWM1_FAULT2 = 30,
    XBAR_OUT_FLEXPWM1_FAULT3 = 31,
    XBAR_OUT_FLEXPWM1_FORCE  = 32,

    /* FlexPWM external sync */
    XBAR_OUT_FLEXPWM1_EXT_SYNC0 = 36,
    XBAR_OUT_FLEXPWM1_EXT_SYNC1 = 37,
    XBAR_OUT_FLEXPWM1_EXT_SYNC2 = 38,
    XBAR_OUT_FLEXPWM1_EXT_SYNC3 = 39,

    /* QTIMER triggers */
    XBAR_OUT_QTIMER1_TMR0_INPUT = 68,
    XBAR_OUT_QTIMER1_TMR1_INPUT = 69,
    XBAR_OUT_QTIMER1_TMR2_INPUT = 70,
    XBAR_OUT_QTIMER1_TMR3_INPUT = 71,

    /* ENC triggers */
    XBAR_OUT_ENC1_PHASE_A_INPUT = 84,
    XBAR_OUT_ENC1_PHASE_B_INPUT = 85,

    /* ADC_ETC triggers */
    XBAR_OUT_ADC_ETC_TRIG00 = 103,
    XBAR_OUT_ADC_ETC_TRIG01 = 104,
    XBAR_OUT_ADC_ETC_TRIG02 = 105,
    XBAR_OUT_ADC_ETC_TRIG03 = 106,
    XBAR_OUT_ADC_ETC_TRIG10 = 107,
    XBAR_OUT_ADC_ETC_TRIG11 = 108,
    XBAR_OUT_ADC_ETC_TRIG12 = 109,
    XBAR_OUT_ADC_ETC_TRIG13 = 110,
    XBAR_OUT_ADC_ETC_TRIG20 = 111,
    XBAR_OUT_ADC_ETC_TRIG21 = 112,
    XBAR_OUT_ADC_ETC_TRIG22 = 113,
    XBAR_OUT_ADC_ETC_TRIG23 = 114,

} xbar_output_t;

/*
 * XBAR API Functions
 */

/**
 * @brief Initialize XBAR subsystem
 *
 * Enables XBAR clocks and resets routing.
 *
 * @return NV_OK on success
 */
nv_error_t xbar_init(void);

/**
 * @brief Connect XBAR input to output
 *
 * @param input Input signal
 * @param output Output signal
 * @return NV_OK on success
 */
nv_error_t xbar_connect(xbar_input_t input, xbar_output_t output);

/**
 * @brief Disconnect XBAR output
 *
 * @param output Output signal to disconnect
 * @return NV_OK on success
 */
nv_error_t xbar_disconnect(xbar_output_t output);

/**
 * @brief Set up sequencer trigger routing
 *
 * Routes GPT1 compare to DMA trigger for deterministic GPIO updates.
 *
 * @return NV_OK on success
 */
nv_error_t xbar_setup_sequencer_trigger(void);

/**
 * @brief Set up ADC trigger routing
 *
 * Routes master pulse to ADC_ETC trigger input.
 *
 * @param source Trigger source (GPT compare, GPIO, etc.)
 * @param adc_trigger ADC_ETC trigger number
 * @return NV_OK on success
 */
nv_error_t xbar_setup_adc_trigger(xbar_input_t source, uint8_t adc_trigger);

/**
 * @brief Set up external trigger input routing
 *
 * Routes external trigger pin to sequencer start.
 *
 * @param gpio_xbar_input XBAR input for external trigger GPIO
 * @return NV_OK on success
 */
nv_error_t xbar_setup_external_trigger(xbar_input_t gpio_xbar_input);

/**
 * @brief Set up trigger output routing
 *
 * Routes internal trigger signal to external GPIO output.
 *
 * @param source Internal trigger source
 * @param gpio_xbar_output XBAR output connected to GPIO
 * @return NV_OK on success
 */
nv_error_t xbar_setup_trigger_output(xbar_input_t source, xbar_output_t gpio_xbar_output);

/**
 * @brief Debug: dump XBAR routing configuration
 *
 * Prints current XBAR connections to console.
 */
void xbar_dump_config(void);

/*
 * Predefined routing configurations for NV-Center Controller
 */

/**
 * @brief Apply standard NV-Center routing configuration
 *
 * Sets up:
 * - GPT1 compare → DMA trigger (GPIO updates)
 * - Master pulse GPIO → ADC_ETC trigger
 * - External trigger input → GPT1 capture
 * - Internal sync → Trigger output GPIO
 *
 * @return NV_OK on success
 */
nv_error_t xbar_apply_nv_config(void);

#ifdef __cplusplus
}
#endif

#endif /* XBAR_ROUTING_H */
