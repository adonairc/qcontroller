/*
 * NV-Center Controller - ADC Synchronization
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Synchronized ADC acquisition using ADC_ETC (External Trigger Control).
 *
 * Architecture:
 *
 * ┌─────────────┐     XBAR      ┌─────────────┐
 * │    GPT1     │──────────────▶│   ADC_ETC   │
 * │  (Compare)  │   Trigger     │  (Chain 0)  │
 * └─────────────┘               └──────┬──────┘
 *                                      │
 *                               ┌──────▼──────┐
 *                               │    ADC1     │
 *                               │  (12-bit)   │
 *                               └──────┬──────┘
 *                                      │ DMA
 *                               ┌──────▼──────┐
 *                               │   Buffer    │
 *                               │  (Ring)     │
 *                               └─────────────┘
 *
 * The ADC_ETC allows precise phase-aligned sampling with the pulse sequence.
 * Trigger events are generated from the master pulse for lock-in detection.
 */

#ifndef ADC_SYNC_H
#define ADC_SYNC_H

#include "nv_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ADC Configuration Constants
 */
#define ADC_MAX_CHANNELS        8
#define ADC_MAX_RESOLUTION      12
#define ADC_MAX_SAMPLE_RATE     3000000  /* 3 MSPS (12-bit) */

#ifndef CONFIG_NV_ADC_SAMPLE_BUFFER_SIZE
#define CONFIG_NV_ADC_SAMPLE_BUFFER_SIZE 1024
#endif

/*
 * ADC Trigger Sources
 */
typedef enum {
    ADC_TRIG_SOFTWARE   = 0,  /* Software trigger */
    ADC_TRIG_GPT1       = 1,  /* GPT1 compare match */
    ADC_TRIG_GPT2       = 2,  /* GPT2 compare match */
    ADC_TRIG_PWM        = 3,  /* FlexPWM trigger */
    ADC_TRIG_EXTERNAL   = 4,  /* External trigger input */
    ADC_TRIG_MASTER     = 5,  /* Master pulse edge */
} adc_trigger_source_t;

/*
 * ADC Status
 */
typedef enum {
    ADC_STATUS_IDLE         = 0,
    ADC_STATUS_CONFIGURED   = 1,
    ADC_STATUS_RUNNING      = 2,
    ADC_STATUS_COMPLETE     = 3,
    ADC_STATUS_ERROR        = 4,
} adc_status_t;

/*
 * ADC Configuration Structure
 */
typedef struct {
    uint8_t channel;            /* ADC input channel (0-7) */
    uint8_t resolution;         /* Resolution in bits (8, 10, 12) */
    uint8_t averaging;          /* Hardware averaging (1, 4, 8, 16, 32) */
    uint8_t trigger_source;     /* adc_trigger_source_t */
    uint32_t sample_count;      /* Number of samples to acquire */
    uint32_t sample_interval_ns; /* Interval between samples (if continuous) */
    bool continuous;            /* Continuous acquisition mode */
    bool double_buffer;         /* Use double buffering for streaming */
} adc_config_t;

/*
 * ADC Sample Buffer
 */
typedef struct {
    uint16_t *data;             /* Sample data buffer */
    uint32_t capacity;          /* Buffer capacity in samples */
    uint32_t count;             /* Number of valid samples */
    uint32_t read_index;        /* Read position (for streaming) */
    uint32_t write_index;       /* Write position (for streaming) */
    uint32_t timestamp;         /* Timestamp of first sample */
    bool overflow;              /* Buffer overflow flag */
} adc_buffer_t;

/*
 * ADC Statistics
 */
typedef struct {
    uint32_t triggers_received;
    uint32_t samples_acquired;
    uint32_t overflows;
    uint32_t underruns;
    uint16_t last_value;
    uint16_t min_value;
    uint16_t max_value;
    uint32_t sum;               /* For averaging */
} adc_statistics_t;

/*
 * Callback types
 */
typedef void (*adc_data_callback_t)(const uint16_t *data,
                                    uint32_t count,
                                    uint32_t timestamp,
                                    void *user_data);

typedef void (*adc_trigger_callback_t)(uint32_t timestamp, void *user_data);

/*
 * ADC Synchronization API
 */

/**
 * @brief Initialize ADC subsystem
 *
 * @return NV_OK on success
 */
nv_error_t adc_sync_init(void);

/**
 * @brief Deinitialize ADC subsystem
 *
 * @return NV_OK on success
 */
nv_error_t adc_sync_deinit(void);

/**
 * @brief Configure ADC channel
 *
 * @param config ADC configuration
 * @return NV_OK on success
 */
nv_error_t adc_sync_configure(const adc_config_t *config);

/**
 * @brief Start ADC acquisition
 *
 * @return NV_OK on success
 */
nv_error_t adc_sync_start(void);

/**
 * @brief Stop ADC acquisition
 *
 * @return NV_OK on success
 */
nv_error_t adc_sync_stop(void);

/**
 * @brief Issue software trigger
 *
 * @return NV_OK on success
 */
nv_error_t adc_sync_trigger(void);

/**
 * @brief Get current ADC status
 *
 * @return ADC status
 */
adc_status_t adc_sync_get_status(void);

/**
 * @brief Get sample buffer
 *
 * @param buffer Pointer to buffer structure to fill
 * @return NV_OK on success
 */
nv_error_t adc_sync_get_buffer(adc_buffer_t *buffer);

/**
 * @brief Read samples from buffer
 *
 * @param data Destination buffer
 * @param max_samples Maximum samples to read
 * @param samples_read Actual samples read (output)
 * @return NV_OK on success
 */
nv_error_t adc_sync_read(uint16_t *data, uint32_t max_samples, uint32_t *samples_read);

/**
 * @brief Get ADC statistics
 *
 * @param stats Pointer to statistics structure
 */
void adc_sync_get_statistics(adc_statistics_t *stats);

/**
 * @brief Reset ADC statistics
 */
void adc_sync_reset_statistics(void);

/**
 * @brief Set data ready callback
 *
 * @param callback Callback function
 * @param user_data User context
 */
void adc_sync_set_data_callback(adc_data_callback_t callback, void *user_data);

/**
 * @brief Set trigger callback
 *
 * @param callback Callback function
 * @param user_data User context
 */
void adc_sync_set_trigger_callback(adc_trigger_callback_t callback, void *user_data);

/**
 * @brief Synchronize ADC with sequencer master pulse
 *
 * Sets up XBAR routing so ADC triggers on master pulse edge.
 *
 * @param offset_ns Offset from master pulse edge in nanoseconds
 * @param sample_count Number of samples per trigger
 * @return NV_OK on success
 */
nv_error_t adc_sync_with_master(int32_t offset_ns, uint32_t sample_count);

/**
 * @brief Enable/disable lock-in integration mode
 *
 * When enabled, ADC accumulates samples for lock-in detection.
 *
 * @param enable Enable flag
 * @param reference_channel Channel providing reference (MASTER)
 * @return NV_OK on success
 */
nv_error_t adc_sync_lockin_mode(bool enable, nv_channel_t reference_channel);

/**
 * @brief Get lock-in integrated result
 *
 * @param in_phase In-phase (X) component
 * @param quadrature Quadrature (Y) component
 * @return NV_OK on success
 */
nv_error_t adc_sync_lockin_result(int32_t *in_phase, int32_t *quadrature);

/*
 * Low-level hardware functions
 */

/**
 * @brief Configure ADC_ETC trigger chain
 *
 * @param chain_id Chain ID (0-7)
 * @param trigger_source Hardware trigger source
 * @return NV_OK on success
 */
nv_error_t adc_etc_configure_chain(uint8_t chain_id, uint8_t trigger_source);

/**
 * @brief Configure XBAR routing for ADC trigger
 *
 * @param source XBAR input signal
 * @param dest XBAR output to ADC_ETC
 * @return NV_OK on success
 */
nv_error_t adc_xbar_route(uint8_t source, uint8_t dest);

/**
 * @brief Setup DMA for ADC transfers
 *
 * @param buffer Destination buffer
 * @param count Number of transfers
 * @param circular Enable circular buffer mode
 * @return NV_OK on success
 */
nv_error_t adc_dma_setup(uint16_t *buffer, uint32_t count, bool circular);

#ifdef __cplusplus
}
#endif

#endif /* ADC_SYNC_H */
