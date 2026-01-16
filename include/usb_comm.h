/*
 * NV-Center Controller - USB Communication
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB CDC ACM communication protocol for host UI interface.
 *
 * Protocol Structure (binary, little-endian):
 *
 * Command Frame:
 * ┌──────────────────────────────────────────────────────────────────┐
 * │ SYNC (2) │ CMD (1) │ FLAGS (1) │ LENGTH (2) │ PAYLOAD (N) │ CRC (2) │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * Response Frame:
 * ┌──────────────────────────────────────────────────────────────────┐
 * │ SYNC (2) │ RSP (1) │ STATUS (1) │ LENGTH (2) │ PAYLOAD (N) │ CRC (2) │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * SYNC = 0x4E56 ("NV")
 */

#ifndef USB_COMM_H
#define USB_COMM_H

#include "nv_controller.h"
#include "event_table.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Protocol Constants
 */
#define USB_SYNC_WORD           0x564E      /* "NV" little-endian */
#define USB_MAX_PAYLOAD_SIZE    8192
#define USB_HEADER_SIZE         6
#define USB_CRC_SIZE            2
#define USB_MAX_FRAME_SIZE      (USB_HEADER_SIZE + USB_MAX_PAYLOAD_SIZE + USB_CRC_SIZE)

/*
 * Command Codes
 */
typedef enum {
    /* System Commands (0x00-0x0F) */
    CMD_NOP             = 0x00,  /* No operation (ping) */
    CMD_GET_INFO        = 0x01,  /* Get system information */
    CMD_GET_STATUS      = 0x02,  /* Get current status */
    CMD_GET_STATS       = 0x03,  /* Get statistics */
    CMD_RESET           = 0x04,  /* System reset */
    CMD_RESET_STATS     = 0x05,  /* Reset statistics */

    /* Sequencer Commands (0x10-0x1F) */
    CMD_SEQ_LOAD        = 0x10,  /* Load pulse sequence */
    CMD_SEQ_LOAD_CHUNK  = 0x11,  /* Load sequence chunk (for large sequences) */
    CMD_SEQ_ARM         = 0x12,  /* Arm sequencer */
    CMD_SEQ_DISARM      = 0x13,  /* Disarm sequencer */
    CMD_SEQ_TRIGGER     = 0x14,  /* Software trigger */
    CMD_SEQ_ABORT       = 0x15,  /* Abort sequence */
    CMD_SEQ_PAUSE       = 0x16,  /* Pause sequence */
    CMD_SEQ_RESUME      = 0x17,  /* Resume sequence */
    CMD_SEQ_CONFIG      = 0x18,  /* Configure sequencer */
    CMD_SEQ_GET_STATE   = 0x19,  /* Get sequencer state */

    /* Direct I/O Commands (0x20-0x2F) */
    CMD_IO_SET_ALL      = 0x20,  /* Set all outputs */
    CMD_IO_SET_CHANNEL  = 0x21,  /* Set single channel */
    CMD_IO_GET_ALL      = 0x22,  /* Get all output states */
    CMD_IO_PULSE        = 0x23,  /* Generate single pulse */

    /* ADC Commands (0x30-0x3F) */
    CMD_ADC_CONFIG      = 0x30,  /* Configure ADC */
    CMD_ADC_START       = 0x31,  /* Start ADC acquisition */
    CMD_ADC_STOP        = 0x32,  /* Stop ADC acquisition */
    CMD_ADC_GET_DATA    = 0x33,  /* Get ADC data buffer */
    CMD_ADC_GET_STATUS  = 0x34,  /* Get ADC status */

    /* Preset Sequences (0x40-0x4F) */
    CMD_PRESET_RABI     = 0x40,  /* Generate Rabi sequence */
    CMD_PRESET_RAMSEY   = 0x41,  /* Generate Ramsey sequence */
    CMD_PRESET_ECHO     = 0x42,  /* Generate Hahn echo sequence */
    CMD_PRESET_ODMR     = 0x43,  /* Generate ODMR sequence */

    /* Storage Commands (0x50-0x5F) */
    CMD_STORE_SAVE      = 0x50,  /* Save sequence to flash */
    CMD_STORE_LOAD      = 0x51,  /* Load sequence from flash */
    CMD_STORE_DELETE    = 0x52,  /* Delete stored sequence */
    CMD_STORE_LIST      = 0x53,  /* List stored sequences */

    /* Debug Commands (0xF0-0xFF) */
    CMD_DEBUG_READ_REG  = 0xF0,  /* Read hardware register */
    CMD_DEBUG_WRITE_REG = 0xF1,  /* Write hardware register */
    CMD_DEBUG_LOOPBACK  = 0xFE,  /* Echo payload back */
    CMD_DEBUG_TEST      = 0xFF,  /* Run self-test */
} usb_command_t;

/*
 * Response Status Codes
 */
typedef enum {
    RSP_OK              = 0x00,  /* Command successful */
    RSP_ERR_UNKNOWN_CMD = 0x01,  /* Unknown command */
    RSP_ERR_INVALID_ARG = 0x02,  /* Invalid argument */
    RSP_ERR_BUSY        = 0x03,  /* System busy */
    RSP_ERR_NOT_READY   = 0x04,  /* Not ready for command */
    RSP_ERR_OVERFLOW    = 0x05,  /* Buffer overflow */
    RSP_ERR_CRC         = 0x06,  /* CRC mismatch */
    RSP_ERR_TIMEOUT     = 0x07,  /* Operation timeout */
    RSP_ERR_HARDWARE    = 0x08,  /* Hardware error */
    RSP_ERR_SEQUENCE    = 0x09,  /* Invalid sequence */
    RSP_ERR_STORAGE     = 0x0A,  /* Storage error */
} usb_response_status_t;

/*
 * Command Flags
 */
typedef enum {
    FLAG_NONE           = 0x00,
    FLAG_ASYNC          = 0x01,  /* Async response expected */
    FLAG_CHUNKED        = 0x02,  /* More chunks follow */
    FLAG_LAST_CHUNK     = 0x04,  /* Final chunk */
    FLAG_PRIORITY       = 0x08,  /* High priority command */
} usb_command_flags_t;

/*
 * Frame Header Structure
 */
typedef struct __attribute__((packed)) {
    uint16_t sync;      /* Sync word (0x564E) */
    uint8_t command;    /* Command code */
    uint8_t flags;      /* Command flags */
    uint16_t length;    /* Payload length */
} usb_frame_header_t;

_Static_assert(sizeof(usb_frame_header_t) == USB_HEADER_SIZE, "Header size mismatch");

/*
 * Command Payload Structures
 */

/* CMD_SEQ_CONFIG payload */
typedef struct __attribute__((packed)) {
    uint8_t mode;           /* seq_mode_t */
    uint8_t trigger;        /* nv_trigger_mode_t */
    uint8_t auto_rearm;     /* Boolean */
    uint8_t sync_adc;       /* Boolean */
    uint32_t repeat_count;
    uint32_t trigger_delay;
    uint32_t adc_offset;
} cmd_seq_config_t;

/* CMD_IO_SET_CHANNEL payload */
typedef struct __attribute__((packed)) {
    uint8_t channel;
    uint8_t state;
} cmd_io_set_channel_t;

/* CMD_IO_PULSE payload */
typedef struct __attribute__((packed)) {
    uint8_t channel;
    uint8_t reserved;
    uint32_t duration_ns;
} cmd_io_pulse_t;

/* CMD_ADC_CONFIG payload */
typedef struct __attribute__((packed)) {
    uint8_t channel;
    uint8_t resolution;     /* 8, 10, or 12 bits */
    uint16_t samples;       /* Number of samples */
    uint32_t sample_rate;   /* Samples per second */
    uint8_t trigger_source; /* ADC trigger source */
    uint8_t averaging;      /* Hardware averaging count */
    uint16_t reserved;
} cmd_adc_config_t;

/* CMD_PRESET_RABI payload */
typedef struct __attribute__((packed)) {
    uint32_t init_us;
    uint32_t mw_ns;
    uint32_t readout_us;
    uint32_t repeat;
} cmd_preset_rabi_t;

/* CMD_PRESET_RAMSEY payload */
typedef struct __attribute__((packed)) {
    uint32_t init_us;
    uint32_t pi2_ns;
    uint32_t tau_ns;
    uint32_t readout_us;
} cmd_preset_ramsey_t;

/* CMD_PRESET_ECHO payload */
typedef struct __attribute__((packed)) {
    uint32_t init_us;
    uint32_t pi2_ns;
    uint32_t pi_ns;
    uint32_t tau_ns;
    uint32_t readout_us;
} cmd_preset_echo_t;

/* CMD_STORE_SAVE payload */
typedef struct __attribute__((packed)) {
    uint8_t slot;
    uint8_t name_length;
    char name[30];
} cmd_store_save_t;

/*
 * Response Payload Structures
 */

/* RSP to CMD_GET_INFO */
typedef struct __attribute__((packed)) {
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
    uint8_t channel_count;
    uint32_t timer_freq_hz;
    uint32_t min_pulse_ns;
    uint32_t max_events;
    uint32_t dma_buffer_size;
    char build_date[16];
} rsp_system_info_t;

/* RSP to CMD_GET_STATUS */
typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t armed;
    uint8_t running;
    uint8_t reserved;
    uint32_t uptime_ms;
} rsp_system_status_t;

/* RSP to CMD_GET_STATS */
typedef struct __attribute__((packed)) {
    uint32_t sequences_completed;
    uint32_t events_executed;
    uint32_t trigger_count;
    uint32_t overflow_count;
    uint32_t underrun_count;
    uint32_t max_latency_ns;
    uint32_t last_duration_us;
} rsp_statistics_t;

/* RSP to CMD_SEQ_GET_STATE */
typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t current_output;
    uint16_t reserved;
    uint32_t current_index;
    uint32_t events_remaining;
    uint32_t loops_completed;
    uint32_t timestamp;
} rsp_seq_state_t;

/* RSP to CMD_ADC_GET_STATUS */
typedef struct __attribute__((packed)) {
    uint8_t running;
    uint8_t triggered;
    uint16_t samples_collected;
    uint32_t timestamp;
} rsp_adc_status_t;

/*
 * USB Communication API
 */

/**
 * @brief Initialize USB CDC communication
 *
 * @return NV_OK on success
 */
nv_error_t usb_comm_init(void);

/**
 * @brief Process incoming USB data
 *
 * Call from main loop or dedicated thread.
 */
void usb_comm_process(void);

/**
 * @brief Check if USB is connected
 *
 * @return true if host connected
 */
bool usb_comm_is_connected(void);

/**
 * @brief Send response frame
 *
 * @param command Original command code
 * @param status Response status
 * @param payload Response payload (can be NULL)
 * @param length Payload length
 * @return NV_OK on success
 */
nv_error_t usb_comm_send_response(uint8_t command,
                                  usb_response_status_t status,
                                  const void *payload,
                                  uint16_t length);

/**
 * @brief Send async event notification
 *
 * @param event_type Event type code
 * @param payload Event payload
 * @param length Payload length
 * @return NV_OK on success
 */
nv_error_t usb_comm_send_event(uint8_t event_type,
                               const void *payload,
                               uint16_t length);

/**
 * @brief Send ADC data buffer
 *
 * @param data ADC sample buffer
 * @param samples Number of samples
 * @return NV_OK on success
 */
nv_error_t usb_comm_send_adc_data(const uint16_t *data, uint32_t samples);

/**
 * @brief Set command handler callback
 *
 * @param handler Handler function
 */
typedef nv_error_t (*usb_command_handler_t)(uint8_t command,
                                            const void *payload,
                                            uint16_t length);
void usb_comm_set_handler(usb_command_handler_t handler);

/**
 * @brief Get USB statistics
 *
 * @param rx_bytes Total bytes received
 * @param tx_bytes Total bytes transmitted
 * @param errors Total frame errors
 */
void usb_comm_get_stats(uint32_t *rx_bytes, uint32_t *tx_bytes, uint32_t *errors);

/**
 * @brief Calculate CRC16 for frame
 *
 * Uses CRC-16-CCITT polynomial.
 *
 * @param data Data buffer
 * @param length Data length
 * @return CRC16 value
 */
uint16_t usb_comm_crc16(const uint8_t *data, size_t length);

/*
 * Async event types
 */
typedef enum {
    EVENT_SEQ_COMPLETE  = 0x80,
    EVENT_SEQ_LOOP      = 0x81,
    EVENT_SEQ_ERROR     = 0x82,
    EVENT_TRIGGER       = 0x83,
    EVENT_ADC_READY     = 0x84,
    EVENT_ADC_OVERFLOW  = 0x85,
} usb_event_type_t;

#ifdef __cplusplus
}
#endif

#endif /* USB_COMM_H */
