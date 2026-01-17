/*
 * NV-Center Controller - USB Communication Implementation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB CDC ACM protocol handler for host UI communication.
 * Uses Zephyr's CDC-ACM console snippet approach.
 *
 * Build with: west build -b teensy41 -S cdc-acm-console
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "usb_comm.h"
#include "sequencer.h"
#include "event_table.h"
#include "adc_sync.h"

LOG_MODULE_REGISTER(usb_comm, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Buffer sizes
 */
#ifndef CONFIG_NV_USB_RX_BUFFER_SIZE
#define CONFIG_NV_USB_RX_BUFFER_SIZE 4096
#endif

#ifndef CONFIG_NV_USB_TX_BUFFER_SIZE
#define CONFIG_NV_USB_TX_BUFFER_SIZE 2048
#endif

/*
 * USB Communication Context
 */
typedef struct {
    const struct device *uart_dev;

    /* Receive state machine */
    enum {
        RX_STATE_SYNC_1,
        RX_STATE_SYNC_2,
        RX_STATE_HEADER,
        RX_STATE_PAYLOAD,
        RX_STATE_CRC,
    } rx_state;

    /* Current frame being received */
    usb_frame_header_t rx_header;
    uint8_t rx_payload[USB_MAX_PAYLOAD_SIZE];
    uint16_t rx_payload_idx;
    uint16_t rx_crc_received;
    uint8_t rx_crc_idx;

    /* Ring buffers */
    struct ring_buf rx_ring;
    uint8_t rx_ring_buffer[CONFIG_NV_USB_RX_BUFFER_SIZE];
    struct ring_buf tx_ring;
    uint8_t tx_ring_buffer[CONFIG_NV_USB_TX_BUFFER_SIZE];

    /* Transmit buffer for frame assembly */
    uint8_t tx_frame[USB_MAX_FRAME_SIZE];

    /* Statistics */
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t frame_errors;
    uint32_t crc_errors;

    /* Connection status */
    bool connected;
    bool dtr_set;

    /* Chunked transfer state */
    event_table_t *chunk_table;
    uint32_t chunk_events_received;

    /* Command handler */
    usb_command_handler_t command_handler;

    /* Mutex for thread safety */
    struct k_mutex tx_mutex;

} usb_comm_context_t;

static usb_comm_context_t usb_ctx;

/* Event table for sequence loading */
static event_table_t loaded_table;

/*
 * Build date string
 */
static const char build_date[] = __DATE__;

/*
 * CRC-16-CCITT lookup table
 */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

uint16_t usb_comm_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }

    return crc;
}

/*
 * UART interrupt callback
 */
static void uart_irq_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uint8_t buf[64];
        int len = uart_fifo_read(dev, buf, sizeof(buf));

        if (len > 0) {
            ring_buf_put(&usb_ctx.rx_ring, buf, len);
            usb_ctx.rx_bytes += len;
        }
    }

    if (uart_irq_tx_ready(dev)) {
        uint8_t buf[64];
        int len = ring_buf_get(&usb_ctx.tx_ring, buf, sizeof(buf));

        if (len > 0) {
            uart_fifo_fill(dev, buf, len);
        } else {
            uart_irq_tx_disable(dev);
        }
    }
}

/*
 * Process a complete received frame
 */
static void process_frame(void)
{
    uint8_t cmd = usb_ctx.rx_header.command;
    uint16_t len = usb_ctx.rx_header.length;
    const uint8_t *payload = usb_ctx.rx_payload;

    LOG_DBG("Processing command 0x%02X, payload %u bytes", cmd, len);

    nv_error_t result = NV_OK;
    usb_response_status_t status = RSP_OK;

    switch (cmd) {
    /*
     * System Commands
     */
    case CMD_NOP:
        /* Ping - just respond OK */
        usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        break;

    case CMD_GET_INFO: {
        rsp_system_info_t info = {
            .version_major = NV_CONTROLLER_VERSION_MAJOR,
            .version_minor = NV_CONTROLLER_VERSION_MINOR,
            .version_patch = NV_CONTROLLER_VERSION_PATCH,
            .channel_count = NV_CHANNEL_COUNT,
            .timer_freq_hz = NV_TIMER_FREQ_HZ,
            .min_pulse_ns = NV_TIMER_TICK_NS * NV_MIN_PULSE_TICKS,
            .max_events = CONFIG_NV_SEQUENCER_MAX_EVENTS,
            .dma_buffer_size = CONFIG_NV_SEQUENCER_DMA_BUFFER_SIZE,
        };
        strncpy(info.build_date, build_date, sizeof(info.build_date) - 1);
        usb_comm_send_response(cmd, RSP_OK, &info, sizeof(info));
        break;
    }

    case CMD_GET_STATUS: {
        seq_state_t seq_state;
        seq_get_state(&seq_state);

        rsp_system_status_t status_rsp = {
            .status = seq_state.status,
            .armed = (seq_state.status == NV_STATUS_ARMED) ? 1 : 0,
            .running = (seq_state.status == NV_STATUS_RUNNING) ? 1 : 0,
            .uptime_ms = k_uptime_get_32(),
        };
        usb_comm_send_response(cmd, RSP_OK, &status_rsp, sizeof(status_rsp));
        break;
    }

    case CMD_RESET:
        seq_abort();
        usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        break;

    /*
     * Sequencer Commands
     */
    case CMD_SEQ_LOAD:
        if (len < sizeof(event_table_header_t)) {
            usb_comm_send_response(cmd, RSP_ERR_INVALID_ARG, NULL, 0);
            break;
        }

        result = event_table_deserialize(&loaded_table, payload, len);
        if (result == NV_OK) {
            result = seq_load(&loaded_table);
        }

        status = (result == NV_OK) ? RSP_OK : RSP_ERR_SEQUENCE;
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_SEQ_LOAD_CHUNK: {
        uint8_t flags = usb_ctx.rx_header.flags;

        if (flags & FLAG_CHUNKED) {
            if (usb_ctx.chunk_table == NULL) {
                usb_ctx.chunk_table = &loaded_table;
                event_table_init(usb_ctx.chunk_table);
                usb_ctx.chunk_events_received = 0;
            }

            size_t event_count = len / sizeof(pulse_event_t);
            const pulse_event_t *events = (const pulse_event_t *)payload;

            for (size_t i = 0; i < event_count; i++) {
                result = event_table_insert(usb_ctx.chunk_table,
                                           usb_ctx.chunk_events_received + i,
                                           &events[i]);
                if (result != NV_OK) break;
            }
            usb_ctx.chunk_events_received += event_count;
        }

        if (flags & FLAG_LAST_CHUNK) {
            if (usb_ctx.chunk_table) {
                event_table_sort(usb_ctx.chunk_table);
                result = seq_load(usb_ctx.chunk_table);
                usb_ctx.chunk_table = NULL;
            }
        }

        status = (result == NV_OK) ? RSP_OK : RSP_ERR_SEQUENCE;
        usb_comm_send_response(cmd, status, NULL, 0);
        break;
    }

    case CMD_SEQ_ARM:
        result = seq_arm();
        status = (result == NV_OK) ? RSP_OK : RSP_ERR_NOT_READY;
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_SEQ_DISARM:
        result = seq_disarm();
        usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        break;

    case CMD_SEQ_TRIGGER:
        result = seq_trigger();
        status = (result == NV_OK) ? RSP_OK : RSP_ERR_NOT_READY;
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_SEQ_ABORT:
        seq_abort();
        usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        break;

    case CMD_SEQ_PAUSE:
        result = seq_pause();
        status = (result == NV_OK) ? RSP_OK : RSP_ERR_NOT_READY;
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_SEQ_RESUME:
        result = seq_resume();
        status = (result == NV_OK) ? RSP_OK : RSP_ERR_NOT_READY;
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_SEQ_CONFIG:
        if (len >= sizeof(cmd_seq_config_t)) {
            const cmd_seq_config_t *cfg = (const cmd_seq_config_t *)payload;
            seq_config_t config = {
                .mode = cfg->mode,
                .trigger = cfg->trigger,
                .repeat_count = cfg->repeat_count,
                .trigger_delay_ticks = cfg->trigger_delay,
                .auto_rearm = cfg->auto_rearm,
                .sync_adc = cfg->sync_adc,
                .adc_trigger_offset = cfg->adc_offset,
            };
            result = seq_configure(&config);
            status = (result == NV_OK) ? RSP_OK : RSP_ERR_INVALID_ARG;
        } else {
            status = RSP_ERR_INVALID_ARG;
        }
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_SEQ_GET_STATE: {
        seq_state_t seq_state;
        seq_get_state(&seq_state);

        rsp_seq_state_t state_rsp = {
            .status = seq_state.status,
            .current_output = seq_state.current_output,
            .current_index = seq_state.current_event_index,
            .events_remaining = seq_state.events_remaining,
            .loops_completed = seq_state.loops_completed,
            .timestamp = seq_state.current_timestamp,
        };
        usb_comm_send_response(cmd, RSP_OK, &state_rsp, sizeof(state_rsp));
        break;
    }

    /*
     * Direct I/O Commands
     */
    case CMD_IO_SET_ALL:
        if (len >= 1) {
            seq_set_outputs(payload[0]);
            usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        } else {
            usb_comm_send_response(cmd, RSP_ERR_INVALID_ARG, NULL, 0);
        }
        break;

    case CMD_IO_SET_CHANNEL:
        if (len >= sizeof(cmd_io_set_channel_t)) {
            const cmd_io_set_channel_t *io_cmd = (const cmd_io_set_channel_t *)payload;
            seq_set_channel(io_cmd->channel, io_cmd->state);
            usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        } else {
            usb_comm_send_response(cmd, RSP_ERR_INVALID_ARG, NULL, 0);
        }
        break;

    case CMD_IO_GET_ALL: {
        uint8_t outputs = seq_get_outputs();
        usb_comm_send_response(cmd, RSP_OK, &outputs, 1);
        break;
    }

    case CMD_IO_PULSE:
        if (len >= sizeof(cmd_io_pulse_t)) {
            const cmd_io_pulse_t *pulse_cmd = (const cmd_io_pulse_t *)payload;
            seq_set_channel(pulse_cmd->channel, true);
            k_busy_wait(pulse_cmd->duration_ns / 1000);
            seq_set_channel(pulse_cmd->channel, false);
            usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        } else {
            usb_comm_send_response(cmd, RSP_ERR_INVALID_ARG, NULL, 0);
        }
        break;

    /*
     * ADC Commands
     */
    case CMD_ADC_CONFIG:
        if (len >= sizeof(cmd_adc_config_t)) {
            const cmd_adc_config_t *adc_cmd = (const cmd_adc_config_t *)payload;
            adc_config_t config = {
                .channel = adc_cmd->channel,
                .resolution = adc_cmd->resolution,
                .averaging = adc_cmd->averaging,
                .trigger_source = adc_cmd->trigger_source,
                .sample_count = adc_cmd->samples,
                .sample_interval_ns = 1000000000UL / adc_cmd->sample_rate,
            };
            result = adc_sync_configure(&config);
            status = (result == NV_OK) ? RSP_OK : RSP_ERR_INVALID_ARG;
        } else {
            status = RSP_ERR_INVALID_ARG;
        }
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_ADC_START:
        result = adc_sync_start();
        status = (result == NV_OK) ? RSP_OK : RSP_ERR_NOT_READY;
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_ADC_STOP:
        result = adc_sync_stop();
        usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        break;

    case CMD_ADC_GET_DATA: {
        adc_buffer_t buffer;
        result = adc_sync_get_buffer(&buffer);
        if (result == NV_OK && buffer.count > 0) {
            usb_comm_send_adc_data(buffer.data, buffer.count);
        } else {
            usb_comm_send_response(cmd, RSP_ERR_NOT_READY, NULL, 0);
        }
        break;
    }

    case CMD_ADC_GET_STATUS: {
        adc_status_t adc_status = adc_sync_get_status();
        adc_buffer_t buffer;
        adc_sync_get_buffer(&buffer);

        rsp_adc_status_t status_rsp = {
            .running = (adc_status == ADC_STATUS_RUNNING) ? 1 : 0,
            .triggered = (adc_status == ADC_STATUS_COMPLETE) ? 1 : 0,
            .samples_collected = buffer.count,
            .timestamp = buffer.timestamp,
        };
        usb_comm_send_response(cmd, RSP_OK, &status_rsp, sizeof(status_rsp));
        break;
    }

    /*
     * Preset Sequences
     */
    case CMD_PRESET_RABI:
        if (len >= sizeof(cmd_preset_rabi_t)) {
            const cmd_preset_rabi_t *rabi = (const cmd_preset_rabi_t *)payload;
            result = event_table_rabi(&loaded_table, rabi->init_us,
                                     rabi->mw_ns, rabi->readout_us, rabi->repeat);
            if (result == NV_OK) {
                result = seq_load(&loaded_table);
            }
            status = (result == NV_OK) ? RSP_OK : RSP_ERR_SEQUENCE;
        } else {
            status = RSP_ERR_INVALID_ARG;
        }
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_PRESET_RAMSEY:
        if (len >= sizeof(cmd_preset_ramsey_t)) {
            const cmd_preset_ramsey_t *ramsey = (const cmd_preset_ramsey_t *)payload;
            result = event_table_ramsey(&loaded_table, ramsey->init_us,
                                       ramsey->pi2_ns, ramsey->tau_ns, ramsey->readout_us);
            if (result == NV_OK) {
                result = seq_load(&loaded_table);
            }
            status = (result == NV_OK) ? RSP_OK : RSP_ERR_SEQUENCE;
        } else {
            status = RSP_ERR_INVALID_ARG;
        }
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    case CMD_PRESET_ECHO:
        if (len >= sizeof(cmd_preset_echo_t)) {
            const cmd_preset_echo_t *echo = (const cmd_preset_echo_t *)payload;
            result = event_table_hahn_echo(&loaded_table, echo->init_us,
                                          echo->pi2_ns, echo->pi_ns,
                                          echo->tau_ns, echo->readout_us);
            if (result == NV_OK) {
                result = seq_load(&loaded_table);
            }
            status = (result == NV_OK) ? RSP_OK : RSP_ERR_SEQUENCE;
        } else {
            status = RSP_ERR_INVALID_ARG;
        }
        usb_comm_send_response(cmd, status, NULL, 0);
        break;

    /*
     * Debug Commands
     */
    case CMD_DEBUG_LOOPBACK:
        usb_comm_send_response(cmd, RSP_OK, payload, len);
        break;

    case CMD_DEBUG_TEST:
        usb_comm_send_response(cmd, RSP_OK, NULL, 0);
        break;

    default:
        LOG_WRN("Unknown command: 0x%02X", cmd);
        usb_comm_send_response(cmd, RSP_ERR_UNKNOWN_CMD, NULL, 0);
        break;
    }
}

/*
 * Frame receive state machine
 */
static void process_rx_byte(uint8_t byte)
{
    switch (usb_ctx.rx_state) {
    case RX_STATE_SYNC_1:
        if (byte == (USB_SYNC_WORD & 0xFF)) {
            usb_ctx.rx_state = RX_STATE_SYNC_2;
        }
        break;

    case RX_STATE_SYNC_2:
        if (byte == ((USB_SYNC_WORD >> 8) & 0xFF)) {
            usb_ctx.rx_state = RX_STATE_HEADER;
            usb_ctx.rx_payload_idx = 0;
            memset(&usb_ctx.rx_header, 0, sizeof(usb_ctx.rx_header));
            usb_ctx.rx_header.sync = USB_SYNC_WORD;
        } else if (byte == (USB_SYNC_WORD & 0xFF)) {
            /* Stay in SYNC_2 if we see another sync byte */
        } else {
            usb_ctx.rx_state = RX_STATE_SYNC_1;
        }
        break;

    case RX_STATE_HEADER:
        ((uint8_t *)&usb_ctx.rx_header)[2 + usb_ctx.rx_payload_idx] = byte;
        usb_ctx.rx_payload_idx++;

        if (usb_ctx.rx_payload_idx >= 4) {
            if (usb_ctx.rx_header.length > USB_MAX_PAYLOAD_SIZE) {
                LOG_WRN("Invalid frame length: %u", usb_ctx.rx_header.length);
                usb_ctx.frame_errors++;
                usb_ctx.rx_state = RX_STATE_SYNC_1;
            } else if (usb_ctx.rx_header.length == 0) {
                usb_ctx.rx_state = RX_STATE_CRC;
                usb_ctx.rx_crc_idx = 0;
            } else {
                usb_ctx.rx_state = RX_STATE_PAYLOAD;
                usb_ctx.rx_payload_idx = 0;
            }
        }
        break;

    case RX_STATE_PAYLOAD:
        usb_ctx.rx_payload[usb_ctx.rx_payload_idx++] = byte;

        if (usb_ctx.rx_payload_idx >= usb_ctx.rx_header.length) {
            usb_ctx.rx_state = RX_STATE_CRC;
            usb_ctx.rx_crc_idx = 0;
        }
        break;

    case RX_STATE_CRC:
        if (usb_ctx.rx_crc_idx == 0) {
            usb_ctx.rx_crc_received = byte;
            usb_ctx.rx_crc_idx = 1;
        } else {
            usb_ctx.rx_crc_received |= (uint16_t)byte << 8;

            /* Compute CRC over header and payload */
            uint16_t calc_crc = 0xFFFF;
            for (int i = 0; i < USB_HEADER_SIZE; i++) {
                calc_crc = (calc_crc << 8) ^
                    crc16_table[((calc_crc >> 8) ^ ((uint8_t *)&usb_ctx.rx_header)[i]) & 0xFF];
            }
            for (int i = 0; i < usb_ctx.rx_header.length; i++) {
                calc_crc = (calc_crc << 8) ^
                    crc16_table[((calc_crc >> 8) ^ usb_ctx.rx_payload[i]) & 0xFF];
            }

            if (calc_crc == usb_ctx.rx_crc_received) {
                process_frame();
            } else {
                LOG_WRN("CRC mismatch: expected 0x%04X, got 0x%04X",
                        calc_crc, usb_ctx.rx_crc_received);
                usb_ctx.crc_errors++;
                usb_ctx.frame_errors++;
            }

            usb_ctx.rx_state = RX_STATE_SYNC_1;
        }
        break;
    }
}

/*
 * Public API
 */

nv_error_t usb_comm_init(void)
{
    LOG_INF("Initializing USB CDC communication");

    memset(&usb_ctx, 0, sizeof(usb_ctx));

    /* Initialize ring buffers */
    ring_buf_init(&usb_ctx.rx_ring, sizeof(usb_ctx.rx_ring_buffer),
                  usb_ctx.rx_ring_buffer);
    ring_buf_init(&usb_ctx.tx_ring, sizeof(usb_ctx.tx_ring_buffer),
                  usb_ctx.tx_ring_buffer);

    /* Initialize mutex */
    k_mutex_init(&usb_ctx.tx_mutex);

    /*
     * Get UART device from chosen node (set by cdc-acm-console snippet)
     * The snippet creates the CDC-ACM UART and sets it as zephyr,console
     */
#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_console), okay)
    usb_ctx.uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#else
    LOG_ERR("No console device found in device tree");
    return NV_ERR_HARDWARE;
#endif

    if (!device_is_ready(usb_ctx.uart_dev)) {
        LOG_ERR("UART device not ready");
        return NV_ERR_HARDWARE;
    }

    LOG_INF("Using UART device: %s", usb_ctx.uart_dev->name);

    /*
     * With the new USB device stack (usbd) and cdc-acm-console snippet,
     * USB is automatically enabled during system initialization.
     * We just need to wait for the device to be ready.
     */
    k_msleep(100);

    /* Configure UART interrupt callback */
    uart_irq_callback_user_data_set(usb_ctx.uart_dev, uart_irq_callback, NULL);

    /* Enable RX interrupt */
    uart_irq_rx_enable(usb_ctx.uart_dev);

    usb_ctx.rx_state = RX_STATE_SYNC_1;
    usb_ctx.connected = true;

    LOG_INF("USB CDC initialized successfully");

    return NV_OK;
}

void usb_comm_process(void)
{
    /* Process received bytes from ring buffer */
    uint8_t byte;
    while (ring_buf_get(&usb_ctx.rx_ring, &byte, 1) == 1) {
        process_rx_byte(byte);
    }
}

bool usb_comm_is_connected(void)
{
    return usb_ctx.connected;
}

nv_error_t usb_comm_send_response(uint8_t command,
                                  usb_response_status_t status,
                                  const void *payload,
                                  uint16_t length)
{
    if (!usb_ctx.uart_dev) {
        return NV_ERR_NOT_READY;
    }

    k_mutex_lock(&usb_ctx.tx_mutex, K_FOREVER);

    /* Build frame header */
    usb_frame_header_t header = {
        .sync = USB_SYNC_WORD,
        .command = command | 0x80,  /* Response bit */
        .flags = status,
        .length = length,
    };

    /* Assemble frame */
    size_t frame_len = 0;
    memcpy(usb_ctx.tx_frame, &header, sizeof(header));
    frame_len += sizeof(header);

    if (payload && length > 0) {
        memcpy(usb_ctx.tx_frame + frame_len, payload, length);
        frame_len += length;
    }

    /* Calculate CRC */
    uint16_t crc = usb_comm_crc16(usb_ctx.tx_frame, frame_len);
    usb_ctx.tx_frame[frame_len++] = crc & 0xFF;
    usb_ctx.tx_frame[frame_len++] = (crc >> 8) & 0xFF;

    /* Send frame via polling (simple and reliable) */
    for (size_t i = 0; i < frame_len; i++) {
        uart_poll_out(usb_ctx.uart_dev, usb_ctx.tx_frame[i]);
    }

    usb_ctx.tx_bytes += frame_len;

    k_mutex_unlock(&usb_ctx.tx_mutex);

    return NV_OK;
}

nv_error_t usb_comm_send_event(uint8_t event_type,
                               const void *payload,
                               uint16_t length)
{
    return usb_comm_send_response(event_type, RSP_OK, payload, length);
}

nv_error_t usb_comm_send_adc_data(const uint16_t *data, uint32_t samples)
{
    uint32_t max_samples_per_frame = (USB_MAX_PAYLOAD_SIZE - 4) / sizeof(uint16_t);
    uint32_t offset = 0;

    while (offset < samples) {
        uint32_t chunk_samples = MIN(samples - offset, max_samples_per_frame);

        uint8_t payload[USB_MAX_PAYLOAD_SIZE];
        uint32_t *pheader = (uint32_t *)payload;
        pheader[0] = offset;

        memcpy(payload + 4, &data[offset], chunk_samples * sizeof(uint16_t));

        uint8_t flags = 0;
        if (offset == 0) {
            flags |= FLAG_CHUNKED;
        }
        if (offset + chunk_samples >= samples) {
            flags |= FLAG_LAST_CHUNK;
        }

        usb_comm_send_response(CMD_ADC_GET_DATA, RSP_OK,
                              payload, 4 + chunk_samples * sizeof(uint16_t));

        offset += chunk_samples;
    }

    return NV_OK;
}

void usb_comm_set_handler(usb_command_handler_t handler)
{
    usb_ctx.command_handler = handler;
}

void usb_comm_get_stats(uint32_t *rx_bytes, uint32_t *tx_bytes, uint32_t *errors)
{
    if (rx_bytes) *rx_bytes = usb_ctx.rx_bytes;
    if (tx_bytes) *tx_bytes = usb_ctx.tx_bytes;
    if (errors) *errors = usb_ctx.frame_errors;
}
