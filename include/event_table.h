/*
 * NV-Center Controller - Event Table
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Event table structure for storing pulse sequences.
 * Events are stored as time-ordered (timestamp, channel_mask) pairs.
 *
 * Protocol Format (binary, little-endian):
 *
 * Event Entry (8 bytes):
 * ┌────────────────────────────────────────────────────────────┐
 * │ timestamp (4 bytes) │ channel_mask (1 byte) │ flags (1 byte) │ reserved (2 bytes) │
 * └────────────────────────────────────────────────────────────┘
 *
 * Timestamps are in timer ticks (6.67ns resolution @ 150MHz).
 * Events must be sorted by timestamp in ascending order.
 */

#ifndef EVENT_TABLE_H
#define EVENT_TABLE_H

#include "nv_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Maximum events in a sequence
 */
#ifndef CONFIG_NV_SEQUENCER_MAX_EVENTS
#define CONFIG_NV_SEQUENCER_MAX_EVENTS 4096
#endif

/*
 * Event Flags
 */
typedef enum {
    EVENT_FLAG_NONE         = 0x00,
    EVENT_FLAG_ADC_TRIGGER  = 0x01,  /* Trigger ADC acquisition at this event */
    EVENT_FLAG_LOOP_START   = 0x02,  /* Mark start of loop section */
    EVENT_FLAG_LOOP_END     = 0x04,  /* Mark end of loop section */
    EVENT_FLAG_SYNC_MARKER  = 0x08,  /* Synchronization marker */
    EVENT_FLAG_TRIGGER_OUT  = 0x10,  /* Assert trigger output */
} event_flags_t;

/*
 * Single Pulse Event
 *
 * Represents a state change at a specific timestamp.
 * The channel_mask defines which channels are HIGH at this time.
 */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t timestamp_ticks;   /* Time in timer ticks from sequence start */
    nv_channel_mask_t mask;     /* Channel output states (1=HIGH, 0=LOW) */
    uint8_t flags;              /* Event flags */
    uint16_t reserved;          /* Reserved for future use / alignment */
} pulse_event_t;

_Static_assert(sizeof(pulse_event_t) == 8, "Event size must be 8 bytes for DMA alignment");

/*
 * Event Table Header
 *
 * Metadata for a pulse sequence.
 */
typedef struct {
    uint32_t magic;             /* Magic number for validation (0x4E565351 = "NVSQ") */
    uint16_t version;           /* Table format version */
    uint16_t flags;             /* Table-level flags */
    uint32_t event_count;       /* Number of events in table */
    uint32_t duration_ticks;    /* Total sequence duration */
    uint32_t loop_start_idx;    /* Index of loop start (if looping) */
    uint32_t loop_end_idx;      /* Index of loop end (if looping) */
    uint32_t loop_count;        /* Number of loop iterations (0 = infinite) */
    uint32_t checksum;          /* CRC32 of event data */
} event_table_header_t;

#define EVENT_TABLE_MAGIC 0x4E565351  /* "NVSQ" */
#define EVENT_TABLE_VERSION 0x0100    /* Version 1.0 */

/*
 * Event Table Structure
 *
 * Complete sequence container with header and events.
 */
typedef struct {
    event_table_header_t header;
    pulse_event_t events[CONFIG_NV_SEQUENCER_MAX_EVENTS];
} event_table_t;

/*
 * Pulse Builder
 *
 * Helper structure for constructing pulse sequences programmatically.
 */
typedef struct {
    event_table_t *table;
    uint32_t current_time;      /* Current time position in ticks */
    nv_channel_mask_t current_state;  /* Current output state */
    bool error;                 /* Error flag */
} pulse_builder_t;

/*
 * Event Table API Functions
 */

/**
 * @brief Initialize an empty event table
 *
 * @param table Pointer to event table to initialize
 */
void event_table_init(event_table_t *table);

/**
 * @brief Clear all events from table
 *
 * @param table Pointer to event table
 */
void event_table_clear(event_table_t *table);

/**
 * @brief Add a pulse event to the table
 *
 * Events are automatically sorted by timestamp.
 *
 * @param table Pointer to event table
 * @param timestamp Time in ticks
 * @param mask Channel output states
 * @param flags Event flags
 * @return NV_OK on success, NV_ERR_OVERFLOW if table full
 */
nv_error_t event_table_add(event_table_t *table,
                           uint32_t timestamp,
                           nv_channel_mask_t mask,
                           uint8_t flags);

/**
 * @brief Add pulse event using nanosecond timestamp
 *
 * @param table Pointer to event table
 * @param timestamp_ns Time in nanoseconds
 * @param mask Channel output states
 * @param flags Event flags
 * @return NV_OK on success
 */
nv_error_t event_table_add_ns(event_table_t *table,
                              uint32_t timestamp_ns,
                              nv_channel_mask_t mask,
                              uint8_t flags);

/**
 * @brief Insert event at specific index
 *
 * @param table Pointer to event table
 * @param index Index to insert at
 * @param event Event to insert
 * @return NV_OK on success
 */
nv_error_t event_table_insert(event_table_t *table,
                              uint32_t index,
                              const pulse_event_t *event);

/**
 * @brief Remove event at index
 *
 * @param table Pointer to event table
 * @param index Index to remove
 * @return NV_OK on success
 */
nv_error_t event_table_remove(event_table_t *table, uint32_t index);

/**
 * @brief Get event at index
 *
 * @param table Pointer to event table
 * @param index Event index
 * @return Pointer to event, or NULL if index invalid
 */
const pulse_event_t *event_table_get(const event_table_t *table, uint32_t index);

/**
 * @brief Get number of events in table
 *
 * @param table Pointer to event table
 * @return Event count
 */
uint32_t event_table_count(const event_table_t *table);

/**
 * @brief Sort events by timestamp
 *
 * Call after adding events out of order.
 *
 * @param table Pointer to event table
 */
void event_table_sort(event_table_t *table);

/**
 * @brief Validate event table integrity
 *
 * Checks timestamp ordering, flag consistency, etc.
 *
 * @param table Pointer to event table
 * @return NV_OK if valid, error code otherwise
 */
nv_error_t event_table_validate(const event_table_t *table);

/**
 * @brief Calculate CRC32 checksum of event data
 *
 * @param table Pointer to event table
 * @return CRC32 checksum
 */
uint32_t event_table_checksum(const event_table_t *table);

/**
 * @brief Set loop points in sequence
 *
 * @param table Pointer to event table
 * @param start_idx Start event index
 * @param end_idx End event index
 * @param count Loop iterations (0 = infinite)
 * @return NV_OK on success
 */
nv_error_t event_table_set_loop(event_table_t *table,
                                uint32_t start_idx,
                                uint32_t end_idx,
                                uint32_t count);

/**
 * @brief Serialize event table to binary buffer
 *
 * @param table Source event table
 * @param buffer Destination buffer
 * @param buffer_size Buffer size
 * @param bytes_written Actual bytes written (output)
 * @return NV_OK on success
 */
nv_error_t event_table_serialize(const event_table_t *table,
                                 uint8_t *buffer,
                                 size_t buffer_size,
                                 size_t *bytes_written);

/**
 * @brief Deserialize event table from binary buffer
 *
 * @param table Destination event table
 * @param buffer Source buffer
 * @param buffer_size Buffer size
 * @return NV_OK on success, NV_ERR_PROTOCOL if invalid
 */
nv_error_t event_table_deserialize(event_table_t *table,
                                   const uint8_t *buffer,
                                   size_t buffer_size);

/*
 * Pulse Builder API
 *
 * Fluent interface for constructing sequences.
 */

/**
 * @brief Initialize pulse builder
 *
 * @param builder Builder context
 * @param table Target event table
 */
void pulse_builder_init(pulse_builder_t *builder, event_table_t *table);

/**
 * @brief Reset builder to time zero
 *
 * @param builder Builder context
 */
void pulse_builder_reset(pulse_builder_t *builder);

/**
 * @brief Set channel high at current time
 *
 * @param builder Builder context
 * @param channel Channel to set
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_set(pulse_builder_t *builder, nv_channel_t channel);

/**
 * @brief Set channel low at current time
 *
 * @param builder Builder context
 * @param channel Channel to clear
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_clear(pulse_builder_t *builder, nv_channel_t channel);

/**
 * @brief Toggle channel at current time
 *
 * @param builder Builder context
 * @param channel Channel to toggle
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_toggle(pulse_builder_t *builder, nv_channel_t channel);

/**
 * @brief Set multiple channels from mask
 *
 * @param builder Builder context
 * @param mask Channel mask
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_set_mask(pulse_builder_t *builder, nv_channel_mask_t mask);

/**
 * @brief Advance time by specified ticks
 *
 * @param builder Builder context
 * @param ticks Time to advance
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_delay_ticks(pulse_builder_t *builder, uint32_t ticks);

/**
 * @brief Advance time by nanoseconds
 *
 * @param builder Builder context
 * @param ns Nanoseconds to advance
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_delay_ns(pulse_builder_t *builder, uint32_t ns);

/**
 * @brief Advance time by microseconds
 *
 * @param builder Builder context
 * @param us Microseconds to advance
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_delay_us(pulse_builder_t *builder, uint32_t us);

/**
 * @brief Generate a pulse on channel
 *
 * Sets channel high, waits duration, sets low.
 *
 * @param builder Builder context
 * @param channel Channel for pulse
 * @param duration_ns Pulse duration in nanoseconds
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_pulse_ns(pulse_builder_t *builder,
                                        nv_channel_t channel,
                                        uint32_t duration_ns);

/**
 * @brief Add ADC trigger marker at current time
 *
 * @param builder Builder context
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_adc_trigger(pulse_builder_t *builder);

/**
 * @brief Mark loop start at current position
 *
 * @param builder Builder context
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_loop_start(pulse_builder_t *builder);

/**
 * @brief Mark loop end at current position
 *
 * @param builder Builder context
 * @param count Loop iterations
 * @return Builder pointer for chaining
 */
pulse_builder_t *pulse_builder_loop_end(pulse_builder_t *builder, uint32_t count);

/**
 * @brief Finalize and validate the built sequence
 *
 * @param builder Builder context
 * @return NV_OK if successful, error otherwise
 */
nv_error_t pulse_builder_finalize(pulse_builder_t *builder);

/**
 * @brief Check if builder encountered error
 *
 * @param builder Builder context
 * @return true if error occurred
 */
bool pulse_builder_has_error(const pulse_builder_t *builder);

/*
 * Standard NV-Center Sequence Templates
 */

/**
 * @brief Create Rabi oscillation measurement sequence
 *
 * Laser init -> MW pulse (variable) -> Laser readout
 *
 * @param table Target event table
 * @param init_us Initialization laser duration (us)
 * @param mw_ns Microwave pulse duration (ns)
 * @param readout_us Readout laser duration (us)
 * @param repeat Number of sequence repeats
 * @return NV_OK on success
 */
nv_error_t event_table_rabi(event_table_t *table,
                            uint32_t init_us,
                            uint32_t mw_ns,
                            uint32_t readout_us,
                            uint32_t repeat);

/**
 * @brief Create Ramsey measurement sequence
 *
 * Laser init -> π/2 -> tau delay -> π/2 -> Laser readout
 *
 * @param table Target event table
 * @param init_us Initialization laser duration (us)
 * @param pi2_ns Pi/2 pulse duration (ns)
 * @param tau_ns Free precession time (ns)
 * @param readout_us Readout laser duration (us)
 * @return NV_OK on success
 */
nv_error_t event_table_ramsey(event_table_t *table,
                              uint32_t init_us,
                              uint32_t pi2_ns,
                              uint32_t tau_ns,
                              uint32_t readout_us);

/**
 * @brief Create Hahn echo sequence
 *
 * Laser init -> π/2 -> tau -> π -> tau -> π/2 -> Laser readout
 *
 * @param table Target event table
 * @param init_us Initialization laser duration (us)
 * @param pi2_ns Pi/2 pulse duration (ns)
 * @param pi_ns Pi pulse duration (ns)
 * @param tau_ns Echo delay time (ns)
 * @param readout_us Readout laser duration (us)
 * @return NV_OK on success
 */
nv_error_t event_table_hahn_echo(event_table_t *table,
                                 uint32_t init_us,
                                 uint32_t pi2_ns,
                                 uint32_t pi_ns,
                                 uint32_t tau_ns,
                                 uint32_t readout_us);

/**
 * @brief Create ODMR (CW) measurement sequence
 *
 * Continuous laser + swept MW frequency
 *
 * @param table Target event table
 * @param dwell_us Dwell time per frequency point (us)
 * @param points Number of frequency points
 * @return NV_OK on success
 */
nv_error_t event_table_odmr_cw(event_table_t *table,
                               uint32_t dwell_us,
                               uint32_t points);

#ifdef __cplusplus
}
#endif

#endif /* EVENT_TABLE_H */
