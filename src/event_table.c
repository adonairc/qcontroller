/*
 * NV-Center Controller - Event Table Implementation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pulse sequence event table management and standard NV sensing sequences.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "event_table.h"

LOG_MODULE_REGISTER(event_table, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * CRC32 lookup table (IEEE 802.3 polynomial)
 */
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd706b3,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};

static uint32_t crc32_compute(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }

    return crc ^ 0xFFFFFFFF;
}

/*
 * Quicksort for event sorting by timestamp
 */
static void event_swap(pulse_event_t *a, pulse_event_t *b)
{
    pulse_event_t temp = *a;
    *a = *b;
    *b = temp;
}

static int event_partition(pulse_event_t *events, int low, int high)
{
    uint32_t pivot = events[high].timestamp_ticks;
    int i = low - 1;

    for (int j = low; j < high; j++) {
        if (events[j].timestamp_ticks <= pivot) {
            i++;
            event_swap(&events[i], &events[j]);
        }
    }
    event_swap(&events[i + 1], &events[high]);
    return i + 1;
}

static void event_quicksort(pulse_event_t *events, int low, int high)
{
    if (low < high) {
        int pi = event_partition(events, low, high);
        event_quicksort(events, low, pi - 1);
        event_quicksort(events, pi + 1, high);
    }
}

/*
 * Event Table API Implementation
 */

void event_table_init(event_table_t *table)
{
    if (!table) {
        return;
    }

    memset(table, 0, sizeof(event_table_t));
    table->header.magic = EVENT_TABLE_MAGIC;
    table->header.version = EVENT_TABLE_VERSION;
}

void event_table_clear(event_table_t *table)
{
    if (!table) {
        return;
    }

    table->header.event_count = 0;
    table->header.duration_ticks = 0;
    table->header.loop_start_idx = 0;
    table->header.loop_end_idx = 0;
    table->header.loop_count = 0;
    table->header.checksum = 0;
}

nv_error_t event_table_add(event_table_t *table,
                           uint32_t timestamp,
                           nv_channel_mask_t mask,
                           uint8_t flags)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    if (table->header.event_count >= CONFIG_NV_SEQUENCER_MAX_EVENTS) {
        return NV_ERR_OVERFLOW;
    }

    uint32_t idx = table->header.event_count;

    table->events[idx].timestamp_ticks = timestamp;
    table->events[idx].mask = mask;
    table->events[idx].flags = flags;
    table->events[idx].reserved = 0;

    table->header.event_count++;

    /* Update duration */
    if (timestamp > table->header.duration_ticks) {
        table->header.duration_ticks = timestamp;
    }

    return NV_OK;
}

nv_error_t event_table_add_ns(event_table_t *table,
                              uint32_t timestamp_ns,
                              nv_channel_mask_t mask,
                              uint8_t flags)
{
    return event_table_add(table, nv_ns_to_ticks(timestamp_ns), mask, flags);
}

nv_error_t event_table_insert(event_table_t *table,
                              uint32_t index,
                              const pulse_event_t *event)
{
    if (!table || !event) {
        return NV_ERR_INVALID_ARG;
    }

    if (table->header.event_count >= CONFIG_NV_SEQUENCER_MAX_EVENTS) {
        return NV_ERR_OVERFLOW;
    }

    if (index > table->header.event_count) {
        index = table->header.event_count;
    }

    /* Shift events to make room */
    for (uint32_t i = table->header.event_count; i > index; i--) {
        table->events[i] = table->events[i - 1];
    }

    table->events[index] = *event;
    table->header.event_count++;

    return NV_OK;
}

nv_error_t event_table_remove(event_table_t *table, uint32_t index)
{
    if (!table || index >= table->header.event_count) {
        return NV_ERR_INVALID_ARG;
    }

    /* Shift events down */
    for (uint32_t i = index; i < table->header.event_count - 1; i++) {
        table->events[i] = table->events[i + 1];
    }

    table->header.event_count--;

    return NV_OK;
}

const pulse_event_t *event_table_get(const event_table_t *table, uint32_t index)
{
    if (!table || index >= table->header.event_count) {
        return NULL;
    }

    return &table->events[index];
}

uint32_t event_table_count(const event_table_t *table)
{
    return table ? table->header.event_count : 0;
}

void event_table_sort(event_table_t *table)
{
    if (!table || table->header.event_count < 2) {
        return;
    }

    event_quicksort(table->events, 0, table->header.event_count - 1);

    /* Recalculate duration */
    table->header.duration_ticks =
        table->events[table->header.event_count - 1].timestamp_ticks;
}

nv_error_t event_table_validate(const event_table_t *table)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    if (table->header.magic != EVENT_TABLE_MAGIC) {
        LOG_ERR("Invalid magic: 0x%08X", table->header.magic);
        return NV_ERR_PROTOCOL;
    }

    if (table->header.event_count == 0) {
        LOG_ERR("Empty event table");
        return NV_ERR_SEQUENCE;
    }

    if (table->header.event_count > CONFIG_NV_SEQUENCER_MAX_EVENTS) {
        LOG_ERR("Too many events: %u", table->header.event_count);
        return NV_ERR_OVERFLOW;
    }

    /* Check timestamp ordering */
    for (uint32_t i = 1; i < table->header.event_count; i++) {
        if (table->events[i].timestamp_ticks < table->events[i - 1].timestamp_ticks) {
            LOG_ERR("Events not sorted at index %u", i);
            return NV_ERR_SEQUENCE;
        }
    }

    /* Check loop indices */
    if (table->header.loop_end_idx != 0) {
        if (table->header.loop_start_idx >= table->header.event_count ||
            table->header.loop_end_idx >= table->header.event_count ||
            table->header.loop_start_idx > table->header.loop_end_idx) {
            LOG_ERR("Invalid loop indices");
            return NV_ERR_SEQUENCE;
        }
    }

    return NV_OK;
}

uint32_t event_table_checksum(const event_table_t *table)
{
    if (!table || table->header.event_count == 0) {
        return 0;
    }

    return crc32_compute((const uint8_t *)table->events,
                        table->header.event_count * sizeof(pulse_event_t));
}

nv_error_t event_table_set_loop(event_table_t *table,
                                uint32_t start_idx,
                                uint32_t end_idx,
                                uint32_t count)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    if (start_idx >= table->header.event_count ||
        end_idx >= table->header.event_count ||
        start_idx > end_idx) {
        return NV_ERR_INVALID_ARG;
    }

    table->header.loop_start_idx = start_idx;
    table->header.loop_end_idx = end_idx;
    table->header.loop_count = count;

    /* Set flags on boundary events */
    table->events[start_idx].flags |= EVENT_FLAG_LOOP_START;
    table->events[end_idx].flags |= EVENT_FLAG_LOOP_END;

    return NV_OK;
}

nv_error_t event_table_serialize(const event_table_t *table,
                                 uint8_t *buffer,
                                 size_t buffer_size,
                                 size_t *bytes_written)
{
    if (!table || !buffer || !bytes_written) {
        return NV_ERR_INVALID_ARG;
    }

    size_t header_size = sizeof(event_table_header_t);
    size_t events_size = table->header.event_count * sizeof(pulse_event_t);
    size_t total_size = header_size + events_size;

    if (buffer_size < total_size) {
        return NV_ERR_OVERFLOW;
    }

    /* Copy header */
    memcpy(buffer, &table->header, header_size);

    /* Copy events */
    memcpy(buffer + header_size, table->events, events_size);

    /* Update checksum in buffer */
    event_table_header_t *buf_header = (event_table_header_t *)buffer;
    buf_header->checksum = crc32_compute(buffer + header_size, events_size);

    *bytes_written = total_size;

    return NV_OK;
}

nv_error_t event_table_deserialize(event_table_t *table,
                                   const uint8_t *buffer,
                                   size_t buffer_size)
{
    if (!table || !buffer) {
        return NV_ERR_INVALID_ARG;
    }

    if (buffer_size < sizeof(event_table_header_t)) {
        return NV_ERR_PROTOCOL;
    }

    /* Copy header */
    memcpy(&table->header, buffer, sizeof(event_table_header_t));

    /* Validate header */
    if (table->header.magic != EVENT_TABLE_MAGIC) {
        return NV_ERR_PROTOCOL;
    }

    if (table->header.event_count > CONFIG_NV_SEQUENCER_MAX_EVENTS) {
        return NV_ERR_OVERFLOW;
    }

    size_t events_size = table->header.event_count * sizeof(pulse_event_t);
    size_t total_size = sizeof(event_table_header_t) + events_size;

    if (buffer_size < total_size) {
        return NV_ERR_PROTOCOL;
    }

    /* Copy events */
    memcpy(table->events, buffer + sizeof(event_table_header_t), events_size);

    /* Verify checksum */
    uint32_t calc_checksum = crc32_compute(
        buffer + sizeof(event_table_header_t), events_size);

    if (calc_checksum != table->header.checksum) {
        LOG_ERR("Checksum mismatch: expected 0x%08X, got 0x%08X",
                table->header.checksum, calc_checksum);
        return NV_ERR_PROTOCOL;
    }

    return NV_OK;
}

/*
 * Pulse Builder Implementation
 */

void pulse_builder_init(pulse_builder_t *builder, event_table_t *table)
{
    if (!builder || !table) {
        return;
    }

    builder->table = table;
    builder->current_time = 0;
    builder->current_state = 0;
    builder->error = false;

    event_table_init(table);
}

void pulse_builder_reset(pulse_builder_t *builder)
{
    if (!builder) {
        return;
    }

    builder->current_time = 0;
    builder->current_state = 0;
    builder->error = false;

    if (builder->table) {
        event_table_clear(builder->table);
    }
}

pulse_builder_t *pulse_builder_set(pulse_builder_t *builder, nv_channel_t channel)
{
    if (!builder || builder->error || channel >= NV_CHANNEL_COUNT) {
        if (builder) builder->error = true;
        return builder;
    }

    nv_channel_mask_t new_state = builder->current_state | (1 << channel);

    if (new_state != builder->current_state) {
        builder->current_state = new_state;
        nv_error_t err = event_table_add(builder->table, builder->current_time,
                                         new_state, EVENT_FLAG_NONE);
        if (err != NV_OK) {
            builder->error = true;
        }
    }

    return builder;
}

pulse_builder_t *pulse_builder_clear(pulse_builder_t *builder, nv_channel_t channel)
{
    if (!builder || builder->error || channel >= NV_CHANNEL_COUNT) {
        if (builder) builder->error = true;
        return builder;
    }

    nv_channel_mask_t new_state = builder->current_state & ~(1 << channel);

    if (new_state != builder->current_state) {
        builder->current_state = new_state;
        nv_error_t err = event_table_add(builder->table, builder->current_time,
                                         new_state, EVENT_FLAG_NONE);
        if (err != NV_OK) {
            builder->error = true;
        }
    }

    return builder;
}

pulse_builder_t *pulse_builder_toggle(pulse_builder_t *builder, nv_channel_t channel)
{
    if (!builder || builder->error || channel >= NV_CHANNEL_COUNT) {
        if (builder) builder->error = true;
        return builder;
    }

    builder->current_state ^= (1 << channel);

    nv_error_t err = event_table_add(builder->table, builder->current_time,
                                     builder->current_state, EVENT_FLAG_NONE);
    if (err != NV_OK) {
        builder->error = true;
    }

    return builder;
}

pulse_builder_t *pulse_builder_set_mask(pulse_builder_t *builder, nv_channel_mask_t mask)
{
    if (!builder || builder->error) {
        return builder;
    }

    if (mask != builder->current_state) {
        builder->current_state = mask;
        nv_error_t err = event_table_add(builder->table, builder->current_time,
                                         mask, EVENT_FLAG_NONE);
        if (err != NV_OK) {
            builder->error = true;
        }
    }

    return builder;
}

pulse_builder_t *pulse_builder_delay_ticks(pulse_builder_t *builder, uint32_t ticks)
{
    if (!builder || builder->error) {
        return builder;
    }

    builder->current_time += ticks;

    return builder;
}

pulse_builder_t *pulse_builder_delay_ns(pulse_builder_t *builder, uint32_t ns)
{
    return pulse_builder_delay_ticks(builder, nv_ns_to_ticks(ns));
}

pulse_builder_t *pulse_builder_delay_us(pulse_builder_t *builder, uint32_t us)
{
    return pulse_builder_delay_ticks(builder, nv_us_to_ticks(us));
}

pulse_builder_t *pulse_builder_pulse_ns(pulse_builder_t *builder,
                                        nv_channel_t channel,
                                        uint32_t duration_ns)
{
    if (!builder || builder->error || channel >= NV_CHANNEL_COUNT) {
        if (builder) builder->error = true;
        return builder;
    }

    /* Set channel high */
    pulse_builder_set(builder, channel);

    /* Wait duration */
    pulse_builder_delay_ns(builder, duration_ns);

    /* Set channel low */
    pulse_builder_clear(builder, channel);

    return builder;
}

pulse_builder_t *pulse_builder_adc_trigger(pulse_builder_t *builder)
{
    if (!builder || builder->error) {
        return builder;
    }

    /* Add event with ADC trigger flag */
    nv_error_t err = event_table_add(builder->table, builder->current_time,
                                     builder->current_state, EVENT_FLAG_ADC_TRIGGER);
    if (err != NV_OK) {
        builder->error = true;
    }

    return builder;
}

pulse_builder_t *pulse_builder_loop_start(pulse_builder_t *builder)
{
    if (!builder || builder->error || !builder->table) {
        return builder;
    }

    builder->table->header.loop_start_idx = builder->table->header.event_count;

    return builder;
}

pulse_builder_t *pulse_builder_loop_end(pulse_builder_t *builder, uint32_t count)
{
    if (!builder || builder->error || !builder->table) {
        return builder;
    }

    builder->table->header.loop_end_idx = builder->table->header.event_count;
    builder->table->header.loop_count = count;

    return builder;
}

nv_error_t pulse_builder_finalize(pulse_builder_t *builder)
{
    if (!builder || !builder->table) {
        return NV_ERR_INVALID_ARG;
    }

    if (builder->error) {
        return NV_ERR_OVERFLOW;
    }

    /* Ensure we end with all outputs low */
    if (builder->current_state != 0) {
        event_table_add(builder->table, builder->current_time, 0, EVENT_FLAG_NONE);
    }

    /* Sort events (should already be sorted, but ensure) */
    event_table_sort(builder->table);

    /* Update checksum */
    builder->table->header.checksum = event_table_checksum(builder->table);

    return event_table_validate(builder->table);
}

bool pulse_builder_has_error(const pulse_builder_t *builder)
{
    return builder ? builder->error : true;
}

/*
 * Standard NV-Center Sequence Templates
 */

nv_error_t event_table_rabi(event_table_t *table,
                            uint32_t init_us,
                            uint32_t mw_ns,
                            uint32_t readout_us,
                            uint32_t repeat)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    pulse_builder_t builder;
    pulse_builder_init(&builder, table);

    for (uint32_t i = 0; i < repeat; i++) {
        /* Initialization: Laser ON */
        pulse_builder_set(&builder, NV_CHANNEL_LASER);
        pulse_builder_set(&builder, NV_CHANNEL_MASTER);  /* Lock-in ref */
        pulse_builder_delay_us(&builder, init_us);
        pulse_builder_clear(&builder, NV_CHANNEL_LASER);

        /* Wait for thermalization */
        pulse_builder_delay_us(&builder, 1);

        /* Microwave pulse */
        pulse_builder_set(&builder, NV_CHANNEL_MW_I);
        pulse_builder_delay_ns(&builder, mw_ns);
        pulse_builder_clear(&builder, NV_CHANNEL_MW_I);

        /* Readout: Laser ON with ADC trigger */
        pulse_builder_adc_trigger(&builder);
        pulse_builder_set(&builder, NV_CHANNEL_LASER);
        pulse_builder_delay_us(&builder, readout_us);
        pulse_builder_clear(&builder, NV_CHANNEL_LASER);
        pulse_builder_clear(&builder, NV_CHANNEL_MASTER);

        /* Inter-sequence delay */
        pulse_builder_delay_us(&builder, 10);
    }

    return pulse_builder_finalize(&builder);
}

nv_error_t event_table_ramsey(event_table_t *table,
                              uint32_t init_us,
                              uint32_t pi2_ns,
                              uint32_t tau_ns,
                              uint32_t readout_us)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    pulse_builder_t builder;
    pulse_builder_init(&builder, table);

    /* Initialization: Laser ON */
    pulse_builder_set(&builder, NV_CHANNEL_LASER);
    pulse_builder_set(&builder, NV_CHANNEL_MASTER);
    pulse_builder_delay_us(&builder, init_us);
    pulse_builder_clear(&builder, NV_CHANNEL_LASER);

    /* Wait for thermalization */
    pulse_builder_delay_us(&builder, 1);

    /* First π/2 pulse */
    pulse_builder_set(&builder, NV_CHANNEL_MW_I);
    pulse_builder_delay_ns(&builder, pi2_ns);
    pulse_builder_clear(&builder, NV_CHANNEL_MW_I);

    /* Free precession time tau */
    pulse_builder_delay_ns(&builder, tau_ns);

    /* Second π/2 pulse */
    pulse_builder_set(&builder, NV_CHANNEL_MW_I);
    pulse_builder_delay_ns(&builder, pi2_ns);
    pulse_builder_clear(&builder, NV_CHANNEL_MW_I);

    /* Readout: Laser ON with ADC trigger */
    pulse_builder_adc_trigger(&builder);
    pulse_builder_set(&builder, NV_CHANNEL_LASER);
    pulse_builder_delay_us(&builder, readout_us);
    pulse_builder_clear(&builder, NV_CHANNEL_LASER);
    pulse_builder_clear(&builder, NV_CHANNEL_MASTER);

    return pulse_builder_finalize(&builder);
}

nv_error_t event_table_hahn_echo(event_table_t *table,
                                 uint32_t init_us,
                                 uint32_t pi2_ns,
                                 uint32_t pi_ns,
                                 uint32_t tau_ns,
                                 uint32_t readout_us)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    pulse_builder_t builder;
    pulse_builder_init(&builder, table);

    /* Initialization: Laser ON */
    pulse_builder_set(&builder, NV_CHANNEL_LASER);
    pulse_builder_set(&builder, NV_CHANNEL_MASTER);
    pulse_builder_delay_us(&builder, init_us);
    pulse_builder_clear(&builder, NV_CHANNEL_LASER);

    /* Wait for thermalization */
    pulse_builder_delay_us(&builder, 1);

    /* First π/2 pulse */
    pulse_builder_set(&builder, NV_CHANNEL_MW_I);
    pulse_builder_delay_ns(&builder, pi2_ns);
    pulse_builder_clear(&builder, NV_CHANNEL_MW_I);

    /* First tau delay */
    pulse_builder_delay_ns(&builder, tau_ns);

    /* π refocusing pulse */
    pulse_builder_set(&builder, NV_CHANNEL_MW_I);
    pulse_builder_delay_ns(&builder, pi_ns);
    pulse_builder_clear(&builder, NV_CHANNEL_MW_I);

    /* Second tau delay */
    pulse_builder_delay_ns(&builder, tau_ns);

    /* Final π/2 pulse */
    pulse_builder_set(&builder, NV_CHANNEL_MW_I);
    pulse_builder_delay_ns(&builder, pi2_ns);
    pulse_builder_clear(&builder, NV_CHANNEL_MW_I);

    /* Readout: Laser ON with ADC trigger */
    pulse_builder_adc_trigger(&builder);
    pulse_builder_set(&builder, NV_CHANNEL_LASER);
    pulse_builder_delay_us(&builder, readout_us);
    pulse_builder_clear(&builder, NV_CHANNEL_LASER);
    pulse_builder_clear(&builder, NV_CHANNEL_MASTER);

    return pulse_builder_finalize(&builder);
}

nv_error_t event_table_odmr_cw(event_table_t *table,
                               uint32_t dwell_us,
                               uint32_t points)
{
    if (!table) {
        return NV_ERR_INVALID_ARG;
    }

    pulse_builder_t builder;
    pulse_builder_init(&builder, table);

    /* Continuous laser during entire sequence */
    pulse_builder_set(&builder, NV_CHANNEL_LASER);

    for (uint32_t i = 0; i < points; i++) {
        /* Set master pulse for this measurement point */
        pulse_builder_set(&builder, NV_CHANNEL_MASTER);

        /* MW ON for dwell time */
        pulse_builder_set(&builder, NV_CHANNEL_MW_I);
        pulse_builder_adc_trigger(&builder);
        pulse_builder_delay_us(&builder, dwell_us);
        pulse_builder_clear(&builder, NV_CHANNEL_MW_I);
        pulse_builder_clear(&builder, NV_CHANNEL_MASTER);

        /* Small gap between points */
        pulse_builder_delay_us(&builder, 1);
    }

    pulse_builder_clear(&builder, NV_CHANNEL_LASER);

    return pulse_builder_finalize(&builder);
}
