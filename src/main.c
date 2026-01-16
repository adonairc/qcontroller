/*
 * NV-Center Controller - Main Application
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Firmware for NV-center quantum sensing experiments.
 * Target: Teensy 4.1 (NXP i.MX RT1062)
 * RTOS: Zephyr
 *
 * Features:
 * - Hardware-timed pulse sequence generation (<10ns jitter)
 * - USB CDC communication with host UI
 * - Phase-aligned ADC acquisition via ADC_ETC
 * - Standard NV sensing sequences (Rabi, Ramsey, Hahn Echo, ODMR)
 *
 * Architecture:
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                        Host PC (UI)                             │
 * └───────────────────────────┬─────────────────────────────────────┘
 *                             │ USB CDC
 *                             ▼
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                    USB Communication                            │
 * │                   (usb_comm.c)                                   │
 * └───────────────────────────┬─────────────────────────────────────┘
 *                             │
 *              ┌──────────────┼──────────────┐
 *              ▼              ▼              ▼
 * ┌───────────────────┐ ┌───────────┐ ┌───────────────┐
 * │    Sequencer      │ │   ADC     │ │   Direct I/O  │
 * │  (sequencer.c)    │ │(adc_sync) │ │   Control     │
 * └─────────┬─────────┘ └─────┬─────┘ └───────────────┘
 *           │                 │
 *           ▼                 ▼
 * ┌───────────────────────────────────────────┐
 * │              XBAR Routing                 │
 * │           (xbar_routing.c)                │
 * └───────────────────────────────────────────┘
 *           │                 │
 *           ▼                 ▼
 * ┌─────────────────┐  ┌─────────────────┐
 * │   GPT Timer     │  │    ADC_ETC      │
 * │   + DMA         │  │                 │
 * └────────┬────────┘  └────────┬────────┘
 *          │                    │
 *          ▼                    ▼
 * ┌─────────────────┐  ┌─────────────────┐
 * │     GPIO        │  │      ADC        │
 * │   (TTL Out)     │  │   (12-bit)      │
 * └─────────────────┘  └─────────────────┘
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "nv_controller.h"
#include "sequencer.h"
#include "event_table.h"
#include "usb_comm.h"
#include "adc_sync.h"
#include "xbar_routing.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Thread stack sizes and priorities
 */
#define USB_THREAD_STACK_SIZE   2048
#define USB_THREAD_PRIORITY     5

#define STATUS_LED_STACK_SIZE   512
#define STATUS_LED_PRIORITY     10

/*
 * Thread stacks
 */
K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(status_led_stack, STATUS_LED_STACK_SIZE);

static struct k_thread usb_thread_data;
static struct k_thread status_led_thread_data;

/*
 * Global state
 */
static nv_status_t controller_status = NV_STATUS_IDLE;
static nv_statistics_t controller_stats;

/* Status LED (Teensy 4.1 onboard LED = GPIO1.2) */
static const struct gpio_dt_spec status_led = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .pin = 2,
    .dt_flags = GPIO_ACTIVE_HIGH,
};

/*
 * Sequence completion callback
 */
static void sequence_complete_callback(nv_status_t status, void *user_data)
{
    ARG_UNUSED(user_data);

    controller_stats.sequences_completed++;

    LOG_INF("Sequence completed, status=%d", status);

    /* Send completion event to host */
    uint8_t event_data[4] = {
        status,
        controller_stats.sequences_completed & 0xFF,
        (controller_stats.sequences_completed >> 8) & 0xFF,
        0
    };
    usb_comm_send_event(EVENT_SEQ_COMPLETE, event_data, sizeof(event_data));
}

/*
 * ADC data ready callback
 */
static void adc_data_ready_callback(const uint16_t *data, uint32_t count,
                                    uint32_t timestamp, void *user_data)
{
    ARG_UNUSED(user_data);

    LOG_DBG("ADC data ready: %u samples at t=%u", count, timestamp);

    /* Send ADC ready event to host */
    uint8_t event_data[8] = {
        count & 0xFF,
        (count >> 8) & 0xFF,
        (count >> 16) & 0xFF,
        (count >> 24) & 0xFF,
        timestamp & 0xFF,
        (timestamp >> 8) & 0xFF,
        (timestamp >> 16) & 0xFF,
        (timestamp >> 24) & 0xFF,
    };
    usb_comm_send_event(EVENT_ADC_READY, event_data, sizeof(event_data));
}

/*
 * USB communication thread
 */
static void usb_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("USB communication thread started");

    while (1) {
        /* Process USB commands */
        usb_comm_process();

        /* Yield to other threads */
        k_msleep(1);
    }
}

/*
 * Status LED thread - indicates system state
 */
static void status_led_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (!device_is_ready(status_led.port)) {
        LOG_WRN("Status LED GPIO not ready");
        return;
    }

    gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);

    while (1) {
        seq_state_t state;
        seq_get_state(&state);

        switch (state.status) {
        case NV_STATUS_IDLE:
            /* Slow blink - idle */
            gpio_pin_toggle_dt(&status_led);
            k_msleep(1000);
            break;

        case NV_STATUS_ARMED:
            /* Medium blink - armed */
            gpio_pin_toggle_dt(&status_led);
            k_msleep(250);
            break;

        case NV_STATUS_RUNNING:
            /* Fast blink - running */
            gpio_pin_toggle_dt(&status_led);
            k_msleep(50);
            break;

        case NV_STATUS_ERROR:
            /* Double blink - error */
            gpio_pin_set_dt(&status_led, 1);
            k_msleep(100);
            gpio_pin_set_dt(&status_led, 0);
            k_msleep(100);
            gpio_pin_set_dt(&status_led, 1);
            k_msleep(100);
            gpio_pin_set_dt(&status_led, 0);
            k_msleep(700);
            break;

        default:
            k_msleep(500);
            break;
        }
    }
}

/*
 * Shell Commands for debugging
 */

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    seq_state_t state;
    seq_get_state(&state);

    shell_print(sh, "NV-Center Controller Status");
    shell_print(sh, "===========================");
    shell_print(sh, "Version: %d.%d.%d",
                NV_CONTROLLER_VERSION_MAJOR,
                NV_CONTROLLER_VERSION_MINOR,
                NV_CONTROLLER_VERSION_PATCH);
    shell_print(sh, "Status: %d", state.status);
    shell_print(sh, "Current event: %u", state.current_event_index);
    shell_print(sh, "Events remaining: %u", state.events_remaining);
    shell_print(sh, "Loops completed: %u", state.loops_completed);
    shell_print(sh, "Output state: 0x%02X", state.current_output);
    shell_print(sh, "Timer count: %u", state.current_timestamp);
    shell_print(sh, "Uptime: %u ms", k_uptime_get_32());

    return 0;
}

static int cmd_stats(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Statistics");
    shell_print(sh, "==========");
    shell_print(sh, "Sequences completed: %u", controller_stats.sequences_completed);
    shell_print(sh, "Events executed: %u", controller_stats.events_executed);
    shell_print(sh, "Trigger count: %u", controller_stats.trigger_count);
    shell_print(sh, "Overflow count: %u", controller_stats.overflow_count);

    uint32_t rx_bytes, tx_bytes, errors;
    usb_comm_get_stats(&rx_bytes, &tx_bytes, &errors);
    shell_print(sh, "USB RX: %u bytes", rx_bytes);
    shell_print(sh, "USB TX: %u bytes", tx_bytes);
    shell_print(sh, "USB errors: %u", errors);

    return 0;
}

static int cmd_set_output(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: set_output <mask>");
        return -EINVAL;
    }

    uint8_t mask = strtoul(argv[1], NULL, 0);
    seq_set_outputs(mask);

    shell_print(sh, "Outputs set to 0x%02X", mask);

    return 0;
}

static int cmd_pulse(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_print(sh, "Usage: pulse <channel> <duration_us>");
        return -EINVAL;
    }

    uint8_t channel = strtoul(argv[1], NULL, 0);
    uint32_t duration = strtoul(argv[2], NULL, 0);

    if (channel >= NV_CHANNEL_COUNT) {
        shell_print(sh, "Invalid channel: %d", channel);
        return -EINVAL;
    }

    shell_print(sh, "Generating pulse on channel %d for %u us", channel, duration);

    seq_set_channel(channel, true);
    k_busy_wait(duration);
    seq_set_channel(channel, false);

    return 0;
}

static int cmd_test_sequence(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Loading test Rabi sequence...");

    static event_table_t test_table;
    nv_error_t err = event_table_rabi(&test_table, 3, 100, 1, 10);

    if (err != NV_OK) {
        shell_print(sh, "Failed to create sequence: %d", err);
        return -EIO;
    }

    err = seq_load(&test_table);
    if (err != NV_OK) {
        shell_print(sh, "Failed to load sequence: %d", err);
        return -EIO;
    }

    shell_print(sh, "Sequence loaded: %u events", test_table.header.event_count);
    shell_print(sh, "Duration: %u ticks (%u ns)",
                test_table.header.duration_ticks,
                nv_ticks_to_ns(test_table.header.duration_ticks));

    return 0;
}

static int cmd_arm(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    nv_error_t err = seq_arm();
    if (err != NV_OK) {
        shell_print(sh, "Failed to arm: %d", err);
        return -EIO;
    }

    shell_print(sh, "Sequencer armed");
    return 0;
}

static int cmd_trigger(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    nv_error_t err = seq_trigger();
    if (err != NV_OK) {
        shell_print(sh, "Failed to trigger: %d", err);
        return -EIO;
    }

    shell_print(sh, "Sequence triggered");
    return 0;
}

static int cmd_abort(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    seq_abort();
    shell_print(sh, "Sequence aborted");
    return 0;
}

static int cmd_xbar_dump(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    xbar_dump_config();
    shell_print(sh, "XBAR configuration dumped to log");
    return 0;
}

/* Register shell commands */
SHELL_STATIC_SUBCMD_SET_CREATE(nv_cmds,
    SHELL_CMD(status, NULL, "Show system status", cmd_status),
    SHELL_CMD(stats, NULL, "Show statistics", cmd_stats),
    SHELL_CMD(set_output, NULL, "Set output mask", cmd_set_output),
    SHELL_CMD(pulse, NULL, "Generate pulse <ch> <us>", cmd_pulse),
    SHELL_CMD(test, NULL, "Load test sequence", cmd_test_sequence),
    SHELL_CMD(arm, NULL, "Arm sequencer", cmd_arm),
    SHELL_CMD(trigger, NULL, "Software trigger", cmd_trigger),
    SHELL_CMD(abort, NULL, "Abort sequence", cmd_abort),
    SHELL_CMD(xbar, NULL, "Dump XBAR config", cmd_xbar_dump),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(nv, &nv_cmds, "NV-Center Controller commands", NULL);

/*
 * Global API Implementation
 */

nv_error_t nv_controller_init(void)
{
    return NV_OK;  /* Initialization done in main() */
}

nv_error_t nv_controller_start(void)
{
    return seq_arm();
}

nv_error_t nv_controller_stop(void)
{
    return seq_abort();
}

nv_status_t nv_controller_get_status(void)
{
    seq_state_t state;
    seq_get_state(&state);
    return state.status;
}

void nv_controller_get_info(nv_system_info_t *info)
{
    if (!info) return;

    info->version_major = NV_CONTROLLER_VERSION_MAJOR;
    info->version_minor = NV_CONTROLLER_VERSION_MINOR;
    info->version_patch = NV_CONTROLLER_VERSION_PATCH;
    info->channel_count = NV_CHANNEL_COUNT;
    info->timer_freq_hz = NV_TIMER_FREQ_HZ;
    info->min_pulse_ns = NV_TIMER_TICK_NS * NV_MIN_PULSE_TICKS;
    info->max_events = CONFIG_NV_SEQUENCER_MAX_EVENTS;
    info->dma_buffer_size = CONFIG_NV_SEQUENCER_DMA_BUFFER_SIZE;
}

void nv_controller_get_statistics(nv_statistics_t *stats)
{
    if (stats) {
        memcpy(stats, &controller_stats, sizeof(nv_statistics_t));
    }
}

void nv_controller_reset_statistics(void)
{
    memset(&controller_stats, 0, sizeof(nv_statistics_t));
}

/*
 * Main Entry Point
 */

int main(void)
{
    nv_error_t err;

    printk("\n");
    printk("==========================================\n");
    printk("  NV-Center Quantum Controller v%d.%d.%d\n",
           NV_CONTROLLER_VERSION_MAJOR,
           NV_CONTROLLER_VERSION_MINOR,
           NV_CONTROLLER_VERSION_PATCH);
    printk("  Target: Teensy 4.1 (i.MX RT1062)\n");
    printk("  Timer: %u MHz, Resolution: %u ns\n",
           NV_TIMER_FREQ_HZ / 1000000,
           NV_TIMER_TICK_NS);
    printk("==========================================\n\n");

    LOG_INF("Initializing NV-Center Controller...");

    /* Initialize XBAR first (required by other subsystems) */
    err = xbar_init();
    if (err != NV_OK) {
        LOG_ERR("Failed to initialize XBAR: %d", err);
        return -1;
    }

    /* Initialize pulse sequencer */
    err = seq_init();
    if (err != NV_OK) {
        LOG_ERR("Failed to initialize sequencer: %d", err);
        return -1;
    }

    /* Set up sequencer callback */
    seq_set_completion_callback(sequence_complete_callback, NULL);

    /* Initialize ADC synchronization */
    err = adc_sync_init();
    if (err != NV_OK) {
        LOG_ERR("Failed to initialize ADC: %d", err);
        return -1;
    }

    /* Set up ADC callback */
    adc_sync_set_data_callback(adc_data_ready_callback, NULL);

    /* Apply standard NV XBAR configuration */
    err = xbar_apply_nv_config();
    if (err != NV_OK) {
        LOG_ERR("Failed to apply XBAR config: %d", err);
        return -1;
    }

    /* Initialize USB communication */
    err = usb_comm_init();
    if (err != NV_OK) {
        LOG_ERR("Failed to initialize USB: %d", err);
        return -1;
    }

    /* Configure default sequencer settings */
    seq_config_t default_config = {
        .mode = SEQ_MODE_ONESHOT,
        .trigger = NV_TRIGGER_SOFTWARE,
        .repeat_count = 1,
        .trigger_delay_ticks = 0,
        .auto_rearm = false,
        .sync_adc = true,
        .adc_trigger_offset = 0,
    };
    seq_configure(&default_config);

    LOG_INF("Initialization complete");

    /* Create USB processing thread */
    k_thread_create(&usb_thread_data, usb_thread_stack,
                    K_THREAD_STACK_SIZEOF(usb_thread_stack),
                    usb_thread_entry, NULL, NULL, NULL,
                    USB_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&usb_thread_data, "usb_comm");

    /* Create status LED thread */
    k_thread_create(&status_led_thread_data, status_led_stack,
                    K_THREAD_STACK_SIZEOF(status_led_stack),
                    status_led_thread_entry, NULL, NULL, NULL,
                    STATUS_LED_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&status_led_thread_data, "status_led");

    LOG_INF("System ready. Waiting for commands...");

    /* Main thread just sleeps - work is done in other threads */
    while (1) {
        k_msleep(1000);
    }

    return 0;
}
