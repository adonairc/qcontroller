# QController - NV-Center Quantum Sensing Controller

Firmware for precise pulse sequence generation for NV-center (Nitrogen-Vacancy) quantum sensing experiments on Teensy 4.1 (NXP i.MX RT1062) using Zephyr RTOS.

## Features

- **Hardware-timed pulse generation** with <10ns jitter using GPT timer
- **DMA-based GPIO updates** for deterministic timing without RTOS jitter
- **Phase-aligned ADC acquisition** via ADC_ETC (External Trigger Control)
- **USB CDC communication** for host UI control
- **Standard NV sensing sequences**: Rabi, Ramsey, Hahn Echo, ODMR
- **XBAR signal routing** for flexible hardware interconnection

## Hardware

### Target Platform
- **MCU**: NXP i.MX RT1062 (ARM Cortex-M7 @ 600MHz)
- **Board**: Teensy 4.1
- **Timer Resolution**: ~6.67ns (150MHz GPT clock)

### TTL Output Channels

| Channel | Pin | Function |
|---------|-----|----------|
| MW_I | 2 (GPIO4.4) | Microwave In-phase |
| MW_Q | 3 (GPIO4.5) | Microwave Quadrature |
| LASER | 4 (GPIO4.6) | Laser control (532nm) |
| MASTER | 5 (GPIO4.8) | Master pulse (lock-in reference) |
| TRIG_OUT | 6 (GPIO2.10) | Trigger output |
| TRIG_IN | 7 (GPIO2.17) | Trigger input |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Host PC (UI)                             │
└───────────────────────────┬─────────────────────────────────────┘
                            │ USB CDC
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                    USB Communication                            │
│               Binary protocol with CRC-16                       │
└───────────────────────────┬─────────────────────────────────────┘
                            │
             ┌──────────────┼──────────────┐
             ▼              ▼              ▼
┌───────────────────┐ ┌───────────┐ ┌───────────────┐
│    Sequencer      │ │   ADC     │ │   Direct I/O  │
│   GPT + DMA       │ │ ADC_ETC   │ │   Control     │
└─────────┬─────────┘ └─────┬─────┘ └───────────────┘
          │                 │
          └────────┬────────┘
                   ▼
┌───────────────────────────────────────────┐
│              XBAR Routing                 │
│      Hardware signal interconnection      │
└───────────────────────────────────────────┘
```

## Building

### Prerequisites

1. Install Zephyr SDK (>= 0.16.0)
2. Set up Zephyr workspace

### Build Commands

```bash
# Set up environment
source ~/zephyrproject/zephyr/zephyr-env.sh

# Build for Teensy 4.1 with USB CDC-ACM console
west build -p always -b teensy41 -S cdc-acm-console .

# Flash
west flash
```

The `-S cdc-acm-console` snippet enables USB CDC-ACM for console/shell and data communication.

## USB Protocol

### Frame Format

```
┌──────┬─────┬───────┬────────┬─────────┬─────┐
│ SYNC │ CMD │ FLAGS │ LENGTH │ PAYLOAD │ CRC │
│ 2B   │ 1B  │ 1B    │ 2B     │ N bytes │ 2B  │
└──────┴─────┴───────┴────────┴─────────┴─────┘

SYNC = 0x4E56 ("NV")
CRC = CRC-16-CCITT
```

### Command Reference

| Code | Command | Description |
|------|---------|-------------|
| 0x00 | NOP | Ping/heartbeat |
| 0x01 | GET_INFO | Get system information |
| 0x02 | GET_STATUS | Get current status |
| 0x10 | SEQ_LOAD | Load pulse sequence |
| 0x12 | SEQ_ARM | Arm sequencer |
| 0x14 | SEQ_TRIGGER | Software trigger |
| 0x15 | SEQ_ABORT | Abort sequence |
| 0x20 | IO_SET_ALL | Set all outputs |
| 0x30 | ADC_CONFIG | Configure ADC |
| 0x40 | PRESET_RABI | Generate Rabi sequence |
| 0x41 | PRESET_RAMSEY | Generate Ramsey sequence |
| 0x42 | PRESET_ECHO | Generate Hahn echo sequence |

## Event Table Format

Pulse sequences are stored as time-ordered events:

```c
typedef struct {
    uint32_t timestamp_ticks;  // Time in timer ticks (~6.67ns each)
    uint8_t mask;              // Channel output states (bitmask)
    uint8_t flags;             // ADC trigger, loop markers, etc.
    uint16_t reserved;
} pulse_event_t;  // 8 bytes per event
```

## Standard NV Sequences

### Rabi Oscillation
```
Laser ON (init) → MW pulse (variable τ) → Laser ON (readout)
```

### Ramsey Interferometry
```
Laser ON → π/2 pulse → τ delay → π/2 pulse → Laser ON (readout)
```

### Hahn Echo
```
Laser ON → π/2 → τ → π → τ → π/2 → Laser ON (readout)
```

### ODMR (CW)
```
Continuous Laser + swept MW frequency
```

## Shell Commands

Connect via USB serial terminal (115200 baud):

```
nv status      - Show system status
nv stats       - Show statistics
nv set_output  - Set output mask
nv pulse       - Generate single pulse
nv test        - Load test sequence
nv arm         - Arm sequencer
nv trigger     - Software trigger
nv abort       - Abort sequence
nv xbar        - Dump XBAR configuration
```

## Timing Specifications

| Parameter | Value |
|-----------|-------|
| Timer frequency | 150 MHz |
| Timer resolution | ~6.67 ns |
| Minimum pulse width | ~13 ns (2 ticks) |
| Maximum sequence duration | ~28.6 seconds (32-bit timer) |
| Maximum events | 4096 (configurable) |
| DMA buffer size | 256 events |

## License

SPDX-License-Identifier: Apache-2.0
