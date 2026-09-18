# Zitto_MB_V1 — V0.005

S32K144 VCU firmware using a cooperative, non-blocking super-loop architecture.

## V0.005 baseline

V0.005 is branched directly from the working dev/can1-v0.004 commit.

- CAN1 is retained as the V0.004 reference implementation.
- CAN2 is an independent FlexCAN0 driver.
- Module enable/disable configuration is in src/main.c.
- CAN1 and CAN2 have independent state, baud detection, mailbox, counters and callbacks.
- UART TX uses a non-blocking software queue so CAN/sensor producers do not wait for UART wire time.
- No module waits for another module to complete detection, recovery or transmission.
- Target MCU is S32K144 only.

## Module model

Each enabled module owns its own hardware and state: UART, CAN1, CAN2, IMU, CSA, GPIO, FLM, OTA and RTT/debug.

The main loop calls only module tasks.

## Main configuration

Edit the V0.005 switches at the top of src/main.c:

APP_UART_ENABLE     1U
APP_CAN1_ENABLE     1U
APP_CAN2_ENABLE     1U
APP_IMU_ENABLE      0U
APP_CSA_ENABLE      0U
APP_GPIO_ENABLE     1U
APP_FLM_ENABLE      1U
APP_OTA_ENABLE      0U

## CAN2 hardware assumption

The CAN2 implementation follows the current project assumption: FlexCAN0, PTB0 = CAN2 RX ALT3, PTB1 = CAN2 TX ALT3, PTB5 = transceiver shutdown. SHDN LOW is normal and HIGH is shutdown.

The transceiver/pin mapping must be checked against the actual Zitto_MB_V1 schematic before hardware validation.

## Scheduler

The main loop runs at a 1 ms software cadence. UART transmission is queued and serviced in small byte batches.

## Data flow

CAN1 / CAN2 / IMU / CSA -> module-local state or callback -> UART packet queue -> LPUART0 -> ESP32

A slow UART transmission does not hold the CAN driver in a blocking transmit loop.

## Validation boundary

This branch has been reviewed statically against the project architecture and source interfaces. A real S32DS build, flash and PCAN/CAN2 bench test still require the S32DS/J-Link/S32K144 hardware environment.