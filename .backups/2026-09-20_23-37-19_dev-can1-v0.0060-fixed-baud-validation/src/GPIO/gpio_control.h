/*
 * gpio_control.h
 *
 * Zitto_MB_V1 / S32K144
 *
 * Remote GPIO control service
 *
 * Architecture:
 *
 * ESP32
 *   ↓ UART
 * uart_pkt
 *   ↓
 * APP command dispatcher
 *   ↓
 * Gpio_ControlProcessCommand()
 *   ↓
 * physical GPIO
 *   ↓
 * readback
 *   ↓
 * UART status/ACK
 *
 * Only the GPIOs explicitly listed in this module are remotely
 * controllable.
 */

#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "S32K144.h"
#include <stdint.h>
#include "../UART/uart_pkt.h"

/* ============================================================
 * GPIO logical IDs
 * ============================================================ */

#define GPIO_ID_1       1U
#define GPIO_ID_2       2U
#define GPIO_ID_3       3U
#define GPIO_ID_4       4U
#define GPIO_ID_5       5U
#define GPIO_ID_6       6U
#define GPIO_ID_7       7U
#define GPIO_ID_8       8U
#define GPIO_ID_9       9U
#define GPIO_ID_10      10U
#define GPIO_ID_11      11U
#define GPIO_ID_12      12U
#define GPIO_ID_13      13U

#define GPIO_CONTROL_COUNT     13U
/* ============================================================
 * GPIO command payload
 *
 * Byte 0 = logical GPIO ID
 * Byte 1 = direction
 * Byte 2 = requested state
 *
 * direction:
 *     0 = INPUT
 *     1 = OUTPUT
 *
 * state:
 *     0 = LOW
 *     1 = HIGH
 *
 * For INPUT, state is ignored.
 * ============================================================ */


typedef struct
{
    uint8_t gpio_id;
    uint8_t direction;
    uint8_t state;
    uint8_t valid;
} GpioStatus_t;

/* ============================================================
 * API
 * ============================================================ */

/*
 * Initialize all remotely controllable GPIOs.
 *
 * Safe startup:
 *     - configure as GPIO
 *     - configure as INPUT
 *     - no active drive
 */
void Gpio_ControlInit(void);

/*
 * Execute GPIO command received from ESP32.
 *
 * payload:
 *     [0] logical GPIO ID
 *     [1] direction
 *     [2] state
 *
 * len:
 *     payload length
 *
 * Returns:
 *     0 = success
 *    -1 = invalid command
 *    -2 = invalid GPIO ID
 */
int Gpio_ControlProcessCommand(const uint8_t *data, uint16_t len);

/*
 * Read actual physical GPIO level.
 *
 * Returns:
 *     GPIO_STATE_LOW
 *     GPIO_STATE_HIGH
 */
uint8_t Gpio_ControlRead(uint8_t gpio_id);

/*
 * Get complete status of one GPIO.
 *
 * Returns:
 *     0 = valid
 *    -1 = invalid GPIO ID
 */
int Gpio_ControlGetStatus(uint8_t gpio_id, GpioStatus_t *status);

/*
 * Send all GPIO states to ESP32.
 *
 * This is used:
 *     - after initialization
 *     - after GPIO command
 *     - after STATUS request
 *     - periodically if required
 */
void Gpio_ControlSendStatus(void);

/*
 * Optional periodic GPIO monitoring.
 *
 * Reads the actual physical pin and reports changes.
 */
void Gpio_ControlTask(void);

#endif /* GPIO_CONTROL_H */
