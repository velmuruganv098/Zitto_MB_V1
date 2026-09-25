/*
 * gpio_control.c
 *
 * Zitto_MB_V1 / S32K144
 *
 * Remote GPIO control service.
 *
 * IMPORTANT:
 * Only GPIOs explicitly present in g_gpio_table[] are controllable.
 *
 * Hardware-reserved pins are deliberately NOT present in the table.
 *
 * Reserved examples:
 *
 *   PTB2  - CAN shutdown
 *   PTC7  - ADC throttle
 *   PTC6  - ADC temperature
 *   PTA7  - OSC32_IN
 *   PTA10 - MCU-WP-FLM
 *   PTA11 - MCU-HD-FLM / HOLD
 *   PTC2  - LPUART0_RX
 *   PTC3  - LPUART0_TX
 *   PTA2  - I2C0 SDA
 *   PTA3  - I2C0 SCL
 *   PTB0 etc. are included only when confirmed free.
 */

#include "gpio_control.h"
#include "uart_pkt.h"
#include "SEGGER_RTT.h"
#include "DEBUG/debug_rtt.h"
#include <string.h>

/* ============================================================
 * Internal GPIO descriptor
 * ============================================================ */

typedef struct
{
    uint8_t     logical_id;
    uint8_t     port;
    uint8_t     pin;
    uint8_t     default_direction;
    uint8_t     default_state;

    GPIO_Type  *gpio;
    PORT_Type  *port_base;
    uint32_t    pcc_index;
} GpioDescriptor_t;

/* ============================================================
 * GPIO table
 *
 * 13 remotely controllable GPIOs.
 *
 * Physical mapping:
 *
 * ID1   -> Pin 1  -> PTD1
 * ID2   -> Pin 2  -> PTD0
 * ID3   -> Pin 3  -> PTE5
 * ID4   -> Pin 4  -> PTE4
 * ID5   -> Pin 12 -> PTE9
 * ID6   -> Pin 13 -> PTE8
 * ID7   -> Pin 18 -> PTD5
 * ID8   -> Pin 19 -> PTC1
 * ID9   -> Pin 21 -> PTC15
 * ID10  -> Pin 22 -> PTC14
 * ID11  -> Pin 23 -> PTB3
 * ID12  -> Pin 25 -> PTB1
 * ID13  -> Pin 26 -> PTB0
 *
 * Startup:
 *     INPUT
 *     LOW / inactive
 * ============================================================ */

static GpioDescriptor_t g_gpio_table[GPIO_CONTROL_COUNT] =
{
    {
        GPIO_ID_1,
        'D', 1U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTD, PORTD,
        PCC_PORTD_INDEX
    },

    {
        GPIO_ID_2,
        'D', 0U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTD, PORTD,
        PCC_PORTD_INDEX
    },

    {
        GPIO_ID_3,
        'E', 5U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTE, PORTE,
        PCC_PORTE_INDEX
    },

    {
        GPIO_ID_4,
        'E', 4U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTE, PORTE,
        PCC_PORTE_INDEX
    },

    {
        GPIO_ID_5,
        'E', 9U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTE, PORTE,
        PCC_PORTE_INDEX
    },

    {
        GPIO_ID_6,
        'E', 8U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTE, PORTE,
        PCC_PORTE_INDEX
    },

    {
        GPIO_ID_7,
        'D', 5U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTD, PORTD,
        PCC_PORTD_INDEX
    },

    {
        GPIO_ID_8,
        'C', 1U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTC, PORTC,
        PCC_PORTC_INDEX
    },

    {
        GPIO_ID_9,
        'C', 15U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTC, PORTC,
        PCC_PORTC_INDEX
    },

    {
        GPIO_ID_10,
        'C', 14U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTC, PORTC,
        PCC_PORTC_INDEX
    },

    {
        GPIO_ID_11,
        'B', 3U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTB, PORTB,
        PCC_PORTB_INDEX
    },

    {
        GPIO_ID_12,
        'B', 1U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTB, PORTB,
        PCC_PORTB_INDEX
    },

    {
        GPIO_ID_13,
        'B', 0U,
        GPIO_DIR_INPUT,
        GPIO_STATE_LOW,
        PTB, PORTB,
        PCC_PORTB_INDEX
    }
};

/* ============================================================
 * Runtime direction/state tracking
 *
 * These values are only software state.
 *
 * Physical GPIO readback is ALWAYS obtained from PDIR/PDIR
 * when status is generated.
 * ============================================================ */

static uint8_t g_gpio_direction[GPIO_CONTROL_COUNT];
static uint8_t g_gpio_state[GPIO_CONTROL_COUNT];

/* ============================================================
 * Find GPIO descriptor
 * ============================================================ */

static GpioDescriptor_t *gpio_find(uint8_t logical_id)
{
    uint8_t i;

    for(i = 0U; i < GPIO_CONTROL_COUNT; i++)
    {
        if(g_gpio_table[i].logical_id == logical_id)
        {
            return &g_gpio_table[i];
        }
    }

    return NULL;
}

/* ============================================================
 * Enable PORT clock
 * ============================================================ */

static void gpio_enable_clock(const GpioDescriptor_t *g)
{
    if(g == NULL)
    {
        return;
    }

    PCC->PCCn[g->pcc_index] |= PCC_PCCn_CGC_MASK;
}

/* ============================================================
 * Configure GPIO as digital GPIO
 * ============================================================ */

static void gpio_config_pin(const GpioDescriptor_t *g)
{
    if(g == NULL)
    {
        return;
    }

    /*
     * MUX = 1 -> GPIO
     *
     * No pull-up/down is forced here.
     *
     * External hardware should determine the safe idle level
     * when the pin is configured as input.
     */
    g->port_base->PCR[g->pin] = PORT_PCR_MUX(1U);
}

/* ============================================================
 * Read actual physical level
 * ============================================================ */

static uint8_t gpio_read_physical(const GpioDescriptor_t *g)
{
    uint32_t mask;

    if(g == NULL)
    {
        return GPIO_STATE_LOW;
    }

    mask = (1UL << g->pin);

    return ((g->gpio->PDIR & mask) != 0UL)
             ? GPIO_STATE_HIGH
             : GPIO_STATE_LOW;
}

/* ============================================================
 * Set output level
 * ============================================================ */

static void gpio_write(const GpioDescriptor_t *g, uint8_t state)
{
    uint32_t mask;

    if(g == NULL)
    {
        return;
    }

    mask = (1UL << g->pin);

    if(state == GPIO_STATE_HIGH)
    {
        g->gpio->PSOR = mask;
    }
    else
    {
        g->gpio->PCOR = mask;
    }
}

/* ============================================================
 * Configure direction
 * ============================================================ */

static void gpio_set_direction(const GpioDescriptor_t *g,
                               uint8_t direction)
{
    uint32_t mask;

    if(g == NULL)
    {
        return;
    }

    mask = (1UL << g->pin);

    if(direction == GPIO_DIR_OUTPUT)
    {
        g->gpio->PDDR |= mask;
    }
    else
    {
        g->gpio->PDDR &= ~mask;
    }
}

/* ============================================================
 * Initialization
 * ============================================================ */

void Gpio_ControlInit(void)
{
    uint8_t i;

    for(i = 0U; i < GPIO_CONTROL_COUNT; i++)
    {
        GpioDescriptor_t *g = &g_gpio_table[i];

        /*
         * Enable port clock.
         */
        gpio_enable_clock(g);

        /*
         * Select GPIO function.
         */
        gpio_config_pin(g);

        /*
         * Put output latch LOW BEFORE changing direction.
         *
         * This prevents an unwanted HIGH pulse if the pin
         * becomes an output later.
         */
        gpio_write(g, GPIO_STATE_LOW);

        /*
         * Safe startup direction = INPUT.
         */
        gpio_set_direction(g, GPIO_DIR_INPUT);

        g_gpio_direction[i] = GPIO_DIR_INPUT;

        /*
         * Record actual physical level.
         */
        g_gpio_state[i] = gpio_read_physical(g);
    }

    SEGGER_RTT_printf(
        0,
        "[GPIO] Initialized: %u controllable GPIOs\r\n",
        (unsigned)GPIO_CONTROL_COUNT);

    /*
     * Tell ESP32 the initial actual states.
     */
    Gpio_ControlSendStatus();
}

/* ============================================================
 * Process GPIO SET command
 * ============================================================ */

int Gpio_ControlProcessCommand(const uint8_t *data, uint16_t len)
{
    GpioSetCmd_t cmd;
    GpioDescriptor_t *g;
    uint8_t actual_state;
    uint8_t index = 0U;

    if(data == NULL)
    {
        return -1;
    }

    if(len < (uint16_t)sizeof(GpioSetCmd_t))
    {
        Uart_Pkt_SendLog("GPIO:invalid_len");
        return -1;
    }

    /*
     * memcpy avoids alignment problems when UART payload
     * is not naturally aligned.
     */
    memcpy(&cmd, data, sizeof(GpioSetCmd_t));

    /*
     * Validate GPIO ID.
     */
    g = gpio_find(cmd.gpio_id);

    if(g == NULL)
    {
        Uart_Pkt_SendLog("GPIO:invalid_id");
        return -2;
    }

    /*
     * Validate direction.
     */
    if((cmd.direction != GPIO_DIR_INPUT) &&
       (cmd.direction != GPIO_DIR_OUTPUT))
    {
        Uart_Pkt_SendLog("GPIO:invalid_direction");
        return -1;
    }

    /*
     * Validate state.
     */
    if((cmd.state != GPIO_STATE_LOW) &&
       (cmd.state != GPIO_STATE_HIGH))
    {
        Uart_Pkt_SendLog("GPIO:invalid_state");
        return -1;
    }

    /*
     * Find software table index.
     */
    while(index < GPIO_CONTROL_COUNT)
    {
        if(g_gpio_table[index].logical_id == cmd.gpio_id)
        {
            break;
        }

        index++;
    }

    if(index >= GPIO_CONTROL_COUNT)
    {
        return -2;
    }

    /*
     * Ensure GPIO clock and GPIO mux.
     */
    gpio_enable_clock(g);
    gpio_config_pin(g);

    /*
     * IMPORTANT:
     *
     * When changing to OUTPUT:
     *
     * 1. Set output latch first.
     * 2. Then change direction.
     *
     * This avoids an unwanted opposite-level pulse.
     */
    if(cmd.direction == GPIO_DIR_OUTPUT)
    {
        gpio_write(g, cmd.state);
        gpio_set_direction(g, GPIO_DIR_OUTPUT);
    }
    else
    {
        /*
         * Return to input.
         *
         * Keep output latch LOW so that if this GPIO is later
         * changed to output, it starts from a known state.
         */
        gpio_write(g, GPIO_STATE_LOW);
        gpio_set_direction(g, GPIO_DIR_INPUT);
    }

    /*
     * Update software state.
     */
    g_gpio_direction[index] = cmd.direction;

    /*
     * ALWAYS read the physical pin.
     *
     * Do NOT simply report cmd.state.
     */
    actual_state = gpio_read_physical(g);

    g_gpio_state[index] = actual_state;

    /*
     * Debug output.
     */
    SEGGER_RTT_printf(
        0,
        "[GPIO] ID=%u PT%c%u DIR=%s REQ=%s ACT=%s\r\n",
        (unsigned)cmd.gpio_id,
        g->port,
        (unsigned)g->pin,
        (cmd.direction == GPIO_DIR_OUTPUT) ? "OUT" : "IN",
        (cmd.state == GPIO_STATE_HIGH) ? "HIGH" : "LOW",
        (actual_state == GPIO_STATE_HIGH) ? "HIGH" : "LOW");

    /*
     * Send ACTUAL physical state to ESP32.
     *
     * This is important:
     *
     * Server requested:
     *
     *     GPIO 5 = HIGH
     *
     * MCU reports:
     *
     *     GPIO 5 = HIGH
     *
     * only after reading the physical pin.
     */
    Gpio_ControlSendStatus();

    return 0;
}

/* ============================================================
 * Read GPIO
 * ============================================================ */

uint8_t Gpio_ControlRead(uint8_t gpio_id)
{
    GpioDescriptor_t *g;
    uint8_t i;

    g = gpio_find(gpio_id);

    if(g == NULL)
    {
        return GPIO_STATE_LOW;
    }

    for(i = 0U; i < GPIO_CONTROL_COUNT; i++)
    {
        if(g_gpio_table[i].logical_id == gpio_id)
        {
            g_gpio_state[i] = gpio_read_physical(g);
            return g_gpio_state[i];
        }
    }

    return GPIO_STATE_LOW;
}

/* ============================================================
 * Get GPIO status
 * ============================================================ */

int Gpio_ControlGetStatus(uint8_t gpio_id,
                          GpioStatus_t *status)
{
    GpioDescriptor_t *g;
    uint8_t i;

    if(status == NULL)
    {
        return -1;
    }

    memset(status, 0, sizeof(GpioStatus_t));

    g = gpio_find(gpio_id);

    if(g == NULL)
    {
        return -1;
    }

    for(i = 0U; i < GPIO_CONTROL_COUNT; i++)
    {
        if(g_gpio_table[i].logical_id == gpio_id)
        {
            status->gpio_id  = gpio_id;
            status->direction = g_gpio_direction[i];

            /*
             * ALWAYS obtain actual physical level.
             */
            status->state = gpio_read_physical(g);

            g_gpio_state[i] = status->state;

            status->valid = 1U;

            return 0;
        }
    }

    return -1;
}

/* ============================================================
 * Send complete GPIO status
 * ============================================================ */

void Gpio_ControlSendStatus(void)
{
    GpioStatusPayload_t payload;
    GpioStatus_t status;
    uint8_t i;

    payload.count = GPIO_CONTROL_COUNT;

    for(i = 0U; i < GPIO_CONTROL_COUNT; i++)
    {
        uint8_t gpio_id = (uint8_t)(i + 1U);

        if(Gpio_ControlGetStatus(gpio_id, &status) == 0)
        {
            payload.gpio[i].gpio_id = status.gpio_id;
            payload.gpio[i].direction = status.direction;
            payload.gpio[i].actual_state = status.state;
        }
        else
        {
            payload.gpio[i].gpio_id = gpio_id;
            payload.gpio[i].direction = GPIO_DIR_INPUT;
            payload.gpio[i].actual_state = GPIO_STATE_LOW;
        }
    }

    (void)Uart_Pkt_SendGpioStatus(&payload);
}

/* ============================================================
 * Periodic GPIO task
 * ============================================================ */

void Gpio_ControlTask(void)
{
    uint8_t i;

    /*
     * Read every physical GPIO.
     *
     * If hardware changes the input level, ESP32 can be
     * informed of the new actual state.
     *
     * For now we only refresh the cached state.
     */
    uint8_t changed = 0U;

    for(i = 0U; i < GPIO_CONTROL_COUNT; i++)
    {
        uint8_t now_state = gpio_read_physical(&g_gpio_table[i]);
        if(now_state != g_gpio_state[i])
        {
            EVT_LOG("[GPIO] ID=%u PT%c%u %s changed -> %s\r\n",
                    (unsigned)g_gpio_table[i].logical_id, g_gpio_table[i].port,
                    (unsigned)g_gpio_table[i].pin,
                    (g_gpio_direction[i] == GPIO_DIR_OUTPUT) ? "OUT" : "IN",
                    now_state ? "HIGH" : "LOW");
            changed = 1U;
        }
        g_gpio_state[i] = now_state;
    }

    /* V0.0073: push the new state so the UI follows real pin changes. */
    if(changed != 0U)
    {
        Gpio_ControlSendStatus();
    }
}
