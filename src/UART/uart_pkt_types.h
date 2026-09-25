#ifndef UART_PKT_TYPES_H
#define UART_PKT_TYPES_H

#include <stdint.h>
#include <stddef.h>


/* ============================================================================
 * UART PACKET LIMITS
 * ========================================================================== */

#ifndef UART_PKT_MAX_PAYLOAD
#define UART_PKT_MAX_PAYLOAD    256U
#endif


/* ============================================================================
 * GENERIC UART PACKET
 *
 * Used internally by uart_pkt.c and command processing.
 *
 * Packet format conceptually:
 *
 *   VERSION | TYPE | LENGTH | SEQ | PAYLOAD
 * ========================================================================== */

typedef struct
{
    uint8_t version;
    uint8_t type;
    uint8_t seq;

    uint16_t len;

    uint8_t data[UART_PKT_MAX_PAYLOAD];

} UartPkt_t;

/* ============================================================================
 * IMU PACKET
 *
 * Acceleration:
 *      milli-g
 *
 * Gyroscope:
 *      milli-degrees per second
 *
 * Temperature:
 *      0.1 degree Celsius
 * ========================================================================== */

typedef struct
{
    int32_t ax_mg;
    int32_t ay_mg;
    int32_t az_mg;

    int32_t gx_mdps;
    int32_t gy_mdps;
    int32_t gz_mdps;

    int16_t temp_c10;

    uint32_t ts_ms;

    /* V0.0073 motion tracking (appended - bytes 0..31 unchanged) */
    int32_t  px_mm10;       /* displacement since IMU start, 0.1 mm, X */
    int32_t  py_mm10;       /* Y */
    int32_t  pz_mm10;       /* Z (up) */
    uint32_t dist_mm10;     /* travelled path length since IMU start, 0.1 mm */
    int16_t  roll_cd;       /* orientation, 0.01 deg */
    int16_t  pitch_cd;
    int16_t  yaw_cd;
    uint8_t  moving;        /* 1 = board moving (not zero-velocity) */
    uint8_t  flags;         /* bit0 calibrated, bit1 tracking */
    uint32_t imu_up_ms;     /* time since tracking started (after calibration) */
    int32_t  speed_mms;     /* current speed, mm/s */

} ImuPkt_t;                 /* 64 bytes */


/* ============================================================================
 * CSA PACKET
 *
 * Keep these fields only if they match csa.c.
 *
 * IMPORTANT:
 * If your csa.c uses different member names, those names must be preserved.
 * ========================================================================== */

typedef struct
{
    int32_t current_ma;

    int32_t voltage_mv;

    int32_t power_mw;

    uint32_t ts_ms;

} CsaPkt_t;


/* ============================================================================
 * CAN FRAME PACKET
 *
 * bus:
 *      1 = CAN1
 *      2 = CAN2
 *
 * ide:
 *      0 = Standard ID
 *      1 = Extended ID
 *
 * rtr:
 *      0 = Data frame
 *      1 = Remote frame
 * ========================================================================== */

typedef struct
{
    uint8_t bus;

    uint8_t ide;

    uint8_t rtr;

    uint8_t dlc;

    uint32_t can_id;

    uint8_t data[8];

    uint32_t ts_ms;

} CanFramePkt_t;


/* ============================================================================
 * CAN STATUS PACKET
 *
 * bus:
 *      1 = CAN1
 *      2 = CAN2
 *
 * state:
 *      Raw Can1_State_t value when bus==1, raw Can2_State_t value when
 *      bus==2. The two enums are not the same - the receiver must use
 *      bus to pick the right decode table.
 *
 * tx_err_cnt / rx_err_cnt:
 *      CAN1 only (can1.h tracks them separately). Always 0 for CAN2,
 *      which only exposes a combined error_count.
 * ========================================================================== */

typedef struct
{
    uint8_t bus;

    uint8_t state;

    uint8_t ready;

    uint8_t bus_off;

    uint32_t detected_baud_kbps;

    uint32_t rx_count;

    uint32_t error_count;

    uint32_t tx_err_cnt;

    uint32_t rx_err_cnt;

    uint32_t irq_count;

    uint32_t error_irq_count;

    uint32_t mb_irq_count;

    uint32_t ts_ms;

} CanStatusPkt_t;


/* ============================================================================
 * FLM (FLASH LOG) STATUS PACKET
 * ========================================================================== */

typedef struct
{
    uint32_t total_pages;

    uint32_t used_pages;

    uint32_t free_pages;

    uint32_t next_page;

    uint32_t last_page;

    uint32_t records;

    uint32_t ts_ms;

} FlmStatusPkt_t;


/* ============================================================================
 * GENERAL STATUS PACKET
 * ========================================================================== */

typedef struct
{
    uint8_t imu_en;
    uint8_t csa_en;
    uint8_t can1_en;
    uint8_t can2_en;
    uint8_t flm_en;

    uint8_t ota_pending;

    uint8_t reserved0;
    uint8_t reserved1;

    uint32_t can1_baud_kbps;
    uint32_t can2_baud_kbps;

    uint32_t flash_free_pages;

    uint32_t uptime_ms;
    uint32_t reset_cause;

    uint32_t heartbeat_count;

} StatusPkt_t;

/* ============================================================================
 * LED CONTROL COMMAND
 *
 * Payload for CMD_LED_CTRL
 * ========================================================================== */

typedef struct
{
    uint16_t period_ms;

    uint8_t duty_pct;

    uint8_t reserved;

} LedCtrlCmd_t;


#endif /* UART_PKT_TYPES_H */
