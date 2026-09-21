#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * ============================================================
 * Zitto_MB_V1
 * Application Configuration
 * ============================================================
 */

/* ------------------------------------------------------------
 * Module Enable / Disable
 * ------------------------------------------------------------ */

#define APP_MODULE_CAN1_ENABLE          1U
#define APP_MODULE_CAN2_ENABLE          1U
#define APP_MODULE_IMU_ENABLE           1U
#define APP_MODULE_GPIO_ENABLE          1U
#define APP_MODULE_CSA_ENABLE           1U
#define APP_MODULE_FLM_ENABLE           1U
#define APP_MODULE_OTA_ENABLE           1U


/* ------------------------------------------------------------
 * Application Timing
 * ------------------------------------------------------------ */

#define APP_TASK_PERIOD_MS              10U

#define APP_HEARTBEAT_PERIOD_MS         1000U

#define APP_STATUS_PERIOD_MS            1000U


/* ------------------------------------------------------------
 * UART
 * ------------------------------------------------------------ */

#define APP_UART_ENABLE                 1U


/* ------------------------------------------------------------
 * CAN
 * ------------------------------------------------------------ */

#define APP_CAN_ENABLE                  1U


/* ------------------------------------------------------------
 * GPIO
 * ------------------------------------------------------------ */

#define APP_GPIO_STATUS_PERIOD_MS       500U


/* ------------------------------------------------------------
 * IMU
 * ------------------------------------------------------------ */

#define APP_IMU_STATUS_PERIOD_MS        100U


/* ------------------------------------------------------------
 * Flash Logging Manager
 * ------------------------------------------------------------ */

#define APP_FLM_ENABLE                  1U


/* ------------------------------------------------------------
 * OTA
 * ------------------------------------------------------------ */

#define APP_OTA_ENABLE                  1U


/* OTA block count.
 *
 * IMPORTANT:
 * This must be defined only once.
 */
#ifndef OTA_BLOCK_COUNT
#define OTA_BLOCK_COUNT                 64U
#endif


/* ------------------------------------------------------------
 * Application Version
 * ------------------------------------------------------------ */

#define APP_VERSION_MAJOR               1U
#define APP_VERSION_MINOR               0U
#define APP_VERSION_PATCH               0U


#endif /* APP_CONFIG_H */
