#ifndef COMMON_CONFIG_H
#define COMMON_CONFIG_H

#include "common_types.h"

/* System */
#define CFG_SYS_CLOCK_HZ        80000000UL

/* I2C bit-bang */
#define CFG_I2C_SDA_PIN         2U
#define CFG_I2C_SCL_PIN         3U
#define CFG_I2C_HALF_BIT_US     5U

/* IMU */
#define CFG_IMU_I2C_ADDR        0x69U
#define CFG_IMU_GYRO_CFG        0x68U
#define CFG_IMU_ACCEL_CFG       0x68U
#define CFG_IMU_TASK_MS         50U
#define CFG_IMU_CAL_SAMPLES     40U
#define CFG_IMU_STILL_THR       80
#define CFG_IMU_ZVU_CNT         4U
#define CFG_IMU_ZPU_CNT         20U
#define CFG_ACCEL_LSB_PER_G     16384
#define CFG_GYRO_LSB_PER_DPS    131
#define CFG_G_MM_S2             9810

/* LED */
#define CFG_LED1_PIN            0U
#define CFG_LED2_PIN            5U

#endif
