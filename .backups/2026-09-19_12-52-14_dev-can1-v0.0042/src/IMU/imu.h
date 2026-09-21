#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

#include "../UART/uart_pkt_types.h"


/* ============================================================
 * Feature Configuration
 * ============================================================ */

#ifndef IMU_FEAT_ACCEL
#define IMU_FEAT_ACCEL          1
#endif

#ifndef IMU_FEAT_GYRO
#define IMU_FEAT_GYRO           1
#endif

#ifndef IMU_FEAT_TEMP
#define IMU_FEAT_TEMP           1
#endif

#ifndef IMU_FEAT_DISPLACEMENT
#define IMU_FEAT_DISPLACEMENT   1
#endif


/* ============================================================
 * ICM-42670 Register Definitions
 * ============================================================ */

#define ICM_REG_MCLK_RDY         0x00U
#define ICM_REG_SIG_PATH_RST     0x02U
#define ICM_REG_TEMP_DATA1       0x09U
#define ICM_REG_ACCEL_X1         0x0BU

#define ICM_REG_PWR_MGMT0        0x1FU
#define ICM_REG_GYRO_CFG0        0x20U
#define ICM_REG_ACCEL_CFG0       0x21U

#define ICM_REG_INTF_CFG1        0x36U
#define ICM_REG_WHO_AM_I         0x75U


/* ============================================================
 * IMU Configuration
 * ============================================================ */

#define IMU_I2C_ADDR             0x69U

#define IMU_GYRO_CFG             0x68U
#define IMU_ACCEL_CFG            0x68U


/* ============================================================
 * Timing
 * ============================================================ */

#define IMU_DT_MS                50U

#define IMU_CAL_SAMPLES          40U


/* ============================================================
 * Motion Thresholds
 * ============================================================ */

#define IMU_STILL_THR            80

#define IMU_GYRO_ZRO_THR         200

#define IMU_ZVU_CNT              4U

#define IMU_ZPU_CNT              20U


/* ============================================================
 * Public API
 * ============================================================ */

void Imu_Init(void);

void Imu_Calibrate(void);

void Imu_Task(void);

uint8_t Imu_IsReady(void);


/* Get latest IMU packet */

void Imu_GetLastPkt(ImuPkt_t *out);


#endif /* IMU_H */
