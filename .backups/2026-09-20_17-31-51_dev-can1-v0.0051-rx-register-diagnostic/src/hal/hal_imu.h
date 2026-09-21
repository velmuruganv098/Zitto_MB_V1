#ifndef HAL_IMU_H
#define HAL_IMU_H
#include "common/common_types.h"

typedef struct {
    S32 accelX_cmss;
    S32 accelY_cmss;
    S32 accelZ_cmss;
    S32 gyroX_dps;
    S32 gyroY_dps;
    S32 gyroZ_dps;
    S32 temp_cC;
} Hal_Imu_Data_t;

typedef struct {
    S32 ax; S32 ay; S32 az;
    S32 gx; S32 gy; S32 gz;
} Hal_Imu_Bias_t;

Std_ReturnType Hal_Imu_Init      (void);
Std_ReturnType Hal_Imu_Calibrate (void);
Std_ReturnType Hal_Imu_GetData   (Hal_Imu_Data_t *pOut);
void           Hal_Imu_GetBias   (Hal_Imu_Bias_t *pBias);

#endif
