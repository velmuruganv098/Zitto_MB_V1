#ifndef MCAL_I2C_H
#define MCAL_I2C_H
#include "common/common_types.h"

void           Mcal_I2c_Init  (void);
Std_ReturnType Mcal_I2c_Write (U8 addr, U8 reg, U8 val);
Std_ReturnType Mcal_I2c_Read  (U8 addr, U8 reg, U8 *pBuf, U8 len);
Bool           Mcal_I2c_Probe (U8 addr);

#endif
