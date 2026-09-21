#ifndef CSA_H
#define CSA_H

#include <stdint.h>
#include "../UART/uart_pkt.h"

#define CSA_I2C_ADDR 0x40U
#define CSA_SHUNT_UOHM 1000U
#define CSA_MAX_CURRENT_MA 15000U
#define CSA_PRINT_EVERY 20U

#define INA226_REG_CONFIG 0x00U
#define INA226_REG_VSHUNT 0x01U
#define INA226_REG_VBUS 0x02U
#define INA226_REG_POWER 0x03U
#define INA226_REG_CURRENT 0x04U
#define INA226_REG_CALIB 0x05U
#define INA226_REG_MASKEN 0x06U
#define INA226_REG_ALERT 0x07U
#define INA226_REG_MFR_ID 0xFEU
#define INA226_REG_DIE_ID 0xFFU

#define INA226_MFR_ID_VAL 0x5449U
#define INA226_DIE_ID_MASK 0xFFF0U
#define INA226_DIE_ID_VAL 0x2260U
#define INA226_CFG_VAL 0x4467U

typedef struct{
uint8_t connected;
int32_t vbus_mv;
int32_t vshunt_uv;
int32_t current_ma;
int32_t power_mw;
uint16_t mfr_id;
uint16_t die_id;
}Csa_Data_t;

void Csa_Init(void);
void Csa_Task(void);
void Csa_GetData(Csa_Data_t *out);
void Csa_GetLastPkt(CsaPkt_t *out);
uint8_t Csa_IsReady(void);

#endif
