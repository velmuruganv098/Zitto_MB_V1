#ifndef APP_MODULES_H
#define APP_MODULES_H

#include <stdint.h>

typedef enum
{
    APP_MODULE_IMU = 0,
    APP_MODULE_CSA,
    APP_MODULE_CAN1,
    APP_MODULE_CAN2,
    APP_MODULE_FLM,
    APP_MODULE_OTA,
    APP_MODULE_GPIO,
    APP_MODULE_MAX
} AppModule_t;

void App_ModulesInit(void);
uint8_t App_ModuleGet(AppModule_t module);
uint8_t App_ModuleSet(AppModule_t module,uint8_t enable);

uint8_t App_ImuEnabled(void);
uint8_t App_CsaEnabled(void);
uint8_t App_Can1Enabled(void);
uint8_t App_Can2Enabled(void);
uint8_t App_FlmEnabled(void);
uint8_t App_OtaEnabled(void);
uint8_t App_GpioEnabled(void);

#endif
