#include "APP/app_modules.h"
#include "app_config.h"

static uint8_t s_module[APP_MODULE_MAX];

void App_ModulesInit(void)
{
    s_module[APP_MODULE_IMU] =
        APP_MODULE_IMU_ENABLE ? 1U : 0U;

    s_module[APP_MODULE_CSA] =
        APP_MODULE_CSA_ENABLE ? 1U : 0U;

    s_module[APP_MODULE_CAN1] =
        APP_MODULE_CAN1_ENABLE ? 1U : 0U;

    s_module[APP_MODULE_CAN2] =
        APP_MODULE_CAN2_ENABLE ? 1U : 0U;

    s_module[APP_MODULE_FLM] =
        APP_MODULE_FLM_ENABLE ? 1U : 0U;

    s_module[APP_MODULE_OTA] =
        APP_MODULE_OTA_ENABLE ? 1U : 0U;

    s_module[APP_MODULE_GPIO] =
        APP_MODULE_GPIO_ENABLE ? 1U : 0U;
}

uint8_t App_ModuleGet(AppModule_t module)
{
    if(module >= APP_MODULE_MAX)
    {
        return 0U;
    }

    return s_module[module];
}

uint8_t App_ModuleSet(
    AppModule_t module,
    uint8_t enable
)
{
    if(module >= APP_MODULE_MAX)
    {
        return 0U;
    }

    s_module[module] = enable ? 1U : 0U;

    return 1U;
}

uint8_t App_ImuEnabled(void)
{
    return s_module[APP_MODULE_IMU];
}

uint8_t App_CsaEnabled(void)
{
    return s_module[APP_MODULE_CSA];
}

uint8_t App_Can1Enabled(void)
{
    return s_module[APP_MODULE_CAN1];
}

uint8_t App_Can2Enabled(void)
{
    return s_module[APP_MODULE_CAN2];
}

uint8_t App_FlmEnabled(void)
{
    return s_module[APP_MODULE_FLM];
}

uint8_t App_OtaEnabled(void)
{
    return s_module[APP_MODULE_OTA];
}

uint8_t App_GpioEnabled(void)
{
    return s_module[APP_MODULE_GPIO];
}
