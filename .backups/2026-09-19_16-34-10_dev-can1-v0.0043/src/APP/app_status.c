#include <stddef.h>

#include "app_status.h"
#include "app_modules.h"

static AppStatus_t s_status;

void App_StatusInit(void)
{
    s_status.imu             = 0U;
    s_status.csa             = 0U;
    s_status.can1            = 0U;
    s_status.can2            = 0U;
    s_status.flm             = 0U;
    s_status.ota             = 0U;
    s_status.ota_active      = 0U;
    s_status.reserved        = 0U;

    s_status.uptime_ms       = 0U;
    s_status.heartbeat_count = 0U;
}

void App_StatusUpdate(uint32_t ms)
{
    s_status.uptime_ms = ms;

    s_status.imu  = App_ImuEnabled();
    s_status.csa  = App_CsaEnabled();
    s_status.can1 = App_Can1Enabled();
    s_status.can2 = App_Can2Enabled();
    s_status.flm  = App_FlmEnabled();
    s_status.ota  = App_OtaEnabled();
}

void App_StatusGet(AppStatus_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = s_status;
}

void App_StatusSetOtaActive(uint8_t active)
{
    s_status.ota_active = (active != 0U) ? 1U : 0U;
}

void App_StatusHeartbeat(void)
{
    s_status.heartbeat_count++;
}

uint32_t App_GetUptimeMs(void)
{
    return s_status.uptime_ms;
}

uint32_t App_GetHeartbeatCount(void)
{
    return s_status.heartbeat_count;
}
