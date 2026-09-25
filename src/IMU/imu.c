/*
 * imu.c  -  Zitto_MB_V1 / S32K144  -  ICM-42670 (bit-banged I2C on PTA2/PTA3)
 *
 * V0.0073 motion tracking
 * -----------------------
 *   - 100 Hz sampling (IMU_DT_MS = 10), sensor ODR 200 Hz, +-2 g / +-250 dps.
 *   - Non-blocking 2 s calibration at boot (board must be still): gravity
 *     vector, gravity magnitude and gyro bias.
 *   - Orientation: quaternion integrated from the bias-corrected gyro, with a
 *     slow accelerometer tilt correction while the board is still.
 *   - Linear acceleration = rotate(accel) into the start frame - gravity.
 *   - Zero-velocity update: when still (|a| ~ 1 g and |w| small for 100 ms)
 *     velocity is forced to 0 and position is frozen - position is NOT reset,
 *     so X/Y/Z is the board's displacement since power-on / tracking start.
 *   - Travelled distance = accumulated path length.
 *   Units reported: mm (0.1 mm resolution), mm/s, degrees.
 *
 * Accuracy note: this is double integration of a MEMS accelerometer.  Short
 * hand movements (a few seconds between still periods) are tracked to
 * roughly centimetre level; error grows with the time spent moving without a
 * still pause.  Imu_ZeroPosition() (CMD_IMU_ZERO) restarts the origin.
 */
#include "imu.h"
#include "S32K144.h"
#include "SEGGER_RTT.h"
#include "../UART/uart_pkt.h"
#include "DEBUG/debug_rtt.h"
/* The firmware links with -nodefaultlibs (no libm): tiny self-contained math.
 * sqrt uses the FPU instruction; atan uses a 0.0015 rad polynomial. */
static float m_sqrt(float x)
{
    float r;
    if(x <= 0.0f) { return 0.0f; }
    __asm volatile ("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
    return r;
}

static float m_abs(float x) { return (x < 0.0f) ? -x : x; }

static float m_atan_unit(float x)           /* |x| <= 1 */
{
    return 0.785398163f * x - x * (m_abs(x) - 1.0f) * (0.2447f + 0.0663f * m_abs(x));
}

static float m_atan2(float y, float x)
{
    float ax = m_abs(x), ay = m_abs(y), a;
    if((ax == 0.0f) && (ay == 0.0f)) { return 0.0f; }
    if(ax >= ay) { a = m_atan_unit(ay / ax); }
    else         { a = 1.570796327f - m_atan_unit(ax / ay); }
    if(x < 0.0f) { a = 3.141592654f - a; }
    return (y < 0.0f) ? -a : a;
}

static float m_asin(float x)
{
    return m_atan2(x, m_sqrt(1.0f - x * x));
}

#define BB_SDA 2U
#define BB_SCL 3U
#define BB_DLY 2U                   /* was 5: ~2x faster bit-banged I2C */

#define G_MM_S2             9806.65f
#define DEG2RAD             0.0174532925f
#define RAD2DEG             57.2957795f
#define STILL_ACC_G         0.035f  /* | |a| - g | below this ...          */
#define STILL_GYR_DPS       2.5f    /* ... and |w| below this ...           */
#define STILL_SAMPLES       10U     /* ... for 100 ms -> still (ZUPT)       */
#define ACC_DEADBAND_MMS2   25.0f   /* linear accel below this is noise     */
#define TILT_GAIN           0.02f   /* accel tilt correction per still sample */
#define V_LEAK              0.9995f /* velocity leak while moving (drift limiter) */

static int32_t s_ax0, s_ay0, s_az0;
static int32_t s_gx0, s_gy0, s_gz0;
static int32_t s_ax_mg, s_ay_mg, s_az_mg;
static int32_t s_gx_mdps, s_gy_mdps, s_gz_mdps;
static int16_t s_temp_c10;
static uint8_t s_ready;
static uint32_t s_n;

/* tracking state (floats: Cortex-M4F hard FPU) */
static float s_q0 = 1.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;
static float s_vx, s_vy, s_vz;          /* mm/s */
static float s_px, s_py, s_pz;          /* mm   */
static float s_dist;                    /* mm   */
static float s_gmag = 1.0f;             /* measured gravity magnitude, g */
static uint8_t  s_still_cnt;
static uint8_t  s_moving;
static uint8_t  s_tracking;
static uint32_t s_track_start_ms;
static uint32_t s_last_ms;

#define IMU_CAL_TIMEOUT_MS   (IMU_CAL_SAMPLES * IMU_DT_MS * 3U)

typedef enum
{
    IMU_CAL_IDLE = 0,
    IMU_CAL_RUNNING,
    IMU_CAL_DONE,
    IMU_CAL_TIMED_OUT
} ImuCalState_t;

static ImuCalState_t s_cal_state = IMU_CAL_IDLE;
static uint16_t s_cal_index;
static int32_t  s_cal_sa, s_cal_sb, s_cal_sc, s_cal_sd, s_cal_se, s_cal_sf;
static int32_t  s_cal_ca, s_cal_cd;
static uint32_t s_cal_last_ms;
static uint32_t s_cal_start_ms;

/* ------------------------------------------------------------------ bit-banged I2C */
static void prv_Ms(volatile uint32_t ms)
{
    while(ms--) { volatile uint32_t n = 80000U; while(n--) { ; } }
}

static void prv_Us(volatile uint32_t us)
{
    while(us--) { volatile uint32_t n = 80U; while(n--) { ; } }
}

static void prv_SdaL(void) { PTA->PDDR |= (1UL << BB_SDA); PTA->PCOR = (1UL << BB_SDA); }
static void prv_SdaH(void) { PTA->PDDR &= ~(1UL << BB_SDA); }
static void prv_SclL(void) { PTA->PDDR |= (1UL << BB_SCL); PTA->PCOR = (1UL << BB_SCL); }

static void prv_SclH(void)
{
    volatile uint32_t t = 5000U;
    PTA->PDDR &= ~(1UL << BB_SCL);
    while(!(PTA->PDIR & (1UL << BB_SCL)) && --t) { ; }
    prv_Us(BB_DLY);
}

static uint8_t prv_SdaR(void) { return (uint8_t)((PTA->PDIR >> BB_SDA) & 1U); }

static void prv_I2cInit(void)
{
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTA->PCR[BB_SDA] = PORT_PCR_MUX(1U);
    PORTA->PCR[BB_SCL] = PORT_PCR_MUX(1U);
    prv_SdaH();
    PTA->PDDR &= ~(1UL << BB_SCL);
    prv_Us(BB_DLY);
}

static void prv_Start(void) { prv_SdaH(); prv_Us(BB_DLY); prv_SclH(); prv_SdaL(); prv_Us(BB_DLY); prv_SclL(); prv_Us(BB_DLY); }
static void prv_Stop(void)  { prv_SdaL(); prv_Us(BB_DLY); prv_SclH(); prv_SdaH(); prv_Us(BB_DLY); }

static void prv_WBit(uint8_t b)
{
    if(b) { prv_SdaH(); } else { prv_SdaL(); }
    prv_Us(BB_DLY);
    prv_SclH();
    prv_SclL();
    prv_Us(BB_DLY);
}

static uint8_t prv_RBit(void)
{
    uint8_t b;
    prv_SdaH();
    prv_Us(BB_DLY);
    prv_SclH();
    b = prv_SdaR();
    prv_SclL();
    prv_Us(BB_DLY);
    return b;
}

static uint8_t prv_WByte(uint8_t v)
{
    uint8_t i;
    for(i = 0U; i < 8U; i++) { prv_WBit((uint8_t)(v & 0x80U)); v <<= 1U; }
    return prv_RBit() ? 0U : 1U;
}

static uint8_t prv_RByte(uint8_t ack)
{
    uint8_t i, v = 0U;
    for(i = 0U; i < 8U; i++) { v = (uint8_t)((v << 1U) | prv_RBit()); }
    prv_WBit(ack ? 0U : 1U);
    return v;
}

static int prv_Wr(uint8_t reg, uint8_t val)
{
    uint8_t a1, a2, a3;
    prv_Start();
    a1 = prv_WByte((uint8_t)((IMU_I2C_ADDR << 1U) | 0U));
    a2 = prv_WByte(reg);
    a3 = prv_WByte(val);
    prv_Stop();
    return (a1 && a2 && a3) ? 0 : -1;
}

static int prv_Rd(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    if(!buf || !len) { return -1; }
    prv_Start();
    if(!prv_WByte((uint8_t)((IMU_I2C_ADDR << 1U) | 0U))) { prv_Stop(); return -1; }
    if(!prv_WByte(reg)) { prv_Stop(); return -1; }
    prv_Start();
    if(!prv_WByte((uint8_t)((IMU_I2C_ADDR << 1U) | 1U))) { prv_Stop(); return -1; }
    for(i = 0U; i < len; i++) { buf[i] = prv_RByte((uint8_t)(i < (uint8_t)(len - 1U))); }
    prv_Stop();
    return 0;
}

static int16_t prv_S16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8U) | lo);
}

/* ------------------------------------------------------------------ init */
void Imu_Init(void)
{
    uint8_t v = 0U, r = 0U;
    s_ready = 0U;
    prv_I2cInit();
    (void)prv_Wr(ICM_REG_SIG_PATH_RST, 0x10U);
    prv_Ms(10U);
    do { prv_Ms(1U); (void)prv_Rd(ICM_REG_MCLK_RDY, &v, 1U); r++; } while((v == 0U) && (r < 50U));
    (void)prv_Wr(ICM_REG_INTF_CFG1, 0x41U);
    prv_Ms(1U);
    (void)prv_Wr(ICM_REG_PWR_MGMT0, 0x10U);
    prv_Ms(2U);
    r = 0U;
    do { prv_Ms(1U); (void)prv_Rd(ICM_REG_MCLK_RDY, &v, 1U); r++; } while(!(v & 0x08U) && (r < 30U));
    (void)prv_Wr(ICM_REG_PWR_MGMT0, 0x14U);
    prv_Ms(5U);
    (void)prv_Wr(ICM_REG_PWR_MGMT0, 0x1CU);
    prv_Ms(5U);
    (void)prv_Wr(ICM_REG_PWR_MGMT0, 0x1FU);
    (void)prv_Wr(ICM_REG_GYRO_CFG0, IMU_GYRO_CFG);
    (void)prv_Wr(ICM_REG_ACCEL_CFG0, IMU_ACCEL_CFG);
    prv_Ms(50U);
    (void)prv_Rd(ICM_REG_ACCEL_X1, &v, 1U);
    s_ready = (v != 0x80U) ? 1U : 0U;
    RTT_LOG("[IMU] ICM-42670 %s (+-2g, +-250dps, ODR 200Hz, sample %ums)\r\n",
            s_ready ? "SENSORS ON" : "FAIL", (unsigned)IMU_DT_MS);
}

/* ------------------------------------------------------------------ tracking helpers */
static void q_normalize(void)
{
    float n = m_sqrt(s_q0 * s_q0 + s_q1 * s_q1 + s_q2 * s_q2 + s_q3 * s_q3);
    if(n > 0.0f) { s_q0 /= n; s_q1 /= n; s_q2 /= n; s_q3 /= n; }
}

/* body -> start-frame rotation of vector (x,y,z) */
static void q_rotate(float x, float y, float z, float *ox, float *oy, float *oz)
{
    float q0 = s_q0, q1 = s_q1, q2 = s_q2, q3 = s_q3;
    *ox = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * x + 2.0f * (q1 * q2 - q0 * q3) * y + 2.0f * (q1 * q3 + q0 * q2) * z;
    *oy = 2.0f * (q1 * q2 + q0 * q3) * x + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * y + 2.0f * (q2 * q3 - q0 * q1) * z;
    *oz = 2.0f * (q1 * q3 - q0 * q2) * x + 2.0f * (q2 * q3 + q0 * q1) * y + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * z;
}

static void tracking_start(void)
{
    /* Start frame: Z along the measured gravity (up), yaw 0.  The quaternion
     * rotating the body gravity direction g onto +Z is [1 + g.z, g x z]. */
    float ax = (float)s_ax0 / 16384.0f, ay = (float)s_ay0 / 16384.0f, az = (float)s_az0 / 16384.0f;
    float gx, gy, gz, roll, pitch;
    s_gmag = m_sqrt(ax * ax + ay * ay + az * az);
    if(s_gmag < 0.5f) { s_gmag = 1.0f; ax = 0.0f; ay = 0.0f; az = 1.0f; }
    gx = ax / s_gmag; gy = ay / s_gmag; gz = az / s_gmag;
    if(gz > -0.999f)
    {
        s_q0 = 1.0f + gz; s_q1 = gy; s_q2 = -gx; s_q3 = 0.0f;
    }
    else
    {
        s_q0 = 0.0f; s_q1 = 1.0f; s_q2 = 0.0f; s_q3 = 0.0f;     /* board upside down */
    }
    q_normalize();
    roll  = m_atan2(ay, az);
    pitch = m_atan2(-ax, m_sqrt(ay * ay + az * az));
    s_vx = s_vy = s_vz = 0.0f;
    s_px = s_py = s_pz = 0.0f;
    s_dist = 0.0f;
    s_still_cnt = STILL_SAMPLES;
    s_moving = 0U;
    s_tracking = 1U;
    s_track_start_ms = Uart_GetMs();
    s_last_ms = s_track_start_ms;
    EVT_LOG("[IMU] tracking started: g=%ld mg, roll=%ld pitch=%ld deg - position origin set\r\n",
            (long)(s_gmag * 1000.0f), (long)(roll * RAD2DEG), (long)(pitch * RAD2DEG));
}

void Imu_ZeroPosition(void)
{
    s_vx = s_vy = s_vz = 0.0f;
    s_px = s_py = s_pz = 0.0f;
    s_dist = 0.0f;
    s_track_start_ms = Uart_GetMs();
    EVT_LOG("[IMU] position / distance zeroed\r\n");
}

/* ------------------------------------------------------------------ calibration */
static void imu_cal_finish(void)
{
    if(s_cal_ca > 0) { s_ax0 = s_cal_sa / s_cal_ca; s_ay0 = s_cal_sb / s_cal_ca; s_az0 = s_cal_sc / s_cal_ca; }
    if(s_cal_cd > 0) { s_gx0 = s_cal_sd / s_cal_cd; s_gy0 = s_cal_se / s_cal_cd; s_gz0 = s_cal_sf / s_cal_cd; }
    RTT_LOG("[IMU] calibration: accel=(%ld,%ld,%ld) gyro bias=(%ld,%ld,%ld) samples=%ld/%ld\r\n",
            (long)s_ax0, (long)s_ay0, (long)s_az0, (long)s_gx0, (long)s_gy0, (long)s_gz0,
            (long)s_cal_ca, (long)s_cal_cd);
    tracking_start();
}

void Imu_Calibrate(void)
{
    if(!s_ready) { return; }
    RTT_LOG("[IMU] calibrating - keep the board STILL for %u ms\r\n", (unsigned)(IMU_CAL_SAMPLES * IMU_DT_MS));
    s_cal_index = 0U;
    s_cal_sa = s_cal_sb = s_cal_sc = s_cal_sd = s_cal_se = s_cal_sf = 0;
    s_cal_ca = s_cal_cd = 0;
    s_cal_start_ms = Uart_GetMs();
    s_cal_last_ms = s_cal_start_ms;
    s_tracking = 0U;
    s_cal_state = IMU_CAL_RUNNING;
}

static void imu_cal_step(void)
{
    uint8_t raw[14];
    uint32_t now = Uart_GetMs();

    if((now - s_cal_start_ms) >= IMU_CAL_TIMEOUT_MS)
    {
        RTT_LOG("[IMU_ERR] calibration timeout after %u/%u samples\r\n",
                (unsigned)s_cal_index, (unsigned)IMU_CAL_SAMPLES);
        imu_cal_finish();
        s_cal_state = IMU_CAL_TIMED_OUT;
        return;
    }
    if((now - s_cal_last_ms) < IMU_DT_MS) { return; }
    s_cal_last_ms = now;

    if(prv_Rd(ICM_REG_TEMP_DATA1, raw, 14U) == 0)
    {
        int16_t ax = prv_S16(raw[2], raw[3]), ay = prv_S16(raw[4], raw[5]), az = prv_S16(raw[6], raw[7]);
        int16_t gx = prv_S16(raw[8], raw[9]), gy = prv_S16(raw[10], raw[11]), gz = prv_S16(raw[12], raw[13]);
        if(ax != (int16_t)0x8000) { s_cal_sa += ax; s_cal_sb += ay; s_cal_sc += az; s_cal_ca++; }
        if((gx > -IMU_GYRO_ZRO_THR) && (gx < IMU_GYRO_ZRO_THR) &&
           (gy > -IMU_GYRO_ZRO_THR) && (gy < IMU_GYRO_ZRO_THR) &&
           (gz > -IMU_GYRO_ZRO_THR) && (gz < IMU_GYRO_ZRO_THR))
        {
            s_cal_sd += gx; s_cal_se += gy; s_cal_sf += gz; s_cal_cd++;
        }
    }
    s_cal_index++;
    if(s_cal_index >= IMU_CAL_SAMPLES)
    {
        imu_cal_finish();
        s_cal_state = IMU_CAL_DONE;
    }
}

/* ------------------------------------------------------------------ task (every IMU_DT_MS) */
void Imu_Task(void)
{
    uint8_t raw[14];
    int16_t tp, ax, ay, az, gx, gy, gz;
    float fax, fay, faz, wx, wy, wz, dt, anorm, wnorm;
    float lx, ly, lz;
    uint32_t now;

    if(!s_ready) { return; }
    if(s_cal_state == IMU_CAL_RUNNING) { imu_cal_step(); return; }
    if(prv_Rd(ICM_REG_TEMP_DATA1, raw, 14U) != 0) { return; }

    tp = prv_S16(raw[0], raw[1]);
    ax = prv_S16(raw[2], raw[3]);   ay = prv_S16(raw[4], raw[5]);   az = prv_S16(raw[6], raw[7]);
    gx = prv_S16(raw[8], raw[9]);   gy = prv_S16(raw[10], raw[11]); gz = prv_S16(raw[12], raw[13]);

    /* raw values for the existing packet fields (accel relative to rest, as before) */
    s_ax_mg = (((int32_t)ax - s_ax0) * 1000L) / 16384L;
    s_ay_mg = (((int32_t)ay - s_ay0) * 1000L) / 16384L;
    s_az_mg = (((int32_t)az - s_az0) * 1000L) / 16384L;
    s_gx_mdps = (((int32_t)gx - s_gx0) * 1000L) / 131L;
    s_gy_mdps = (((int32_t)gy - s_gy0) * 1000L) / 131L;
    s_gz_mdps = (((int32_t)gz - s_gz0) * 1000L) / 131L;
    s_temp_c10 = (int16_t)(((int32_t)tp * 10L) / 128L + 250L);
    s_n++;

    if(s_tracking == 0U) { return; }

    now = Uart_GetMs();
    dt = (float)(now - s_last_ms) * 0.001f;
    s_last_ms = now;
    if((dt <= 0.0f) || (dt > 0.1f)) { dt = (float)IMU_DT_MS * 0.001f; }

    fax = (float)ax / 16384.0f;  fay = (float)ay / 16384.0f;  faz = (float)az / 16384.0f;   /* g */
    wx = (float)((int32_t)gx - s_gx0) / 131.0f * DEG2RAD;                                   /* rad/s */
    wy = (float)((int32_t)gy - s_gy0) / 131.0f * DEG2RAD;
    wz = (float)((int32_t)gz - s_gz0) / 131.0f * DEG2RAD;
    anorm = m_sqrt(fax * fax + fay * fay + faz * faz);
    wnorm = m_sqrt(wx * wx + wy * wy + wz * wz) * RAD2DEG;

    /* stillness */
    if((m_abs(anorm - s_gmag) < STILL_ACC_G) && (wnorm < STILL_GYR_DPS))
    {
        if(s_still_cnt < 255U) { s_still_cnt++; }
    }
    else
    {
        s_still_cnt = 0U;
    }

    /* tilt correction while still: pull the estimated gravity toward the measured one */
    if((s_still_cnt >= STILL_SAMPLES) && (anorm > 0.5f))
    {
        float gx_b, gy_b, gz_b;     /* predicted gravity direction in body frame */
        float q0 = s_q0, q1 = s_q1, q2 = s_q2, q3 = s_q3;
        gx_b = 2.0f * (q1 * q3 - q0 * q2);
        gy_b = 2.0f * (q0 * q1 + q2 * q3);
        gz_b = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        wx += TILT_GAIN / dt * ((fay / anorm) * gz_b - (faz / anorm) * gy_b);
        wy += TILT_GAIN / dt * ((faz / anorm) * gx_b - (fax / anorm) * gz_b);
        wz += TILT_GAIN / dt * ((fax / anorm) * gy_b - (fay / anorm) * gx_b);
    }

    /* orientation update */
    {
        float hx = 0.5f * wx * dt, hy = 0.5f * wy * dt, hz = 0.5f * wz * dt;
        float q0 = s_q0, q1 = s_q1, q2 = s_q2, q3 = s_q3;
        s_q0 = q0 - q1 * hx - q2 * hy - q3 * hz;
        s_q1 = q1 + q0 * hx + q2 * hz - q3 * hy;
        s_q2 = q2 + q0 * hy - q1 * hz + q3 * hx;
        s_q3 = q3 + q0 * hz + q1 * hy - q2 * hx;
        q_normalize();
    }

    /* linear acceleration in the start frame, mm/s^2 */
    q_rotate(fax, fay, faz, &lx, &ly, &lz);
    lx *= G_MM_S2;
    ly *= G_MM_S2;
    lz = (lz - s_gmag) * G_MM_S2;
    if(m_abs(lx) < ACC_DEADBAND_MMS2) { lx = 0.0f; }
    if(m_abs(ly) < ACC_DEADBAND_MMS2) { ly = 0.0f; }
    if(m_abs(lz) < ACC_DEADBAND_MMS2) { lz = 0.0f; }

    if(s_still_cnt >= STILL_SAMPLES)
    {
        /* zero-velocity update: position is held, not reset */
        s_vx = s_vy = s_vz = 0.0f;
        if(s_moving != 0U)
        {
            s_moving = 0U;
            RTT_LOG("[IMU] stopped  pos=(%ld,%ld,%ld) mm  dist=%ld mm\r\n",
                    (long)s_px, (long)s_py, (long)s_pz, (long)s_dist);
        }
    }
    else
    {
        float sx, sy, sz;
        if(s_moving == 0U)
        {
            s_moving = 1U;
            RTT_LOG("[IMU] moving\r\n");
        }
        s_vx = (s_vx + lx * dt) * V_LEAK;
        s_vy = (s_vy + ly * dt) * V_LEAK;
        s_vz = (s_vz + lz * dt) * V_LEAK;
        sx = s_vx * dt; sy = s_vy * dt; sz = s_vz * dt;
        s_px += sx; s_py += sy; s_pz += sz;
        s_dist += m_sqrt(sx * sx + sy * sy + sz * sz);
    }

    {
        static uint32_t s_last_print_ms = 0U;
        if((now - s_last_print_ms) >= 1000U)
        {
            ImuPkt_t p;
            s_last_print_ms = now;
            Imu_GetLastPkt(&p);
            RTT_LOG("[IMU] t=%lus pos_mm=(%ld,%ld,%ld) dist_mm=%lu speed=%ldmm/s rpy=(%d,%d,%d)deg %s T=%d.%dC\r\n",
                    (unsigned long)(p.imu_up_ms / 1000U),
                    (long)(p.px_mm10 / 10), (long)(p.py_mm10 / 10), (long)(p.pz_mm10 / 10),
                    (unsigned long)(p.dist_mm10 / 10U), (long)p.speed_mms,
                    (int)(p.roll_cd / 100), (int)(p.pitch_cd / 100), (int)(p.yaw_cd / 100),
                    p.moving ? "MOVING" : "still", (int)(s_temp_c10 / 10), (int)(s_temp_c10 % 10));
        }
    }
}

void Imu_GetLastPkt(ImuPkt_t *out)
{
    float roll, pitch, yaw, sp;
    if(!out) { return; }
    out->ax_mg = s_ax_mg;
    out->ay_mg = s_ay_mg;
    out->az_mg = s_az_mg;
    out->gx_mdps = s_gx_mdps;
    out->gy_mdps = s_gy_mdps;
    out->gz_mdps = s_gz_mdps;
    out->temp_c10 = s_temp_c10;
    out->ts_ms = Uart_GetMs();

    roll  = m_atan2(2.0f * (s_q0 * s_q1 + s_q2 * s_q3), 1.0f - 2.0f * (s_q1 * s_q1 + s_q2 * s_q2));
    sp    = 2.0f * (s_q0 * s_q2 - s_q3 * s_q1);
    if(sp > 1.0f) { sp = 1.0f; } else if(sp < -1.0f) { sp = -1.0f; }
    pitch = m_asin(sp);
    yaw   = m_atan2(2.0f * (s_q0 * s_q3 + s_q1 * s_q2), 1.0f - 2.0f * (s_q2 * s_q2 + s_q3 * s_q3));

    out->px_mm10   = (int32_t)(s_px * 10.0f);
    out->py_mm10   = (int32_t)(s_py * 10.0f);
    out->pz_mm10   = (int32_t)(s_pz * 10.0f);
    out->dist_mm10 = (uint32_t)(s_dist * 10.0f);
    out->roll_cd   = (int16_t)(roll * RAD2DEG * 100.0f);
    out->pitch_cd  = (int16_t)(pitch * RAD2DEG * 100.0f);
    out->yaw_cd    = (int16_t)(yaw * RAD2DEG * 100.0f);
    out->moving    = s_moving;
    out->flags     = (uint8_t)(((s_cal_state == IMU_CAL_DONE) || (s_cal_state == IMU_CAL_TIMED_OUT)) ? 1U : 0U)
                   | (uint8_t)(s_tracking ? 2U : 0U);
    out->imu_up_ms = s_tracking ? (Uart_GetMs() - s_track_start_ms) : 0U;
    out->speed_mms = (int32_t)m_sqrt(s_vx * s_vx + s_vy * s_vy + s_vz * s_vz);
}

uint8_t Imu_IsReady(void)
{
    return s_ready;
}
