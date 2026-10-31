#include "zf_common_headfile.h"

#define IMU_ZERO_DRIFT_SAMPLE_NUM       2000
#define IMU_SAMPLE_PERIOD_S             0.005f
#define IMU_GYRO_Z_ZERO_DPS             (-0.01506f)
// 实测转动 -90° 时积分约为 -84°：90 / 84 = 1.0714286
#define IMU_GYRO_Z_SCALE                1.0714286f

Angle_type YAW;
Gyro_zero_drift_type Gyro_zero_drift;

int Imu_open_flag = 0;
int g_imu_sample_period_ms = 5;

static float gyro_zero_z_dps = IMU_GYRO_Z_ZERO_DPS;
static float last_gyro_z_dps = 0.0f;
static uint8 have_last_gyro_z = 0;

static int float_to_milli_int(float value)
{
    if(value >= 0.0f)
    {
        return (int)(value * 1000.0f + 0.5f);
    }

    return (int)(value * 1000.0f - 0.5f);
}

void Imu_text(void)
{
    Imu_open_flag = 1;

    ips200_show_string(0, 0, "YAW:");
    ips200_show_float(8 * 5, 0, YAW.YAW_1, 5, 5);

    printf("%d,%d,%d\r\n", imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z);
}

void Imu_value_get(void)
{
    Imu_value_get_dt(IMU_SAMPLE_PERIOD_S);
}

void Imu_value_get_dt(float sample_period_s)
{
    float gyro_z_dps;

    g_imu_sample_period_ms =
        (int)(sample_period_s * 1000.0f + 0.5f);

    if(Imu_open_flag == 1)
    {
        gyro_z_dps = imu660ra_gyro_transition(imu660ra_gyro_z);

        if(!have_last_gyro_z)
        {
            last_gyro_z_dps = gyro_z_dps;
            have_last_gyro_z = 1;
        }

        YAW.YAW_1 += (
            (last_gyro_z_dps + gyro_z_dps) * 0.5f
            - gyro_zero_z_dps)
            * IMU_GYRO_Z_SCALE
            * sample_period_s;
        last_gyro_z_dps = gyro_z_dps;
    }
}

void Imu_zero_drift_reset(void)
{
    Gyro_zero_drift.sample_count = 0;
    Gyro_zero_drift.finished = 0;

    Gyro_zero_drift.sum_x_raw = 0.0f;
    Gyro_zero_drift.sum_y_raw = 0.0f;
    Gyro_zero_drift.sum_z_raw = 0.0f;

    Gyro_zero_drift.sum_x_dps = 0.0f;
    Gyro_zero_drift.sum_y_dps = 0.0f;
    Gyro_zero_drift.sum_z_dps = 0.0f;

    Gyro_zero_drift.avg_x_raw = 0.0f;
    Gyro_zero_drift.avg_y_raw = 0.0f;
    Gyro_zero_drift.avg_z_raw = 0.0f;

    Gyro_zero_drift.avg_x_dps = 0.0f;
    Gyro_zero_drift.avg_y_dps = 0.0f;
    Gyro_zero_drift.avg_z_dps = 0.0f;
    Gyro_zero_drift.z_drift_deg = 0.0f;

    gyro_zero_z_dps = IMU_GYRO_Z_ZERO_DPS;
    last_gyro_z_dps = 0.0f;
    have_last_gyro_z = 0;
    YAW.YAW_1 = 0.0f;
}

void Imu_zero_drift_test(void)
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    Imu_open_flag = 0;

    if(Gyro_zero_drift.finished)
    {
        ips200_show_string(0, 16 * 0, "GYRO ZERO OK");
        ips200_show_string(0, 16 * 2, "Z dps:");
        ips200_show_float(8 * 7, 16 * 2, Gyro_zero_drift.avg_z_dps, 3, 5);
        ips200_show_string(0, 16 * 3, "10s deg:");
        ips200_show_float(8 * 8, 16 * 3, Gyro_zero_drift.z_drift_deg, 3, 4);
        return;
    }

    gyro_x_dps = imu660ra_gyro_transition(imu660ra_gyro_x);
    gyro_y_dps = imu660ra_gyro_transition(imu660ra_gyro_y);
    gyro_z_dps = imu660ra_gyro_transition(imu660ra_gyro_z);

    Gyro_zero_drift.sum_x_raw += imu660ra_gyro_x;
    Gyro_zero_drift.sum_y_raw += imu660ra_gyro_y;
    Gyro_zero_drift.sum_z_raw += imu660ra_gyro_z;

    Gyro_zero_drift.sum_x_dps += gyro_x_dps;
    Gyro_zero_drift.sum_y_dps += gyro_y_dps;
    Gyro_zero_drift.sum_z_dps += gyro_z_dps;
    Gyro_zero_drift.sample_count++;

    ips200_show_string(0, 16 * 0, "GYRO ZERO TEST");
    ips200_show_string(0, 16 * 2, "KEEP STILL");
    ips200_show_string(0, 16 * 4, "N:");
    ips200_show_int(8 * 3, 16 * 4, Gyro_zero_drift.sample_count, 4);
    ips200_show_string(8 * 8, 16 * 4, "/2000");
    ips200_show_string(0, 16 * 6, "GZ:");
    ips200_show_int(8 * 4, 16 * 6, imu660ra_gyro_z, 6);

    if(Gyro_zero_drift.sample_count >= IMU_ZERO_DRIFT_SAMPLE_NUM)
    {
        Gyro_zero_drift.avg_x_raw = Gyro_zero_drift.sum_x_raw / Gyro_zero_drift.sample_count;
        Gyro_zero_drift.avg_y_raw = Gyro_zero_drift.sum_y_raw / Gyro_zero_drift.sample_count;
        Gyro_zero_drift.avg_z_raw = Gyro_zero_drift.sum_z_raw / Gyro_zero_drift.sample_count;

        Gyro_zero_drift.avg_x_dps = Gyro_zero_drift.sum_x_dps / Gyro_zero_drift.sample_count;
        Gyro_zero_drift.avg_y_dps = Gyro_zero_drift.sum_y_dps / Gyro_zero_drift.sample_count;
        Gyro_zero_drift.avg_z_dps = Gyro_zero_drift.sum_z_dps / Gyro_zero_drift.sample_count;
        Gyro_zero_drift.z_drift_deg = Gyro_zero_drift.avg_z_dps * IMU_ZERO_DRIFT_SAMPLE_NUM * IMU_SAMPLE_PERIOD_S;

        gyro_zero_z_dps = Gyro_zero_drift.avg_z_dps;
        last_gyro_z_dps = 0.0f;
        have_last_gyro_z = 0;
        YAW.YAW_1 = 0.0f;
        Gyro_zero_drift.finished = 1;

        printf("gyro_zero_raw,%d,%d,%d\r\n",
               (int)Gyro_zero_drift.avg_x_raw,
               (int)Gyro_zero_drift.avg_y_raw,
               (int)Gyro_zero_drift.avg_z_raw);
        printf("gyro_zero_mdps,%d,%d,%d\r\n",
               float_to_milli_int(Gyro_zero_drift.avg_x_dps),
               float_to_milli_int(Gyro_zero_drift.avg_y_dps),
               float_to_milli_int(Gyro_zero_drift.avg_z_dps));
        printf("gyro_z_10s_mdeg,%d\r\n", float_to_milli_int(Gyro_zero_drift.z_drift_deg));
    }
}
