#ifndef _imu_h_
#define _imu_h_

typedef struct
{
    float YAW_1;
    float YAW_2;
    float YAW_3;

    float Ptich_1;
    float Ptich_2;
    float Ptich_3;

    float Roll_1;
    float Roll_2;
    float Roll_3;
}Angle_type;

typedef struct
{
    unsigned int sample_count;
    unsigned char finished;

    float sum_x_raw;
    float sum_y_raw;
    float sum_z_raw;

    float sum_x_dps;
    float sum_y_dps;
    float sum_z_dps;

    float avg_x_raw;
    float avg_y_raw;
    float avg_z_raw;

    float avg_x_dps;
    float avg_y_dps;
    float avg_z_dps;
    float z_drift_deg;
}Gyro_zero_drift_type;

extern Angle_type YAW;
extern int Imu_open_flag;
extern int g_imu_sample_period_ms;
extern Gyro_zero_drift_type Gyro_zero_drift;

void Imu_text(void);
void Imu_value_get(void);
void Imu_value_get_dt(float sample_period_s);
void Imu_zero_drift_reset(void);
void Imu_zero_drift_test(void);

#endif
