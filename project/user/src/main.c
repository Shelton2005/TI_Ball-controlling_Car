#include "zf_common_headfile.h"

#define SERVO_TEST_MODE             0

#if !SERVO_TEST_MODE
#include "Encoder.h"
#include "Menu.h"
#include "Question_3.h"
#include "Question_4.h"
#include "Question5.h"

#define GRAY_SENSOR_TEST_MODE       1

#define LAP_MIN_RUN_TIME_MS          5000
#define LAP_YAW_ARM_DEG              330.0f
#define LAP_STRAIGHT_YAW_STEP_DEG    0.06f
#define LAP_STRAIGHT_CONFIRM_CYCLES  20
#define LAP_STOP_YAW_DEG             (-350.0f)

#define EVENT_POWER_ON               0
#define EVENT_KEY_START              1
#define EVENT_KEY_STOP               4
#define EVENT_LAP_DONE               5
#define EVENT_ENCODER_ERROR          6
#define EVENT_GRAY_ERROR             7
#define EVENT_TASK_NOT_READY          8
#define EVENT_LINE_LOST               9

volatile uint32 pit_ms_ticks = 0;

void pit_handler(uint32 state, void *ptr);
#endif

int main(void)
{
#if SERVO_TEST_MODE
    /*
     * 舵机专用测试模式：
     * 只初始化 80 MHz 系统时钟、四个按键、IPS200 和 B4 舵机。
     * 电机、编码器、灰度传感器、IMU、菜单及各赛题均不初始化。
     */
    clock_init(SYSTEM_CLOCK_80M);

    Key_init();
    key_scan();
    key1_flag = 0;
    key2_flag = 0;
    key3_flag = 0;
    key4_flag = 0;

    /* 主板屏幕接口的 A13 为背光控制脚，驱动库默认未使能。 */
    gpio_init(A13, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    ips200_init(IPS200_TYPE_SPI);
    Steer_init();

    while(true)
    {
        Steer_test();
        system_delay_ms(10);
    }
#else
    uint8 tracking_enabled = 0;
    uint32 system_time_count = 0;
    uint32 last_key_time_ms = 0;
    uint32 last_imu_time_ms = 0;
    uint32 last_encoder_time_ms = 0;
    uint32 last_display_time_ms = 0;
    uint32 tracking_time_ms = 0;
    uint16 finish_straight_count = 0;
    uint8 control_event = EVENT_POWER_ON;
    menu_question_t active_question = MENU_QUESTION_2;
    float last_yaw = 0.0f;
    float start_yaw = 0.0f;

    clock_init(SYSTEM_CLOCK_80M);
    debug_init();

    /* Keep the buzzer pin driven low; Buzzer_check() is globally muted. */
    Buzzer_init();
    Key_init();
    key_scan();
    key1_flag = 0;
    key2_flag = 0;
    key3_flag = 0;
    key4_flag = 0;

    /* 正常循迹模式同样需要显式打开 IPS200 背光。 */
    gpio_init(A13, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    ips200_init(IPS200_TYPE_SPI);
    Steer_init();

#if GRAY_SENSOR_TEST_MODE
    Gray_sensor_test();
#endif

    Gray_sensor_init();
    TB6612_init();
    Encoder_init();

    imu660ra_init();
    Imu_zero_drift_reset();
    Imu_open_flag = 1;

    pit_ms_init(PIT_TIM_A0, 1, pit_handler, NULL);
    interrupt_set_priority(TIMA0_INT_IRQn, 0);

    Line_follow_reset();
    Menu_Init();
    Menu_Display(0);

    while(true)
    {
        uint32 now_ms = pit_ms_ticks;

        if(now_ms != system_time_count)
        {
            system_time_count = now_ms;

            if(system_time_count - last_key_time_ms >= 10)
            {
                last_key_time_ms = system_time_count;
                key_scan();
                Menu_ProcessKeys(system_time_count);

                if(Menu_TakeStopRequest())
                {
                    tracking_enabled = 0;
                    tracking_time_ms = 0;
                    finish_straight_count = 0;
                    Question_3_Stop();
                    Question_4_Stop();
                    Question5_Stop();
                    control_event = EVENT_KEY_STOP;
                }

                if(Menu_TakeStartRequest())
                {
                    active_question = Menu_GetSelectedQuestion();
                    tracking_enabled = 0;
                    tracking_time_ms = 0;
                    finish_straight_count = 0;
                    Question_3_Stop();
                    Question_4_Stop();
                    Question5_Stop();

                    if(active_question == MENU_QUESTION_3)
                    {
                        Question_3_Start(system_time_count);
                        tracking_enabled = 1;
                        control_event = EVENT_KEY_START;
                    }
                    else if(active_question == MENU_QUESTION_2
                        || active_question == MENU_QUESTION_4
                        || active_question == MENU_QUESTION_5)
                    {
                        if(gray_iic_ok)
                        {
                            if(active_question == MENU_QUESTION_2)
                            {
                                start_yaw = YAW.YAW_1;
                                last_yaw = YAW.YAW_1;
                                Line_follow_reset();
                            }
                            else if(active_question == MENU_QUESTION_4)
                            {
                                Question_4_Start(system_time_count);
                            }
                            else
                            {
                                Question5_Start();
                            }

                            tracking_enabled = 1;
                            control_event = EVENT_KEY_START;
                        }
                        else
                        {
                            control_event = EVENT_GRAY_ERROR;
                            Menu_SetResult(
                                MENU_RESULT_GRAY_ERROR,
                                system_time_count);
                        }
                    }
                    else
                    {
                        /*
                         * Question 6 still needs the camera ball-position
                         * controller. Keep its menu entry visible, but never
                         * start a motor task before that module is ready.
                         */
                        control_event = EVENT_TASK_NOT_READY;
                        Menu_SetResult(
                            MENU_RESULT_NOT_READY,
                            system_time_count);
                    }
                }
            }

            if(system_time_count - last_imu_time_ms >= 5)
            {
                uint32 imu_elapsed_ms =
                    system_time_count - last_imu_time_ms;
                float yaw_step;
                float lap_yaw;
                float straight_yaw_step_limit;

                last_imu_time_ms = system_time_count;
                imu660ra_get_gyro();
                Imu_value_get_dt((float)imu_elapsed_ms * 0.001f);

                if(tracking_enabled
                    && active_question == MENU_QUESTION_2)
                {
                    if(YAW.YAW_1 - start_yaw <= LAP_STOP_YAW_DEG)
                    {
                        tracking_enabled = 0;
                        tracking_time_ms = 0;
                        finish_straight_count = 0;
                        Line_follow_reset();
                        control_event = EVENT_LAP_DONE;
                        Menu_SetResult(
                            MENU_RESULT_DONE,
                            system_time_count);
                    }
                    else
                    {
                        tracking_time_ms += imu_elapsed_ms;
                        G_D_navigation();

                        if(!gray_iic_ok)
                        {
                            tracking_enabled = 0;
                            Line_follow_reset();
                            control_event = EVENT_GRAY_ERROR;
                            Menu_SetResult(
                                MENU_RESULT_GRAY_ERROR,
                                system_time_count);
                        }
                        else
                        {
                            yaw_step = YAW.YAW_1 - last_yaw;
                            last_yaw = YAW.YAW_1;
                            lap_yaw = YAW.YAW_1 - start_yaw;
                            straight_yaw_step_limit =
                                LAP_STRAIGHT_YAW_STEP_DEG
                                * (float)imu_elapsed_ms / 5.0f;

                            /*
                             * A is reached after one full turn and the car
                             * settles onto the straight.
                             */
                            if(tracking_time_ms >= LAP_MIN_RUN_TIME_MS
                                && (lap_yaw >= LAP_YAW_ARM_DEG
                                    || lap_yaw <= -LAP_YAW_ARM_DEG)
                                && yaw_step <= straight_yaw_step_limit
                                && yaw_step >= -straight_yaw_step_limit
                                && (g_line_course_point == LINE_POINT_A
                                    || g_line_course_point == LINE_POINT_B)
                                && g_line_state == LINE_STATE_TRACKING)
                            {
                                finish_straight_count++;
                            }
                            else
                            {
                                finish_straight_count = 0;
                            }

                            if(finish_straight_count
                                >= LAP_STRAIGHT_CONFIRM_CYCLES)
                            {
                                tracking_enabled = 0;
                                Line_follow_reset();
                                control_event = EVENT_LAP_DONE;
                                Menu_SetResult(
                                    MENU_RESULT_DONE,
                                    system_time_count);
                            }
                        }
                    }
                }

                if(tracking_enabled
                    && active_question == MENU_QUESTION_3)
                {
                    Question_3_Control_5ms(system_time_count);

                    if(!Question_3_IsRunning())
                    {
                        tracking_enabled = 0;

                        if(g_question_3_state
                            == QUESTION_3_STATE_DONE)
                        {
                            control_event = EVENT_LAP_DONE;
                            Menu_SetResult(
                                MENU_RESULT_DONE,
                                system_time_count);
                        }
                    }
                }

                if(tracking_enabled
                    && active_question == MENU_QUESTION_4)
                {
                    Question_4_Control_5ms(system_time_count);

                    if(!Question_4_IsRunning())
                    {
                        tracking_enabled = 0;

                        if(g_question_4_state
                            == QUESTION_4_STATE_DONE)
                        {
                            control_event = EVENT_LAP_DONE;
                            Menu_SetResult(
                                MENU_RESULT_DONE,
                                system_time_count);
                        }
                        else if(g_question_4_state
                            == QUESTION_4_STATE_LINE_LOST)
                        {
                            control_event = EVENT_LINE_LOST;
                            Menu_SetResult(
                                MENU_RESULT_LINE_LOST,
                                system_time_count);
                        }
                        else if(g_question_4_state
                            == QUESTION_4_STATE_GRAY_ERROR)
                        {
                            control_event = EVENT_GRAY_ERROR;
                            Menu_SetResult(
                                MENU_RESULT_GRAY_ERROR,
                                system_time_count);
                        }
                    }
                }

                if(tracking_enabled
                    && active_question == MENU_QUESTION_5)
                {
                    Question5_Control_5ms(imu_elapsed_ms);

                    if(!Question5_IsRunning())
                    {
                        tracking_enabled = 0;

                        if(g_question5_state == QUESTION5_STATE_DONE)
                        {
                            control_event = EVENT_LAP_DONE;
                            Menu_SetResult(
                                MENU_RESULT_DONE,
                                system_time_count);
                        }
                        else if(g_question5_state
                            == QUESTION5_STATE_GRAY_ERROR)
                        {
                            control_event = EVENT_GRAY_ERROR;
                            Menu_SetResult(
                                MENU_RESULT_GRAY_ERROR,
                                system_time_count);
                        }
                    }
                }

            }

            if(system_time_count - last_encoder_time_ms >= 10)
            {
                last_encoder_time_ms = system_time_count;
                Encoder_sample_10ms();

                if(tracking_enabled
                    && active_question == MENU_QUESTION_2)
                {
                    Line_speed_control_10ms();

                    if(g_encoder_fault)
                    {
                        tracking_enabled = 0;
                        Line_follow_reset();
                        control_event = EVENT_ENCODER_ERROR;
                        Menu_SetResult(
                            MENU_RESULT_ENCODER_ERROR,
                            system_time_count);
                    }
                }
                else if(tracking_enabled
                    && active_question == MENU_QUESTION_4)
                {
                    Question_4_Control_10ms();

                    if(g_question_4_state
                        == QUESTION_4_STATE_ENCODER_ERROR)
                    {
                        tracking_enabled = 0;
                        control_event = EVENT_ENCODER_ERROR;
                        Menu_SetResult(
                            MENU_RESULT_ENCODER_ERROR,
                            system_time_count);
                    }
                }
                else if(tracking_enabled
                    && active_question == MENU_QUESTION_5)
                {
                    Question5_Control_10ms();

                    if(g_question5_state
                        == QUESTION5_STATE_ENCODER_ERROR)
                    {
                        tracking_enabled = 0;
                        control_event = EVENT_ENCODER_ERROR;
                        Menu_SetResult(
                            MENU_RESULT_ENCODER_ERROR,
                            system_time_count);
                    }
                }
            }

            if(system_time_count - last_display_time_ms >= 100)
            {
                last_display_time_ms = system_time_count;
                Menu_Display(system_time_count);

                if(Menu_GetState() == MENU_STATE_RUNNING
                    && (active_question == MENU_QUESTION_2
                        || active_question == MENU_QUESTION_5))
                {
                    ips200_show_string(0, 16 * 6, "YAW:");
                    ips200_show_float(
                        8 * 5, 16 * 6, YAW.YAW_1, 5, 2);
                    ips200_show_string(0, 16 * 8, "ENC L:");
                    ips200_show_int(
                        8 * 7, 16 * 8, g_encoder_left_speed, 5);
                    ips200_show_string(0, 16 * 9, "ENC R:");
                    ips200_show_int(
                        8 * 7, 16 * 9, g_encoder_right_speed, 5);
                    ips200_show_string(
                        0, 16 * 10, "POINT 0B1C2D3A:");
                    ips200_show_int(
                        8 * 15, 16 * 10, g_line_course_point, 1);
                    ips200_show_string(0, 16 * 11, "LSTATE:");
                    ips200_show_int(
                        8 * 8, 16 * 11, g_line_state, 1);
                    ips200_show_string(0, 16 * 12, "RAW:    ACT:");
                    ips200_show_uint(
                        8 * 4, 16 * 12, gray_iic_raw, 3);
                    ips200_show_int(
                        8 * 11, 16 * 12, g_line_active_count, 1);
                    ips200_show_string(
                        0, 16 * 13, "LOST:   ARM:  GAP:");
                    ips200_show_int(
                        8 * 5, 16 * 13, g_line_lost_count, 3);
                    ips200_show_int(
                        8 * 12, 16 * 13, g_line_point_armed, 1);
                    ips200_show_int(
                        8 * 18, 16 * 13, g_line_gap_active, 1);
                    ips200_show_string(
                        0, 16 * 14, "LEVEL:  CAL:");
                    ips200_show_int(
                        8 * 6, 16 * 14, g_line_active_level, 1);
                    ips200_show_int(
                        8 * 12, 16 * 14, g_line_level_calibrated, 1);
                    ips200_show_string(0, 16 * 15, "EVENT:");
                    ips200_show_int(
                        8 * 7, 16 * 15, control_event, 1);
                    ips200_show_string(0, 16 * 16, "IMU DT:");
                    ips200_show_int(
                        8 * 8, 16 * 16, g_imu_sample_period_ms, 3);
                }
                else if(Menu_GetState() == MENU_STATE_RUNNING
                    && active_question == MENU_QUESTION_3)
                {
                    ips200_show_string(0, 16 * 6, "Q3 PHASE:");
                    ips200_show_int(
                        8 * 10,
                        16 * 6,
                        g_question_3_phase,
                        1);
                    ips200_show_string(0, 16 * 8, "SERVO:");
                    ips200_show_int(
                        8 * 7,
                        16 * 8,
                        g_question_3_servo_angle,
                        3);
                    ips200_show_string(0, 16 * 10, "Q3 MS:");
                    ips200_show_uint(
                        8 * 7,
                        16 * 10,
                        g_question_3_elapsed_ms,
                        4);
                }
                else if(Menu_GetState() == MENU_STATE_RUNNING
                    && active_question == MENU_QUESTION_4)
                {
                    ips200_show_string(0, 16 * 6, "Q4 YAW D:");
                    ips200_show_int(
                        8 * 10,
                        16 * 6,
                        g_question_4_yaw_delta_deg,
                        4);
                    ips200_show_string(0, 16 * 8, "LINE ERR:");
                    ips200_show_int(
                        8 * 10, 16 * 8, g_line_error, 5);
                    ips200_show_string(0, 16 * 9, "DIST MM:");
                    ips200_show_uint(
                        8 * 9,
                        16 * 9,
                        g_question_4_distance_mm,
                        4);
                    ips200_show_string(0, 16 * 10, "TARGET L:");
                    ips200_show_int(
                        8 * 10, 16 * 10, g_left_speed_target, 5);
                    ips200_show_string(0, 16 * 11, "TARGET R:");
                    ips200_show_int(
                        8 * 10, 16 * 11, g_right_speed_target, 5);
                    ips200_show_string(0, 16 * 12, "ENC L:");
                    ips200_show_int(
                        8 * 7, 16 * 12, g_encoder_left_speed, 5);
                    ips200_show_string(0, 16 * 13, "ENC R:");
                    ips200_show_int(
                        8 * 7, 16 * 13, g_encoder_right_speed, 5);
                    ips200_show_string(0, 16 * 14, "RAMP:    PH:");
                    ips200_show_int(
                        8 * 6,
                        16 * 14,
                        g_question_4_speed_limit,
                        4);
                    ips200_show_int(
                        8 * 14,
                        16 * 14,
                        g_question_4_motion_phase,
                        1);
                    ips200_show_string(0, 16 * 15, "LINE LOST:");
                    ips200_show_int(
                        8 * 11,
                        16 * 15,
                        g_line_lost_count,
                        3);
                    ips200_show_string(0, 16 * 16, "Q4 STATE:");
                    ips200_show_int(
                        8 * 10,
                        16 * 16,
                        g_question_4_state,
                        1);
                }
            }
        }
    }
#endif
}

#if !SERVO_TEST_MODE
void pit_handler(uint32 state, void *ptr)
{
    (void)state;
    (void)ptr;
    pit_ms_ticks++;
}
#endif
