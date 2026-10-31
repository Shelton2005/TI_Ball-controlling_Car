#include "zf_common_headfile.h"
#include "Encoder.h"
#include "Questoin_2.h"
#include "Question5.h"

/*
 * Question 5 directly reuses the complete-lap controller from Question 2.
 * The ball is expected to be placed and mechanically held at the center.
 */

/*
 * Question 5 tuning block.
 * Change only these Q5_* values when tuning Question 5. Question 2 keeps
 * the independent LINE_* defaults in Questoin_2.c.
 */
#define Q5_CRUISE_SPEED              1800
#define Q5_TOP_STRAIGHT_SPEED        2700
#define Q5_CURVE_SPEED               1050
#define Q5_GAP_SPEED                 1100
#define Q5_BOTTOM_STRAIGHT_SPEED     2000
#define Q5_SEARCH_FORWARD_SPEED       950
#define Q5_SEARCH_TURN_SPEED          200
#define Q5_ARC_ENTRY_OUTER_SPEED     1600
#define Q5_ARC_ENTRY_INNER_SPEED     850
#define Q5_ARC_SEARCH_OUTER_SPEED    1450
#define Q5_ARC_SEARCH_INNER_SPEED    1000

#define Q5_MIN_FORWARD_SPEED          750
#define Q5_PWM_STEP_LIMIT             80
#define Q5_LAUNCH_SPEED_STEP           25
#define Q5_TURN_LIMIT                 600
#define Q5_STRAIGHT_KP_NUM             46
#define Q5_STRAIGHT_KD_NUM             75
#define Q5_CURVE_KP_NUM                55
#define Q5_CURVE_KD_NUM                25
#define Q5_SPEED_ERROR_DIV              8
#define Q5_CURVE_MIN_FORWARD_SPEED    1000

#define Q5_ENCODER_COMMAND_DIV         100
#define Q5_SPEED_KP                     45
#define Q5_SPEED_KI                      4
#define Q5_SPEED_INTEGRAL_MIN             0
#define Q5_SPEED_INTEGRAL_MAX           250
#define Q5_SPEED_MIN_PWM                650
#define Q5_CURVE_PWM_LIMIT             4500
#define Q5_STRAIGHT_PWM_LIMIT          5000

/*
 * Normal B-C and D-A curves use this fixed clockwise differential.
 * Only errors outside the threshold add a limited correction.
 */
#define Q5_CURVE_OPEN_LOOP_ENABLE          1
#define Q5_CURVE_OPEN_OUTER_SPEED       1800
#define Q5_CURVE_OPEN_INNER_SPEED       800
#define Q5_CURVE_ERROR_THRESHOLD        1400
#define Q5_CURVE_CORRECTION_KP_NUM        22
#define Q5_CURVE_CORRECTION_KI_NUM         0
#define Q5_CURVE_CORRECTION_KD_NUM         8
#define Q5_CURVE_CORRECTION_INTEGRAL_LIMIT 0
#define Q5_CURVE_CORRECTION_LIMIT         300

#define Q5_LAP_MIN_RUN_TIME_MS          5000U
#define Q5_LAP_YAW_ARM_DEG              330.0f
#define Q5_LAP_STRAIGHT_YAW_STEP_DEG    0.06f
#define Q5_LAP_STRAIGHT_CONFIRM_CYCLES  20U
#define Q5_LAP_STOP_YAW_DEG             (-355.0f)

#define Q5_SERVO_MID             (SERVO_MOTOR_MID + 3)

#if 0
/*
 * Encoder-acceleration servo compensation.
 * Positive chassis acceleration lowers the front end; braking raises it.
 * Speed and acceleration use Q8 fixed point to retain sub-count resolution
 * from the 10 ms encoder samples.
 */
#define Q5_SERVO_Q_SHIFT                  8
#define Q5_SERVO_FILTER_DIV               2
#define Q5_SERVO_ACCEL_KP_NUM           125
#define Q5_SERVO_JERK_KD_NUM             25
#define Q5_SERVO_GAIN_DEN               100
#define Q5_SERVO_ACCEL_DEADBAND_Q8      128
#define Q5_SERVO_MAX_OFFSET_DEG           6
#define Q5_SERVO_OUTPUT_PERIOD_CYCLES     2U
#define Q5_SERVO_OUTPUT_STEP_DEG          1
#endif

/*
 * Lightweight encoder-speed PD for the servo. This controller has no loops,
 * delays, display writes, or floating-point work in its 20 ms update path.
 */
#define Q5_SERVO_SAFE_Q_SHIFT               8
#define Q5_SERVO_SAFE_SPEED_FILTER_DIV      4
#define Q5_SERVO_SAFE_KP_NUM              130
#define Q5_SERVO_SAFE_KD_NUM               90
#define Q5_SERVO_SAFE_GAIN_DEN            100
#define Q5_SERVO_SAFE_DEADBAND_Q8          128
#define Q5_SERVO_SAFE_MAX_OFFSET_DEG         5
#define Q5_SERVO_SAFE_PERIOD_CYCLES          2U
#define Q5_SERVO_SAFE_STEP_DEG               1

static const line_follow_profile_t question5_line_profile =
{
    .cruise_speed = Q5_CRUISE_SPEED,
    .top_straight_speed = Q5_TOP_STRAIGHT_SPEED,
    .curve_speed = Q5_CURVE_SPEED,
    .gap_speed = Q5_GAP_SPEED,
    .bottom_straight_speed = Q5_BOTTOM_STRAIGHT_SPEED,
    .search_forward_speed = Q5_SEARCH_FORWARD_SPEED,
    .search_turn_speed = Q5_SEARCH_TURN_SPEED,
    .arc_entry_outer_speed = Q5_ARC_ENTRY_OUTER_SPEED,
    .arc_entry_inner_speed = Q5_ARC_ENTRY_INNER_SPEED,
    .arc_search_outer_speed = Q5_ARC_SEARCH_OUTER_SPEED,
    .arc_search_inner_speed = Q5_ARC_SEARCH_INNER_SPEED,
    .min_forward_speed = Q5_MIN_FORWARD_SPEED,
    .pwm_step_limit = Q5_PWM_STEP_LIMIT,
    .turn_limit = Q5_TURN_LIMIT,
    .straight_kp_num = Q5_STRAIGHT_KP_NUM,
    .straight_kd_num = Q5_STRAIGHT_KD_NUM,
    .curve_kp_num = Q5_CURVE_KP_NUM,
    .curve_kd_num = Q5_CURVE_KD_NUM,
    .speed_error_div = Q5_SPEED_ERROR_DIV,
    .curve_min_forward_speed = Q5_CURVE_MIN_FORWARD_SPEED,
    .encoder_command_div = Q5_ENCODER_COMMAND_DIV,
    .speed_kp = Q5_SPEED_KP,
    .speed_ki = Q5_SPEED_KI,
    .speed_integral_min = Q5_SPEED_INTEGRAL_MIN,
    .speed_integral_max = Q5_SPEED_INTEGRAL_MAX,
    .speed_min_pwm = Q5_SPEED_MIN_PWM,
    .curve_pwm_limit = Q5_CURVE_PWM_LIMIT,
    .straight_pwm_limit = Q5_STRAIGHT_PWM_LIMIT,
    .curve_open_loop_enabled = Q5_CURVE_OPEN_LOOP_ENABLE,
    .curve_open_outer_speed = Q5_CURVE_OPEN_OUTER_SPEED,
    .curve_open_inner_speed = Q5_CURVE_OPEN_INNER_SPEED,
    .curve_error_threshold = Q5_CURVE_ERROR_THRESHOLD,
    .curve_correction_kp_num = Q5_CURVE_CORRECTION_KP_NUM,
    .curve_correction_ki_num = Q5_CURVE_CORRECTION_KI_NUM,
    .curve_correction_kd_num = Q5_CURVE_CORRECTION_KD_NUM,
    .curve_correction_integral_limit =
        Q5_CURVE_CORRECTION_INTEGRAL_LIMIT,
    .curve_correction_limit = Q5_CURVE_CORRECTION_LIMIT
};

const line_follow_profile_t *Question5_GetLineProfile(void)
{
    return &question5_line_profile;
}

int g_question5_state = QUESTION5_STATE_IDLE;
/* int g_question5_servo_angle = Q5_SERVO_MID; */

static uint32 question5_tracking_time_ms = 0;
static uint16 question5_finish_straight_count = 0;
static int question5_launch_speed_limit = 0;
static int question5_brake_speed_limit = 0;
static uint8 question5_braking = 0;
#if 0
static int question5_servo_speed_q8 = 0;
static int question5_servo_last_speed_q8 = 0;
static int question5_servo_accel_q8 = 0;
static int question5_servo_last_accel_q8 = 0;
static uint8 question5_servo_output_count = 0;
#endif
static int question5_servo_safe_speed_q8 = 0;
static int question5_servo_safe_last_speed_q8 = 0;
static int question5_servo_safe_last_error_q8 = 0;
static int question5_servo_safe_angle = Q5_SERVO_MID;
static uint8 question5_servo_safe_count = 0;
static float question5_start_yaw = 0.0f;
static float question5_last_yaw = 0.0f;

#if 0
static int Question5_AbsInt(int value)
{
    return (value >= 0) ? value : -value;
}

static int Question5_LimitInt(int value, int min, int max)
{
    if(value > max)
    {
        return max;
    }

    if(value < min)
    {
        return min;
    }

    return value;
}

static void Question5_ResetServoControl(void)
{
    question5_servo_speed_q8 = 0;
    question5_servo_last_speed_q8 = 0;
    question5_servo_accel_q8 = 0;
    question5_servo_last_accel_q8 = 0;
    g_question5_servo_angle = Q5_SERVO_MID;
    question5_servo_output_count = 0;
}

static void Question5_UpdateServo_10ms(void)
{
    int raw_speed;
    int raw_accel_q8;
    int control_accel_q8;
    int control_jerk_q8;
    int control_output;
    int servo_target;
    int servo_min;
    int servo_max;

    raw_speed =
        (Question5_AbsInt(g_encoder_left_speed)
            + Question5_AbsInt(g_encoder_right_speed))
        / 2;

    question5_servo_speed_q8 +=
        ((raw_speed << Q5_SERVO_Q_SHIFT)
            - question5_servo_speed_q8)
        / Q5_SERVO_FILTER_DIV;

    raw_accel_q8 =
        question5_servo_speed_q8
        - question5_servo_last_speed_q8;
    question5_servo_last_speed_q8 =
        question5_servo_speed_q8;

    question5_servo_accel_q8 +=
        (raw_accel_q8 - question5_servo_accel_q8)
        / Q5_SERVO_FILTER_DIV;

    control_accel_q8 = question5_servo_accel_q8;
    control_jerk_q8 =
        question5_servo_accel_q8
        - question5_servo_last_accel_q8;
    question5_servo_last_accel_q8 =
        question5_servo_accel_q8;

    if(Question5_AbsInt(control_accel_q8)
        < Q5_SERVO_ACCEL_DEADBAND_Q8)
    {
        control_accel_q8 = 0;
        control_jerk_q8 = 0;
    }

    control_output =
        (Q5_SERVO_ACCEL_KP_NUM * control_accel_q8
            + Q5_SERVO_JERK_KD_NUM * control_jerk_q8)
        / (Q5_SERVO_GAIN_DEN
            * (1 << Q5_SERVO_Q_SHIFT));

    servo_min =
        Q5_SERVO_MID - Q5_SERVO_MAX_OFFSET_DEG;
    servo_max =
        Q5_SERVO_MID + Q5_SERVO_MAX_OFFSET_DEG;
    servo_target = Question5_LimitInt(
        Q5_SERVO_MID - control_output,
        servo_min,
        servo_max);

    question5_servo_output_count++;
    if(question5_servo_output_count
        < Q5_SERVO_OUTPUT_PERIOD_CYCLES)
    {
        return;
    }
    question5_servo_output_count = 0;

    if(servo_target
        > g_question5_servo_angle
            + Q5_SERVO_OUTPUT_STEP_DEG)
    {
        g_question5_servo_angle +=
            Q5_SERVO_OUTPUT_STEP_DEG;
    }
    else if(servo_target
        < g_question5_servo_angle
            - Q5_SERVO_OUTPUT_STEP_DEG)
    {
        g_question5_servo_angle -=
            Q5_SERVO_OUTPUT_STEP_DEG;
    }
    else
    {
        g_question5_servo_angle = servo_target;
    }

    Steer_set(g_question5_servo_angle);
}
#endif

static int Question5_ServoSafeAbs(int value)
{
    return (value >= 0) ? value : -value;
}

static int Question5_ServoSafeLimit(int value, int min, int max)
{
    if(value > max)
    {
        return max;
    }

    if(value < min)
    {
        return min;
    }

    return value;
}

static void Question5_ResetServoSafe(void)
{
    question5_servo_safe_speed_q8 = 0;
    question5_servo_safe_last_speed_q8 = 0;
    question5_servo_safe_last_error_q8 = 0;
    question5_servo_safe_angle = Q5_SERVO_MID;
    question5_servo_safe_count = 0;
}

static void Question5_UpdateServoSafe_10ms(void)
{
    int raw_speed;
    int acceleration_q8;
    int error_q8;
    int derivative_q8;
    int output;
    int target_angle;
    int new_angle;

    raw_speed =
        (Question5_ServoSafeAbs(g_encoder_left_speed)
            + Question5_ServoSafeAbs(g_encoder_right_speed))
        / 2;

    question5_servo_safe_speed_q8 +=
        ((raw_speed << Q5_SERVO_SAFE_Q_SHIFT)
            - question5_servo_safe_speed_q8)
        / Q5_SERVO_SAFE_SPEED_FILTER_DIV;

    question5_servo_safe_count++;
    if(question5_servo_safe_count
        < Q5_SERVO_SAFE_PERIOD_CYCLES)
    {
        return;
    }
    question5_servo_safe_count = 0;

    acceleration_q8 =
        question5_servo_safe_speed_q8
        - question5_servo_safe_last_speed_q8;
    question5_servo_safe_last_speed_q8 =
        question5_servo_safe_speed_q8;

    error_q8 = -acceleration_q8;
    if(Question5_ServoSafeAbs(error_q8)
        < Q5_SERVO_SAFE_DEADBAND_Q8)
    {
        error_q8 = 0;
    }

    derivative_q8 =
        error_q8 - question5_servo_safe_last_error_q8;
    question5_servo_safe_last_error_q8 = error_q8;

    output =
        (Q5_SERVO_SAFE_KP_NUM * error_q8
            + Q5_SERVO_SAFE_KD_NUM * derivative_q8)
        / (Q5_SERVO_SAFE_GAIN_DEN
            * (1 << Q5_SERVO_SAFE_Q_SHIFT));

    target_angle = Question5_ServoSafeLimit(
        Q5_SERVO_MID + output,
        Q5_SERVO_MID - Q5_SERVO_SAFE_MAX_OFFSET_DEG,
        Q5_SERVO_MID + Q5_SERVO_SAFE_MAX_OFFSET_DEG);

    new_angle = question5_servo_safe_angle;
    if(target_angle
        > question5_servo_safe_angle + Q5_SERVO_SAFE_STEP_DEG)
    {
        new_angle += Q5_SERVO_SAFE_STEP_DEG;
    }
    else if(target_angle
        < question5_servo_safe_angle - Q5_SERVO_SAFE_STEP_DEG)
    {
        new_angle -= Q5_SERVO_SAFE_STEP_DEG;
    }
    else
    {
        new_angle = target_angle;
    }

    if(new_angle != question5_servo_safe_angle)
    {
        question5_servo_safe_angle = new_angle;
        Steer_set(question5_servo_safe_angle);
    }
}

static void Question5_Finish(int state)
{
    #if 0
    if(g_question5_state == QUESTION5_STATE_RUNNING)
    {
        Steer_set(Q5_SERVO_MID);
    }
    Question5_ResetServoControl();
    #endif
    if(g_question5_state == QUESTION5_STATE_RUNNING)
    {
        Steer_set(Q5_SERVO_MID);
    }
    Question5_ResetServoSafe();
    Line_follow_reset();
    question5_tracking_time_ms = 0;
    question5_finish_straight_count = 0;
    question5_launch_speed_limit = 0;
    question5_brake_speed_limit = 0;
    question5_braking = 0;
    g_question5_state = state;
}

void Question5_Start(void)
{
    Line_follow_reset();
    Line_follow_set_profile(Question5_GetLineProfile());
    #if 0
    Question5_ResetServoControl();
    Steer_set(Q5_SERVO_MID);
    #endif
    Question5_ResetServoSafe();
    Steer_set(Q5_SERVO_MID);
    question5_launch_speed_limit = 0;
    question5_brake_speed_limit = 0;
    question5_braking = 0;
    g_line_forward_speed_limit = 0;
    question5_tracking_time_ms = 0;
    question5_finish_straight_count = 0;
    question5_start_yaw = YAW.YAW_1;
    question5_last_yaw = YAW.YAW_1;
    g_question5_state = QUESTION5_STATE_RUNNING;
}

void Question5_Stop(void)
{
    Question5_Finish(QUESTION5_STATE_IDLE);
}

uint8 Question5_IsRunning(void)
{
    return (g_question5_state == QUESTION5_STATE_RUNNING);
}

void Question5_Control_5ms(uint32 elapsed_ms)
{
    float yaw_step;
    float lap_yaw;
    float straight_yaw_step_limit;

    if(!Question5_IsRunning())
    {
        return;
    }

    lap_yaw = YAW.YAW_1 - question5_start_yaw;
    if(!question5_braking
        && lap_yaw <= Q5_LAP_STOP_YAW_DEG)
    {
        question5_braking = 1;
        question5_brake_speed_limit =
            Q5_TOP_STRAIGHT_SPEED;
        question5_finish_straight_count = 0;
    }

    question5_tracking_time_ms += elapsed_ms;

    if(question5_braking)
    {
        if(question5_brake_speed_limit
            > Q5_LAUNCH_SPEED_STEP)
        {
            question5_brake_speed_limit -=
                Q5_LAUNCH_SPEED_STEP;
        }
        else
        {
            question5_brake_speed_limit = 0;
        }

        g_line_forward_speed_limit =
            question5_brake_speed_limit;
    }
    else if(question5_launch_speed_limit
        < Q5_TOP_STRAIGHT_SPEED)
    {
        question5_launch_speed_limit += Q5_LAUNCH_SPEED_STEP;

        if(question5_launch_speed_limit
            >= Q5_TOP_STRAIGHT_SPEED)
        {
            question5_launch_speed_limit =
                Q5_TOP_STRAIGHT_SPEED;
            g_line_forward_speed_limit = M_MAX;
        }
        else
        {
            g_line_forward_speed_limit =
                question5_launch_speed_limit;
        }
    }

    G_D_navigation();

    if(!gray_iic_ok)
    {
        Question5_Finish(QUESTION5_STATE_GRAY_ERROR);
        return;
    }

    yaw_step = YAW.YAW_1 - question5_last_yaw;
    question5_last_yaw = YAW.YAW_1;
    straight_yaw_step_limit =
        Q5_LAP_STRAIGHT_YAW_STEP_DEG
        * (float)elapsed_ms / 5.0f;

    if(!question5_braking
        && question5_tracking_time_ms >= Q5_LAP_MIN_RUN_TIME_MS
        && (lap_yaw >= Q5_LAP_YAW_ARM_DEG
            || lap_yaw <= -Q5_LAP_YAW_ARM_DEG)
        && yaw_step <= straight_yaw_step_limit
        && yaw_step >= -straight_yaw_step_limit
        && (g_line_course_point == LINE_POINT_A
            || g_line_course_point == LINE_POINT_B)
        && g_line_state == LINE_STATE_TRACKING)
    {
        question5_finish_straight_count++;
    }
    else
    {
        question5_finish_straight_count = 0;
    }

    if(question5_finish_straight_count
        >= Q5_LAP_STRAIGHT_CONFIRM_CYCLES)
    {
        Question5_Finish(QUESTION5_STATE_DONE);
    }
}

void Question5_Control_10ms(void)
{
    if(!Question5_IsRunning())
    {
        return;
    }

    Line_speed_control_10ms();
    #if 0
    Question5_UpdateServo_10ms();
    #endif

    if(g_encoder_fault)
    {
        Question5_Finish(QUESTION5_STATE_ENCODER_ERROR);
        return;
    }

    if(question5_braking
        && question5_brake_speed_limit == 0
        && g_left_speed_target == 0
        && g_right_speed_target == 0)
    {
        Question5_Finish(QUESTION5_STATE_DONE);
        return;
    }

    Question5_UpdateServoSafe_10ms();
}
