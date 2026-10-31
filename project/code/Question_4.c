#include "zf_common_headfile.h"
#include "Encoder.h"
#include "Questoin_2.h"
#include "Question5.h"
#include "Question_4.h"

/*
 * Question 4 uses Question 5's complete line-follow profile to run normally
 * through the B-C curve. B does not trigger any extra actions. Braking starts
 * after the car reaches the C-D straight and follows Question 5's
 * forward-speed-limit ramp.
 */
#define Q4_SPEED_STEP                  12
#define Q4_C_STOP_YAW_DEG          (-178.0f)

/*
 * Question 4 curve tuning.
 * These values override a private copy of the Q5 profile, so changing them
 * does not alter Question 5.
 */
#define Q4_ARC_ENTRY_OUTER_SPEED     2000
#define Q4_ARC_ENTRY_INNER_SPEED      700
#define Q4_ARC_SEARCH_OUTER_SPEED    1700
#define Q4_ARC_SEARCH_INNER_SPEED    750

#define Q4_CURVE_OPEN_LOOP_ENABLE       1
#define Q4_CURVE_OPEN_OUTER_SPEED    2100
#define Q4_CURVE_OPEN_INNER_SPEED    700
#define Q4_CURVE_CORRECTION_ENABLE      1
#define Q4_CURVE_ERROR_THRESHOLD      200
#define Q4_CURVE_CORRECTION_KP_NUM      90
#define Q4_CURVE_CORRECTION_KI_NUM      2
#define Q4_CURVE_CORRECTION_KD_NUM     60
#define Q4_CURVE_CORRECTION_INTEGRAL_LIMIT 6000
#define Q4_CURVE_CORRECTION_LIMIT      180
#define Q4_GRAY_CENTER_WEIGHT          250

/*
 * Same encoder-speed PD servo controller used by Question 5.
 * It only writes the servo output and does not modify either wheel command.
 */
#define Q4_SERVO_MID             (SERVO_MOTOR_MID + 3)
#define Q4_SERVO_SAFE_Q_SHIFT               8
#define Q4_SERVO_SAFE_SPEED_FILTER_DIV      4
#define Q4_SERVO_SAFE_KP_NUM              250
#define Q4_SERVO_SAFE_KD_NUM               200
#define Q4_SERVO_SAFE_GAIN_DEN            100
#define Q4_SERVO_SAFE_DEADBAND_Q8          128
#define Q4_SERVO_SAFE_MAX_OFFSET_DEG         5
#define Q4_SERVO_SAFE_PERIOD_CYCLES          2U
#define Q4_SERVO_SAFE_STEP_DEG               1

/* Distance calibration for the on-screen travel-distance estimate. */
#define Q4_ENCODER_COUNTS_PER_METER 6500L

int g_question_4_state = QUESTION_4_STATE_IDLE;
int g_question_4_motion_phase = QUESTION_4_PHASE_IDLE;
int g_question_4_speed_limit = 0;
uint32 g_question_4_distance_mm = 0;
int g_question_4_yaw_delta_deg = 0;

static line_follow_profile_t question_4_line_profile;
static float question_4_start_yaw = 0.0f;
static int question_4_launch_speed_limit = 0;
static int question_4_brake_speed_limit = 0;
static uint8 question_4_braking = 0;
static int question_4_servo_safe_speed_q8 = 0;
static int question_4_servo_safe_last_speed_q8 = 0;
static int question_4_servo_safe_last_error_q8 = 0;
static int question_4_servo_safe_angle = Q4_SERVO_MID;
static uint8 question_4_servo_safe_count = 0;

static int Question_4_LimitInt(int value, int min, int max)
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

static int Question_4_AbsInt(int value)
{
    return (value >= 0) ? value : -value;
}

static int32 Question_4_GetTravelCounts(void)
{
    int32 left_counts = g_encoder_left_total;
    int32 right_counts = g_encoder_right_total;

    if(left_counts < 0)
    {
        left_counts = -left_counts;
    }

    if(right_counts < 0)
    {
        right_counts = -right_counts;
    }

    return (left_counts + right_counts) / 2;
}

static int Question_4_GetForwardTarget(void)
{
    int left_target =
        Question_4_AbsInt(g_left_speed_target);
    int right_target =
        Question_4_AbsInt(g_right_speed_target);

    return Question_4_LimitInt(
        (left_target + right_target) / 2,
        0,
        M_MAX);
}

static void Question_4_ResetControlState(void)
{
    question_4_launch_speed_limit = 0;
    question_4_brake_speed_limit = 0;
    question_4_braking = 0;
}

static void Question_4_ResetServoSafe(void)
{
    question_4_servo_safe_speed_q8 = 0;
    question_4_servo_safe_last_speed_q8 = 0;
    question_4_servo_safe_last_error_q8 = 0;
    question_4_servo_safe_angle = Q4_SERVO_MID;
    question_4_servo_safe_count = 0;
}

static void Question_4_UpdateServoSafe_10ms(void)
{
    int raw_speed;
    int acceleration_q8;
    int error_q8;
    int derivative_q8;
    int output;
    int target_angle;
    int new_angle;

    raw_speed =
        (Question_4_AbsInt(g_encoder_left_speed)
            + Question_4_AbsInt(g_encoder_right_speed))
        / 2;

    question_4_servo_safe_speed_q8 +=
        ((raw_speed << Q4_SERVO_SAFE_Q_SHIFT)
            - question_4_servo_safe_speed_q8)
        / Q4_SERVO_SAFE_SPEED_FILTER_DIV;

    question_4_servo_safe_count++;
    if(question_4_servo_safe_count
        < Q4_SERVO_SAFE_PERIOD_CYCLES)
    {
        return;
    }
    question_4_servo_safe_count = 0;

    acceleration_q8 =
        question_4_servo_safe_speed_q8
        - question_4_servo_safe_last_speed_q8;
    question_4_servo_safe_last_speed_q8 =
        question_4_servo_safe_speed_q8;

    error_q8 = -acceleration_q8;
    if(Question_4_AbsInt(error_q8)
        < Q4_SERVO_SAFE_DEADBAND_Q8)
    {
        error_q8 = 0;
    }

    derivative_q8 =
        error_q8 - question_4_servo_safe_last_error_q8;
    question_4_servo_safe_last_error_q8 = error_q8;

    output =
        (Q4_SERVO_SAFE_KP_NUM * error_q8
            + Q4_SERVO_SAFE_KD_NUM * derivative_q8)
        / (Q4_SERVO_SAFE_GAIN_DEN
            * (1 << Q4_SERVO_SAFE_Q_SHIFT));

    target_angle = Question_4_LimitInt(
        Q4_SERVO_MID + output,
        Q4_SERVO_MID - Q4_SERVO_SAFE_MAX_OFFSET_DEG,
        Q4_SERVO_MID + Q4_SERVO_SAFE_MAX_OFFSET_DEG);

    new_angle = question_4_servo_safe_angle;
    if(target_angle
        > question_4_servo_safe_angle
            + Q4_SERVO_SAFE_STEP_DEG)
    {
        new_angle += Q4_SERVO_SAFE_STEP_DEG;
    }
    else if(target_angle
        < question_4_servo_safe_angle
            - Q4_SERVO_SAFE_STEP_DEG)
    {
        new_angle -= Q4_SERVO_SAFE_STEP_DEG;
    }
    else
    {
        new_angle = target_angle;
    }

    if(new_angle != question_4_servo_safe_angle)
    {
        question_4_servo_safe_angle = new_angle;
        Steer_set(question_4_servo_safe_angle);
    }
}

static void Question_4_Finish(int state)
{
    if(g_question_4_state
        == QUESTION_4_STATE_RUNNING)
    {
        Steer_set(Q4_SERVO_MID);
    }
    Question_4_ResetServoSafe();

    g_question_4_state = state;
    g_question_4_motion_phase = QUESTION_4_PHASE_STOPPED;
    g_question_4_speed_limit = 0;
    Question_4_ResetControlState();
    Line_follow_reset();
}

static void Question_4_StartCPointBraking(void)
{
    int current_forward_target =
        Question_4_GetForwardTarget();
    int bottom_straight_speed =
        question_4_line_profile.bottom_straight_speed;

    /*
     * Question 5 starts its brake limit from the current straight speed.
     * At C, use the current chassis-forward target so exiting the slower
     * curve cannot cause an acceleration before braking begins.
     */
    question_4_brake_speed_limit =
        Question_4_LimitInt(
            current_forward_target,
            0,
            bottom_straight_speed);

    question_4_braking = 1;
    g_question_4_motion_phase =
        QUESTION_4_PHASE_DECELERATING;
}

void Question_4_Start(uint32 now_ms)
{
    (void)now_ms;

    Line_follow_reset();
    Line_follow_set_bottom_straight_trigger(
        Q4_C_STOP_YAW_DEG);
    Line_follow_set_center_weight(
        Q4_GRAY_CENTER_WEIGHT);

    /*
     * Clone Q5's known-good route profile, then override only Q4's curve
     * settings. Line_follow_set_profile copies this independent Q4 profile.
     */
    question_4_line_profile =
        *Question5_GetLineProfile();
    question_4_line_profile.arc_entry_outer_speed =
        Q4_ARC_ENTRY_OUTER_SPEED;
    question_4_line_profile.arc_entry_inner_speed =
        Q4_ARC_ENTRY_INNER_SPEED;
    question_4_line_profile.arc_search_outer_speed =
        Q4_ARC_SEARCH_OUTER_SPEED;
    question_4_line_profile.arc_search_inner_speed =
        Q4_ARC_SEARCH_INNER_SPEED;
    question_4_line_profile.curve_open_loop_enabled =
        Q4_CURVE_OPEN_LOOP_ENABLE;
    question_4_line_profile.curve_open_outer_speed =
        Q4_CURVE_OPEN_OUTER_SPEED;
    question_4_line_profile.curve_open_inner_speed =
        Q4_CURVE_OPEN_INNER_SPEED;

#if Q4_CURVE_CORRECTION_ENABLE
    question_4_line_profile.curve_error_threshold =
        Q4_CURVE_ERROR_THRESHOLD;
    question_4_line_profile.curve_correction_kp_num =
        Q4_CURVE_CORRECTION_KP_NUM;
    question_4_line_profile.curve_correction_ki_num =
        Q4_CURVE_CORRECTION_KI_NUM;
    question_4_line_profile.curve_correction_kd_num =
        Q4_CURVE_CORRECTION_KD_NUM;
    question_4_line_profile.curve_correction_integral_limit =
        Q4_CURVE_CORRECTION_INTEGRAL_LIMIT;
    question_4_line_profile.curve_correction_limit =
        Q4_CURVE_CORRECTION_LIMIT;
#else
    /*
     * Q4 diagnostic mode: keep the fixed curve differential but disable all
     * gray-sensor correction. Q5 retains its original correction settings.
     */
    question_4_line_profile.curve_error_threshold = M_MAX;
    question_4_line_profile.curve_correction_kp_num = 0;
    question_4_line_profile.curve_correction_ki_num = 0;
    question_4_line_profile.curve_correction_kd_num = 0;
    question_4_line_profile.curve_correction_integral_limit = 0;
    question_4_line_profile.curve_correction_limit = 0;
#endif

    Line_follow_set_profile(&question_4_line_profile);

    Question_4_ResetServoSafe();
    Steer_set(Q4_SERVO_MID);
    Question_4_ResetControlState();
    question_4_start_yaw = YAW.YAW_1;

    g_line_forward_speed_limit = 0;
    g_question_4_motion_phase =
        QUESTION_4_PHASE_ACCELERATING;
    g_question_4_speed_limit = 0;
    g_question_4_distance_mm = 0;
    g_question_4_yaw_delta_deg = 0;
    g_question_4_state = QUESTION_4_STATE_RUNNING;
}

void Question_4_Stop(void)
{
    if(g_question_4_state
        == QUESTION_4_STATE_RUNNING)
    {
        Steer_set(Q4_SERVO_MID);
    }
    Question_4_ResetServoSafe();

    g_question_4_state = QUESTION_4_STATE_IDLE;
    g_question_4_motion_phase = QUESTION_4_PHASE_IDLE;
    g_question_4_speed_limit = 0;
    g_question_4_distance_mm = 0;
    g_question_4_yaw_delta_deg = 0;
    Question_4_ResetControlState();
    Line_follow_reset();
}

uint8 Question_4_IsRunning(void)
{
    return (
        g_question_4_state
        == QUESTION_4_STATE_RUNNING);
}

void Question_4_Control_5ms(uint32 now_ms)
{
    int32 travel_counts;
    float yaw_delta;

    (void)now_ms;

    if(!Question_4_IsRunning())
    {
        return;
    }

    travel_counts = Question_4_GetTravelCounts();
    g_question_4_distance_mm =
        (uint32)(
            travel_counts * 1000L
            / Q4_ENCODER_COUNTS_PER_METER);

    yaw_delta = YAW.YAW_1 - question_4_start_yaw;
    g_question_4_yaw_delta_deg = (int)yaw_delta;

    /*
     * Question 5 starts braking at about -355 degrees for a full lap.
     * Question 4 only runs A-B-C, so use the corresponding half-lap angle.
     * This trigger is relative to the Q4 start yaw and does not depend on the
     * full-route B-C-D-A course-point state machine reaching LINE_POINT_D.
     */
    if(!question_4_braking
        && yaw_delta <= Q4_C_STOP_YAW_DEG)
    {
        Question_4_StartCPointBraking();
    }

    if(question_4_braking)
    {
        if(question_4_brake_speed_limit
            > Q4_SPEED_STEP)
        {
            question_4_brake_speed_limit -=
                Q4_SPEED_STEP;
        }
        else
        {
            question_4_brake_speed_limit = 0;
        }

        g_question_4_speed_limit =
            question_4_brake_speed_limit;
        g_line_forward_speed_limit =
            question_4_brake_speed_limit;
    }
    else if(question_4_launch_speed_limit
        < question_4_line_profile.top_straight_speed)
    {
        question_4_launch_speed_limit +=
            Q4_SPEED_STEP;

        if(question_4_launch_speed_limit
            >= question_4_line_profile.top_straight_speed)
        {
            question_4_launch_speed_limit =
                question_4_line_profile.top_straight_speed;
            g_question_4_speed_limit =
                question_4_launch_speed_limit;
            g_line_forward_speed_limit = M_MAX;
            g_question_4_motion_phase =
                QUESTION_4_PHASE_CRUISING;
        }
        else
        {
            g_question_4_speed_limit =
                question_4_launch_speed_limit;
            g_line_forward_speed_limit =
                question_4_launch_speed_limit;
        }
    }

    G_D_navigation();

    if(!gray_iic_ok)
    {
        Question_4_Finish(
            QUESTION_4_STATE_GRAY_ERROR);
    }
}

void Question_4_Control_10ms(void)
{
    if(!Question_4_IsRunning())
    {
        return;
    }

    Line_speed_control_10ms();

    if(g_encoder_fault)
    {
        Question_4_Finish(
            QUESTION_4_STATE_ENCODER_ERROR);
        return;
    }

    /*
     * Match Question 5: finish only after the C-point brake limit and both
     * slewed wheel-speed targets have reached zero.
     */
    if(question_4_braking
        && question_4_brake_speed_limit == 0
        && g_left_speed_target == 0
        && g_right_speed_target == 0)
    {
        Question_4_Finish(
            QUESTION_4_STATE_DONE);
        return;
    }

    Question_4_UpdateServoSafe_10ms();
}
