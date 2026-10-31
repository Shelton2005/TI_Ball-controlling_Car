#include "zf_common_headfile.h"
#include "Encoder.h"

/* QUESTION_2_ROUTE_V3: A-B and C-D both use gray-sensor line following. */

/*
 * 8-channel digital line following.
 * gray_level[0] is car-left, gray_level[7] is car-right.
 * Current Gray_sensor code and the old navigation code treat level 1 as line detected.
 * G_D_navigation is called every 5 ms.
 */
#define LINE_ACTIVE_LEVEL_DEFAULT  1

#define LINE_CRUISE_SPEED          1600
#define LINE_TOP_STRAIGHT_SPEED    3500
#define LINE_CURVE_SPEED           800
#define LINE_GAP_SPEED             800
#define LINE_BOTTOM_STRAIGHT_SPEED 2000
#define LINE_SEARCH_FORWARD_SPEED  700
#define LINE_SEARCH_TURN_SPEED     550
#define LINE_ARC_ENTRY_OUTER_SPEED 1200
#define LINE_ARC_ENTRY_INNER_SPEED 600
#define LINE_ARC_SEARCH_OUTER_SPEED 1050
#define LINE_ARC_SEARCH_INNER_SPEED 550
#define LINE_MIN_FORWARD_SPEED     650
#define LINE_PWM_STEP_LIMIT        220
#define LINE_RIGHT_MOTOR_POLARITY  (-1)

#define LINE_TURN_LIMIT            1350
#define LINE_KP_NUM                70
#define LINE_KD_NUM                70
#define LINE_CURVE_KP_NUM          65
#define LINE_CURVE_KD_NUM          70
#define LINE_GAIN_DEN              100
#define LINE_SPEED_ERROR_DIV       8
#define LINE_CURVE_MIN_FORWARD_SPEED 700

/* All cycle counts below use the 5 ms line-control period. */
#define LINE_LOST_CONFIRM_CYCLES   3
#define LINE_FOUND_CONFIRM_CYCLES  3
#define LINE_STRAIGHT_WHITE_CONFIRM_CYCLES 3
#define LINE_POINT_ARM_CYCLES      20
#define LINE_GAP_HOLD_CYCLES       50
#define LINE_ARC_ENTRY_CYCLES      100
#define LINE_ARC_SEARCH_CYCLES     200
#define LINE_DIRECTION_DEADBAND    250
#define LINE_STRAIGHT_HEADING_DEG  180.0f
#define LINE_POINT_YAW_WINDOW      55.0f
#define LINE_POINT_A_MIN_YAW       300.0f
#define LINE_C_HEADING_TRIGGER_DEG (-175.0f)
#define LINE_FULL_TURN_DEG          360.0f
#define LINE_HEADING_KP            20.0f
#define LINE_HEADING_TURN_SIGN     (-1.0f)

/* Maps speed-command units to quadrature edges measured every 10 ms. */
#define LINE_ENCODER_COMMAND_DIV   100
#define LINE_SPEED_KP              30
#define LINE_SPEED_KI              1.6
#define LINE_SPEED_INTEGRAL_MIN    (-60)
#define LINE_SPEED_INTEGRAL_MAX    250
#define LINE_SPEED_MIN_PWM         500
#define LINE_ENCODER_STALL_CYCLES  30
#define LINE_CURVE_PWM_LIMIT       4000
#define LINE_TOP_PWM_LIMIT         5000

/*
 * Question 2 normal curves use a fixed clockwise wheel-speed differential.
 * These values are private to question2_line_profile; Questions 4 and 5
 * replace the shared controller profile when they start.
 */
#define LINE_CURVE_OPEN_LOOP_ENABLE       1
#define LINE_CURVE_OPEN_OUTER_SPEED    1650
#define LINE_CURVE_OPEN_INNER_SPEED    800

#ifndef LINE_MOTOR_CTRL
#define LINE_MOTOR_CTRL        TB6612_D_Motor_ctrl
/* Use this instead if your car is wired with DRV8701. */
/* #define LINE_MOTOR_CTRL     DRV8701_D_Motor_ctrl */
#endif

static const line_follow_profile_t question2_line_profile =
{
    .cruise_speed = LINE_CRUISE_SPEED,
    .top_straight_speed = LINE_TOP_STRAIGHT_SPEED,
    .curve_speed = LINE_CURVE_SPEED,
    .gap_speed = LINE_GAP_SPEED,
    .bottom_straight_speed = LINE_BOTTOM_STRAIGHT_SPEED,
    .search_forward_speed = LINE_SEARCH_FORWARD_SPEED,
    .search_turn_speed = LINE_SEARCH_TURN_SPEED,
    .arc_entry_outer_speed = LINE_ARC_ENTRY_OUTER_SPEED,
    .arc_entry_inner_speed = LINE_ARC_ENTRY_INNER_SPEED,
    .arc_search_outer_speed = LINE_ARC_SEARCH_OUTER_SPEED,
    .arc_search_inner_speed = LINE_ARC_SEARCH_INNER_SPEED,
    .min_forward_speed = LINE_MIN_FORWARD_SPEED,
    .pwm_step_limit = LINE_PWM_STEP_LIMIT,
    .turn_limit = LINE_TURN_LIMIT,
    .straight_kp_num = LINE_KP_NUM,
    .straight_kd_num = LINE_KD_NUM,
    .curve_kp_num = LINE_CURVE_KP_NUM,
    .curve_kd_num = LINE_CURVE_KD_NUM,
    .speed_error_div = LINE_SPEED_ERROR_DIV,
    .curve_min_forward_speed = LINE_CURVE_MIN_FORWARD_SPEED,
    .encoder_command_div = LINE_ENCODER_COMMAND_DIV,
    .speed_kp = LINE_SPEED_KP,
    .speed_ki = LINE_SPEED_KI,
    .speed_integral_min = LINE_SPEED_INTEGRAL_MIN,
    .speed_integral_max = LINE_SPEED_INTEGRAL_MAX,
    .speed_min_pwm = LINE_SPEED_MIN_PWM,
    .curve_pwm_limit = LINE_CURVE_PWM_LIMIT,
    .straight_pwm_limit = LINE_TOP_PWM_LIMIT,
    .curve_open_loop_enabled = LINE_CURVE_OPEN_LOOP_ENABLE,
    .curve_open_outer_speed = LINE_CURVE_OPEN_OUTER_SPEED,
    .curve_open_inner_speed = LINE_CURVE_OPEN_INNER_SPEED,
    .curve_error_threshold = M_MAX,
    .curve_correction_kp_num = 0,
    .curve_correction_ki_num = 0,
    .curve_correction_kd_num = 0,
    .curve_correction_integral_limit = 0,
    .curve_correction_limit = 0
};

static line_follow_profile_t line_profile;

int g_line_error = 0;
int g_line_active_count = 0;
int g_line_lost_count = 0;
int g_line_state = LINE_STATE_TRACKING;
int g_line_heading_error_deg = 0;
int g_left_speed_target = 0;
int g_right_speed_target = 0;
int g_left_speed_cmd = 0;
int g_right_speed_cmd = 0;
int g_encoder_fault = 0;
int g_line_course_point = LINE_POINT_B;
int g_line_point_armed = 0;
int g_line_gap_active = 0;
int g_line_active_level = LINE_ACTIVE_LEVEL_DEFAULT;
int g_line_level_calibrated = 0;
int g_line_forward_speed_limit = M_MAX;

static int filtered_error = 0;
static int last_error = 0;
static int filtered_derivative = 0;
static int curve_correction_integral = 0;
static int search_direction = 1;
static uint8 have_valid_error = 0;
static float gap_heading_target = 0.0f;
static float straight_heading_reference = 0.0f;
static float bottom_straight_trigger_yaw = 0.0f;
static uint16 line_stable_count = 0;
static uint16 straight_white_count = 0;
static uint16 gap_action_count = 0;
static uint8 gap_pending_advance = 0;
static uint8 gap_reacquire_count = 0;
static uint8 bottom_straight_active = 0;
static uint8 top_straight_active = 1;
static int line_center_weight = 250;
static int left_speed_integral = 0;
static int right_speed_integral = 0;
static int left_last_target_sign = 0;
static int right_last_target_sign = 0;
static uint16 left_no_pulse_count = 0;
static uint16 right_no_pulse_count = 0;

static int limit_int(int value, int min, int max)
{
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

static int abs_int(int value)
{
    return (value >= 0) ? value : -value;
}

static float abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float nearest_straight_heading(float yaw)
{
    int straight_index;
    float relative_yaw = yaw - straight_heading_reference;

    if(relative_yaw >= 0.0f)
    {
        straight_index = (int)(
            (relative_yaw + 90.0f) / LINE_STRAIGHT_HEADING_DEG);
    }
    else
    {
        straight_index = (int)(
            (relative_yaw - 90.0f) / LINE_STRAIGHT_HEADING_DEG);
    }

    return straight_heading_reference
         + (float)straight_index * LINE_STRAIGHT_HEADING_DEG;
}

static uint8 course_point_is_arc_entry(int point)
{
    return (point == LINE_POINT_B || point == LINE_POINT_D);
}

static uint8 course_point_yaw_matches(int point)
{
    float yaw_from_start =
        abs_float(YAW.YAW_1 - straight_heading_reference);

    if(point == LINE_POINT_B)
    {
        return (yaw_from_start <= LINE_POINT_YAW_WINDOW);
    }

    if(point == LINE_POINT_C || point == LINE_POINT_D)
    {
        return (
            abs_float(yaw_from_start - LINE_STRAIGHT_HEADING_DEG)
            <= LINE_POINT_YAW_WINDOW);
    }

    return (yaw_from_start >= LINE_POINT_A_MIN_YAW);
}

static uint8 course_point_white_confirmed(void)
{
    uint8 straight_arc_entry =
        (top_straight_active && g_line_course_point == LINE_POINT_B)
        || (bottom_straight_active
            && g_line_course_point == LINE_POINT_D);

    if(straight_arc_entry)
    {
        return (
            straight_white_count
            >= LINE_STRAIGHT_WHITE_CONFIRM_CYCLES);
    }

    return course_point_yaw_matches(g_line_course_point);
}

static void advance_course_point(void)
{
    if(g_line_course_point >= LINE_POINT_A)
    {
        g_line_course_point = LINE_POINT_B;
        top_straight_active = 1;
        bottom_straight_active = 0;
    }
    else
    {
        g_line_course_point++;
    }
}

static int slew_int(int target, int current)
{
    int step_limit = limit_int(line_profile.pwm_step_limit, 1, M_MAX);

    if(target > current + step_limit)
    {
        return current + step_limit;
    }

    if(target < current - step_limit)
    {
        return current - step_limit;
    }

    return target;
}

static void apply_motor_command(int left_target, int right_target)
{
    left_target = limit_int(left_target, M_MIN, M_MAX);
    right_target = limit_int(right_target, M_MIN, M_MAX);

    g_left_speed_target = slew_int(left_target, g_left_speed_target);
    g_right_speed_target = slew_int(right_target, g_right_speed_target);
}

static int speed_pi_step(
    int target_command,
    int measured_speed_magnitude,
    int *integral,
    int *last_target_sign,
    int pwm_limit)
{
    int target_sign;
    int target_count;
    int command_div;
    int speed_error;
    int pwm;

    if(target_command == 0)
    {
        *integral = 0;
        *last_target_sign = 0;
        return 0;
    }

    target_sign = (target_command > 0) ? 1 : -1;
    if(target_sign != *last_target_sign)
    {
        *integral = 0;
        *last_target_sign = target_sign;
    }

    command_div = limit_int(
        line_profile.encoder_command_div,
        1,
        M_MAX);
    target_count =
        (abs_int(target_command) + command_div / 2)
        / command_div;
    target_count = limit_int(target_count, 1, M_MAX);

    speed_error = target_count - measured_speed_magnitude;
    *integral += speed_error;
    *integral = limit_int(
        *integral,
        line_profile.speed_integral_min,
        line_profile.speed_integral_max);

    pwm = abs_int(target_command)
        + line_profile.speed_kp * speed_error
        + line_profile.speed_ki * (*integral);
    pwm = limit_int(
        pwm,
        line_profile.speed_min_pwm,
        pwm_limit);

    return target_sign * pwm;
}

void Line_speed_control_10ms(void)
{
    int left_speed_feedback;
    int right_speed_feedback;
    int pwm_limit;

    if(abs_int(g_left_speed_target) >= line_profile.min_forward_speed
        && g_encoder_left_speed == 0)
    {
        if(left_no_pulse_count < LINE_ENCODER_STALL_CYCLES)
        {
            left_no_pulse_count++;
        }
    }
    else
    {
        left_no_pulse_count = 0;
    }

    if(abs_int(g_right_speed_target) >= line_profile.min_forward_speed
        && g_encoder_right_speed == 0)
    {
        if(right_no_pulse_count < LINE_ENCODER_STALL_CYCLES)
        {
            right_no_pulse_count++;
        }
    }
    else
    {
        right_no_pulse_count = 0;
    }

    if(left_no_pulse_count >= LINE_ENCODER_STALL_CYCLES)
    {
        g_encoder_fault |= 0x01;
    }

    if(right_no_pulse_count >= LINE_ENCODER_STALL_CYCLES)
    {
        g_encoder_fault |= 0x02;
    }

    if(g_encoder_fault)
    {
        g_left_speed_cmd = 0;
        g_right_speed_cmd = 0;
        LINE_MOTOR_CTRL(0, 0);
        return;
    }

    /*
     * Encoder A/B phase order only changes the sign. Speed control uses
     * pulse-count magnitude, while signed values remain available for display.
     */
    left_speed_feedback = abs_int(g_encoder_left_speed);
    right_speed_feedback = abs_int(g_encoder_right_speed);
    pwm_limit = (top_straight_active || bottom_straight_active)
        ? line_profile.straight_pwm_limit
        : line_profile.curve_pwm_limit;

    g_left_speed_cmd = speed_pi_step(
        g_left_speed_target,
        left_speed_feedback,
        &left_speed_integral,
        &left_last_target_sign,
        pwm_limit);
    g_right_speed_cmd = speed_pi_step(
        g_right_speed_target,
        right_speed_feedback,
        &right_speed_integral,
        &right_last_target_sign,
        pwm_limit);

    LINE_MOTOR_CTRL(
        g_left_speed_cmd,
        g_right_speed_cmd * LINE_RIGHT_MOTOR_POLARITY);
}

void Line_follow_set_profile(const line_follow_profile_t *profile)
{
    if(profile != NULL)
    {
        line_profile = *profile;
        curve_correction_integral = 0;
    }
}

void Line_follow_reset(void)
{
    line_profile = question2_line_profile;

    g_line_error = 0;
    g_line_active_count = 0;
    g_line_lost_count = 0;
    g_line_state = LINE_STATE_TOP_STRAIGHT;
    g_line_heading_error_deg = 0;
    g_left_speed_target = 0;
    g_right_speed_target = 0;
    g_left_speed_cmd = 0;
    g_right_speed_cmd = 0;
    g_encoder_fault = 0;
    g_line_course_point = LINE_POINT_B;

    filtered_error = 0;
    last_error = 0;
    filtered_derivative = 0;
    curve_correction_integral = 0;
    search_direction = 1;
    have_valid_error = 0;
    straight_heading_reference = YAW.YAW_1;
    gap_heading_target = straight_heading_reference;
    bottom_straight_trigger_yaw =
        straight_heading_reference
        + LINE_C_HEADING_TRIGGER_DEG;
    line_stable_count = 0;
    straight_white_count = 0;
    gap_action_count = 0;
    g_line_gap_active = 0;
    g_line_point_armed = 0;
    g_line_active_level = LINE_ACTIVE_LEVEL_DEFAULT;
    g_line_level_calibrated = 0;
    g_line_forward_speed_limit = M_MAX;
    gap_pending_advance = 0;
    gap_reacquire_count = 0;
    bottom_straight_active = 0;
    top_straight_active = 1;
    line_center_weight = 250;
    left_speed_integral = 0;
    right_speed_integral = 0;
    left_last_target_sign = 0;
    right_last_target_sign = 0;
    left_no_pulse_count = 0;
    right_no_pulse_count = 0;

    Encoder_reset();
    LINE_MOTOR_CTRL(0, 0);
}

void Line_follow_set_bottom_straight_trigger(float yaw_delta_deg)
{
    bottom_straight_trigger_yaw =
        straight_heading_reference + yaw_delta_deg;
}

void Line_follow_set_center_weight(int center_weight)
{
    line_center_weight = limit_int(center_weight, 1, 3500);
}

void G_D_navigation(void)
{
    int weight[GRAY_SENSOR_NUM] =
    {
        -3500,
        -2500,
        -1500,
        -line_center_weight,
        line_center_weight,
        1500,
        2500,
        3500
    };
    int weighted_sum = 0;
    int active_count = 0;
    int measured_error;
    int derivative;
    int base_speed;
    int gap_speed;
    int max_turn;
    int turn;
    int steering_kp_num;
    int steering_kd_num;
    int steering_min_forward_speed;
    int correction_error;
    int correction;
    int one_count = 0;
    uint8 all_white_detected = 0;
    int i;

    Gray_scan();

    if(!gray_iic_ok)
    {
        g_line_active_count = 0;
        g_line_state = LINE_STATE_SENSOR_ERROR;
        g_left_speed_target = 0;
        g_right_speed_target = 0;
        g_left_speed_cmd = 0;
        g_right_speed_cmd = 0;
        LINE_MOTOR_CTRL(0, 0);
        return;
    }

    for(i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        if(gray_level[i] == 1)
        {
            one_count++;
        }
    }

    /*
     * At launch the narrow black line normally covers fewer channels than the
     * white floor. Learn the minority level once so both sensor polarities are
     * supported without changing code.
     */
    if(!g_line_level_calibrated)
    {
        if(one_count > 0 && one_count <= 3)
        {
            g_line_active_level = 1;
            g_line_level_calibrated = 1;
        }
        else if(one_count >= 5 && one_count < GRAY_SENSOR_NUM)
        {
            g_line_active_level = 0;
            g_line_level_calibrated = 1;
        }
    }

    for(i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        if(gray_level[i] == g_line_active_level)
        {
            weighted_sum += weight[i];
            active_count++;
        }
    }

    /*
     * A 1.8 cm line cannot cover the full array.  The white level is the
     * inverse of the calibrated black-line level, so a wide black marker is
     * not allowed to trigger the B/D all-white transition.
     */
    if(g_line_level_calibrated
        && ((g_line_active_level == 1 && gray_iic_raw == 0x00)
            || (g_line_active_level == 0 && gray_iic_raw == 0xFF)))
    {
        all_white_detected = 1;
    }

    if(all_white_detected)
    {
        if(straight_white_count < LINE_STRAIGHT_WHITE_CONFIRM_CYCLES)
        {
            straight_white_count++;
        }
    }
    else
    {
        straight_white_count = 0;
    }

    /* Any all-equal pattern is still invalid for weighted line error. */
    if(gray_iic_raw == 0x00 || gray_iic_raw == 0xFF)
    {
        weighted_sum = 0;
        active_count = 0;
    }

    g_line_active_count = active_count;

    /*
     * After the B-C arc reaches -175 degrees, C-D starts using the same
     * gray-sensor PD line following as A-B.  The trigger is moved by one full
     * turn after each lap so an accumulated (unwrapped) YAW remains usable.
     */
    if(!top_straight_active
        && !bottom_straight_active
        && YAW.YAW_1 <= bottom_straight_trigger_yaw)
    {
        bottom_straight_active = 1;
        bottom_straight_trigger_yaw -= LINE_FULL_TURN_DEG;
        top_straight_active = 0;
        /* D is now the next expected all-white straight endpoint. */
        g_line_course_point = LINE_POINT_D;
        g_line_point_armed = 0;
        g_line_gap_active = 0;
        g_line_lost_count = 0;
        line_stable_count = 0;
        straight_white_count = 0;
        gap_action_count = 0;
        gap_pending_advance = 0;
        gap_reacquire_count = 0;
        have_valid_error = 0;
        filtered_derivative = 0;
    }

    if(active_count == 0)
    {
        float heading_error;

        curve_correction_integral = 0;
        gap_reacquire_count = 0;
        line_stable_count = 0;

        if(g_line_lost_count < LINE_ARC_SEARCH_CYCLES)
        {
            g_line_lost_count++;
        }

        if(!g_line_gap_active
            && g_line_lost_count >= LINE_LOST_CONFIRM_CYCLES)
        {
            /*
             * A dropout before the previous gap has been stably reacquired is
             * treated as the same gap, not as the next course point.
             */
            if(gap_pending_advance)
            {
                gap_pending_advance = 0;
                g_line_gap_active = 1;
            }
            else if(g_line_point_armed
                && course_point_white_confirmed())
            {
                g_line_point_armed = 0;
                g_line_gap_active = 1;
            }

            if(g_line_gap_active)
            {
                if(g_line_course_point == LINE_POINT_B)
                {
                    top_straight_active = 0;
                }
                else if(g_line_course_point == LINE_POINT_D)
                {
                    bottom_straight_active = 0;
                }

                gap_action_count = 0;
                gap_heading_target =
                    nearest_straight_heading(YAW.YAW_1);
            }
        }

        if(g_line_gap_active)
        {
            if(gap_action_count < LINE_ARC_SEARCH_CYCLES)
            {
                gap_action_count++;
            }

            if(course_point_is_arc_entry(g_line_course_point))
            {
                /*
                 * B and D are known right-arc entries. A deterministic right
                 * turn prevents a centered car from driving straight through
                 * the all-white gap.
                 */
                g_line_state = LINE_STATE_ARC_ENTRY;
                g_line_heading_error_deg = 0;

                if(gap_action_count <= LINE_ARC_ENTRY_CYCLES)
                {
                    apply_motor_command(
                        line_profile.arc_entry_outer_speed,
                        line_profile.arc_entry_inner_speed);
                }
                else
                {
                    apply_motor_command(
                        line_profile.arc_search_outer_speed,
                        line_profile.arc_search_inner_speed);
                }
            }
            else
            {
                /*
                 * C and A leave an arc onto a straight. Hold the nearest
                 * straight heading while crossing the all-white gap.
                 */
                g_line_state = LINE_STATE_GAP;
                heading_error = gap_heading_target - YAW.YAW_1;
                g_line_heading_error_deg = (int)heading_error;
                gap_speed = line_profile.gap_speed;
                turn = (int)(
                    heading_error
                    * LINE_HEADING_KP
                    * LINE_HEADING_TURN_SIGN);

                turn = limit_int(
                    turn,
                    -(gap_speed - line_profile.min_forward_speed),
                    gap_speed - line_profile.min_forward_speed);

                if(gap_action_count <= LINE_GAP_HOLD_CYCLES)
                {
                    apply_motor_command(
                        gap_speed + turn,
                        gap_speed - turn);
                }
                else
                {
                    g_line_state = LINE_STATE_SEARCH;
                    turn =
                        search_direction
                        * line_profile.search_turn_speed;
                    apply_motor_command(
                        line_profile.search_forward_speed + turn,
                        line_profile.search_forward_speed - turn);
                }
            }
        }
        else if(g_line_lost_count >= LINE_LOST_CONFIRM_CYCLES)
        {
            /*
             * This dropout does not match the expected point/YAW window.
             * Recover slowly without advancing the B-C-D-A sequence.
             */
            g_line_state = LINE_STATE_SEARCH;
            g_line_heading_error_deg = 0;
            turn =
                filtered_error
                * line_profile.straight_kp_num
                / LINE_GAIN_DEN;
            turn = limit_int(
                turn,
                -(line_profile.gap_speed
                    - line_profile.min_forward_speed),
                line_profile.gap_speed
                    - line_profile.min_forward_speed);
            apply_motor_command(
                line_profile.gap_speed + turn,
                line_profile.gap_speed - turn);
        }

        return;
    }

    if(g_line_gap_active)
    {
        if(gap_reacquire_count < LINE_FOUND_CONFIRM_CYCLES)
        {
            gap_reacquire_count++;
        }

        if(gap_reacquire_count >= LINE_FOUND_CONFIRM_CYCLES)
        {
            g_line_gap_active = 0;
            gap_pending_advance = 1;
            gap_action_count = 0;
            line_stable_count = 0;
        }
    }
    else
    {
        gap_reacquire_count = 0;

        if(gap_pending_advance || !g_line_point_armed)
        {
            if(line_stable_count < LINE_POINT_ARM_CYCLES)
            {
                line_stable_count++;
            }

            if(line_stable_count >= LINE_POINT_ARM_CYCLES)
            {
                if(gap_pending_advance)
                {
                    advance_course_point();
                    gap_pending_advance = 0;
                }

                g_line_point_armed = 1;
            }
        }
    }

    measured_error = weighted_sum / active_count;

    if(!have_valid_error)
    {
        filtered_error = measured_error;
        last_error = measured_error;
        filtered_derivative = 0;
        have_valid_error = 1;
    }
    else
    {
        filtered_error = (filtered_error * 2 + measured_error) / 3;
        derivative = filtered_error - last_error;
        last_error = filtered_error;

        if(g_line_lost_count > 0)
        {
            filtered_derivative = 0;
        }
        else
        {
            filtered_derivative = (filtered_derivative * 2 + derivative) / 3;
        }
    }

    g_line_lost_count = 0;
    if(top_straight_active)
    {
        g_line_state = LINE_STATE_TOP_STRAIGHT;
    }
    else if(bottom_straight_active)
    {
        g_line_state = LINE_STATE_BOTTOM_STRAIGHT;
    }
    else
    {
        g_line_state = LINE_STATE_TRACKING;
    }
    g_line_heading_error_deg = 0;
    g_line_error = filtered_error;

    if(filtered_error > LINE_DIRECTION_DEADBAND)
    {
        search_direction = 1;
    }
    else if(filtered_error < -LINE_DIRECTION_DEADBAND)
    {
        search_direction = -1;
    }

    if(top_straight_active
        || bottom_straight_active
        || !line_profile.curve_open_loop_enabled)
    {
        curve_correction_integral = 0;
    }

    /*
     * Optional fixed-differential curve mode with a soft PID trim. The fixed
     * wheel-speed split supplies the nominal arc; PID only nudges that split
     * to keep the detected line near the center of the sensor array.
     */
    if(!top_straight_active
        && !bottom_straight_active
        && line_profile.curve_open_loop_enabled)
    {
        int integral_limit;
        int derivative_for_correction;

        correction_error = 0;

        if(filtered_error > line_profile.curve_error_threshold)
        {
            correction_error =
                filtered_error - line_profile.curve_error_threshold;
        }
        else if(filtered_error
            < -line_profile.curve_error_threshold)
        {
            correction_error =
                filtered_error + line_profile.curve_error_threshold;
        }

        integral_limit = limit_int(
            line_profile.curve_correction_integral_limit,
            0,
            M_MAX);

        if(line_profile.curve_correction_ki_num == 0
            || integral_limit == 0)
        {
            curve_correction_integral = 0;
        }
        else if(correction_error != 0)
        {
            curve_correction_integral = limit_int(
                curve_correction_integral + correction_error,
                -integral_limit,
                integral_limit);
        }
        else
        {
            /* Decay the stored trim near center to prevent a hard kick. */
            curve_correction_integral =
                curve_correction_integral * 7 / 8;
        }

        derivative_for_correction =
            (correction_error != 0)
            ? filtered_derivative
            : 0;

        correction =
            (correction_error
                * line_profile.curve_correction_kp_num
            + curve_correction_integral
                * line_profile.curve_correction_ki_num
            + derivative_for_correction
                * line_profile.curve_correction_kd_num)
            / LINE_GAIN_DEN;

        if(line_profile.curve_correction_limit > 0)
        {
            correction = limit_int(
                correction,
                -line_profile.curve_correction_limit,
                line_profile.curve_correction_limit);
        }
        else
        {
            correction = 0;
        }

        apply_motor_command(
            line_profile.curve_open_outer_speed + correction,
            line_profile.curve_open_inner_speed - correction);
        return;
    }

    if(top_straight_active)
    {
        base_speed =
            line_profile.top_straight_speed
            - abs_int(filtered_error)
                / limit_int(line_profile.speed_error_div, 1, M_MAX);
        base_speed = limit_int(
            base_speed,
            line_profile.curve_speed,
            line_profile.top_straight_speed);
    }
    else if(bottom_straight_active)
    {
        base_speed =
            line_profile.bottom_straight_speed
            - abs_int(filtered_error)
                / limit_int(line_profile.speed_error_div, 1, M_MAX);
        base_speed = limit_int(
            base_speed,
            line_profile.curve_speed,
            line_profile.bottom_straight_speed);
    }
    else
    {
        base_speed =
            line_profile.cruise_speed
            - abs_int(filtered_error)
                / limit_int(line_profile.speed_error_div, 1, M_MAX);
        base_speed = limit_int(
            base_speed,
            line_profile.curve_speed,
            line_profile.cruise_speed);
    }

    /*
     * Apply an optional launch limit at the same point where Question 4
     * limits its base speed: before steering and wheel-target slew.
     */
    base_speed = limit_int(
        base_speed,
        0,
        limit_int(g_line_forward_speed_limit, 0, M_MAX));

    if(top_straight_active || bottom_straight_active)
    {
        steering_kp_num = line_profile.straight_kp_num;
        steering_kd_num = line_profile.straight_kd_num;
        steering_min_forward_speed =
            line_profile.min_forward_speed;
    }
    else
    {
        steering_kp_num = line_profile.curve_kp_num;
        steering_kd_num = line_profile.curve_kd_num;
        steering_min_forward_speed =
            line_profile.curve_min_forward_speed;
    }

    turn = (filtered_error * steering_kp_num
          + filtered_derivative * steering_kd_num) / LINE_GAIN_DEN;

    max_turn = base_speed - steering_min_forward_speed;
    max_turn = limit_int(max_turn, 0, line_profile.turn_limit);
    turn = limit_int(turn, -max_turn, max_turn);

    apply_motor_command(base_speed + turn, base_speed - turn);
}
