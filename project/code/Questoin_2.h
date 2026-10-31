#ifndef QUESTOIN_2_H
#define QUESTOIN_2_H

#define LINE_STATE_TRACKING       0
#define LINE_STATE_GAP            1
#define LINE_STATE_SEARCH         2
#define LINE_STATE_SENSOR_ERROR   3
#define LINE_STATE_LAUNCH         4
#define LINE_STATE_ARC_ENTRY      5
#define LINE_STATE_BOTTOM_STRAIGHT 6
#define LINE_STATE_TOP_STRAIGHT   7

/* The value is the next course point: A-B expects B, C-D expects D. */
#define LINE_POINT_B              0
#define LINE_POINT_C              1
#define LINE_POINT_D              2
#define LINE_POINT_A              3

typedef struct
{
    int cruise_speed;
    int top_straight_speed;
    int curve_speed;
    int gap_speed;
    int bottom_straight_speed;
    int search_forward_speed;
    int search_turn_speed;
    int arc_entry_outer_speed;
    int arc_entry_inner_speed;
    int arc_search_outer_speed;
    int arc_search_inner_speed;
    int min_forward_speed;
    int pwm_step_limit;
    int turn_limit;
    int straight_kp_num;
    int straight_kd_num;
    int curve_kp_num;
    int curve_kd_num;
    int speed_error_div;
    int curve_min_forward_speed;
    int encoder_command_div;
    int speed_kp;
    int speed_ki;
    int speed_integral_min;
    int speed_integral_max;
    int speed_min_pwm;
    int curve_pwm_limit;
    int straight_pwm_limit;
    int curve_open_loop_enabled;
    int curve_open_outer_speed;
    int curve_open_inner_speed;
    int curve_error_threshold;
    int curve_correction_kp_num;
    int curve_correction_ki_num;
    int curve_correction_kd_num;
    int curve_correction_integral_limit;
    int curve_correction_limit;
} line_follow_profile_t;

extern int g_line_error;
extern int g_line_active_count;
extern int g_line_lost_count;
extern int g_line_state;
extern int g_line_heading_error_deg;
extern int g_left_speed_target;
extern int g_right_speed_target;
extern int g_left_speed_cmd;
extern int g_right_speed_cmd;
extern int g_line_course_point;
extern int g_line_point_armed;
extern int g_line_gap_active;
extern int g_line_active_level;
extern int g_line_level_calibrated;
/*
 * Optional forward-speed limit applied before steering and target slew.
 * Leave at M_MAX for normal Question 2 operation.
 */
extern int g_line_forward_speed_limit;
/* Bit 0: left encoder stalled. Bit 1: right encoder stalled. */
extern int g_encoder_fault;

void Line_follow_reset(void);
void Line_follow_set_profile(const line_follow_profile_t *profile);
void Line_follow_set_bottom_straight_trigger(float yaw_delta_deg);
void Line_follow_set_center_weight(int center_weight);
void Line_speed_control_10ms(void);
void G_D_navigation(void);

#endif
