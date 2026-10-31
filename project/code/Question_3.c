#include "zf_common_headfile.h"
#include "Question_3.h"

/*
 * Question 3 eight-segment open-loop profile.
 *
 * The 100 ms segments provide fast launch/reversal impulses. The A-B and
 * D-E segments recover more slowly so the ball is not driven by another
 * abrupt step. Every ramp is calculated from absolute elapsed time by the
 * existing 5 ms non-blocking task.
 */
#define Q3_SERVO_LEVEL                 SERVO_MOTOR_MID
#define Q3_SERVO_POINT_A               (Q3_SERVO_LEVEL - 17)
#define Q3_SERVO_POINT_B               (Q3_SERVO_LEVEL - 3)
#define Q3_SERVO_POINT_C               (Q3_SERVO_POINT_B-5)
#define Q3_SERVO_POINT_D               (Q3_SERVO_LEVEL + 16)
#define Q3_SERVO_POINT_E               (Q3_SERVO_LEVEL + 4)
#define Q3_SERVO_POINT_F               (Q3_SERVO_POINT_E +2)
#define Q3_SERVO_POINT_G               (Q3_SERVO_LEVEL   )
#define Q3_SERVO_POINT_H               (Q3_SERVO_LEVEL +5 )

#define Q3_POINT_A_TIME_MS             250U
#define Q3_POINT_B_TIME_MS             650U
#define Q3_POINT_C_TIME_MS            1500U
#define Q3_POINT_D_TIME_MS            1750U
#define Q3_POINT_E_TIME_MS            2350U
#define Q3_POINT_F_TIME_MS            3700U
#define Q3_POINT_G_TIME_MS            3800U
#define Q3_POINT_H_TIME_MS            3900U

int g_question_3_state = QUESTION_3_STATE_IDLE;
int g_question_3_phase = QUESTION_3_PHASE_IDLE;
int g_question_3_servo_angle = Q3_SERVO_LEVEL;
uint32 g_question_3_elapsed_ms = 0;

static uint32 question_3_start_ms = 0;

static void Question_3_SetServo(int angle)
{
    g_question_3_servo_angle = angle;
    Steer_set(angle);
}

static int Question_3_InterpolateAngle(
    int start_angle,
    int end_angle,
    uint32 segment_elapsed_ms,
    uint32 segment_duration_ms)
{
    int32 angle_delta = (int32)(end_angle - start_angle);
    int32 scaled_delta;

    if(segment_elapsed_ms >= segment_duration_ms)
    {
        return end_angle;
    }

    scaled_delta =
        angle_delta * (int32)segment_elapsed_ms
        / (int32)segment_duration_ms;

    return start_angle + (int)scaled_delta;
}

static void Question_3_ApplyOutput(int phase, int servo_angle)
{
    g_question_3_phase = phase;

    if(servo_angle != g_question_3_servo_angle)
    {
        Question_3_SetServo(servo_angle);
    }
}

void Question_3_Start(uint32 now_ms)
{
    question_3_start_ms = now_ms;
    g_question_3_elapsed_ms = 0;
    g_question_3_state = QUESTION_3_STATE_RUNNING;
    g_question_3_phase = QUESTION_3_PHASE_MID_TO_A;
    Question_3_SetServo(Q3_SERVO_LEVEL);
}

void Question_3_Stop(void)
{
    Question_3_SetServo(Q3_SERVO_LEVEL);
    g_question_3_state = QUESTION_3_STATE_STOPPED;
    g_question_3_phase = QUESTION_3_PHASE_STOPPED;
}

void Question_3_Control_5ms(uint32 now_ms)
{
    uint32 segment_elapsed_ms;
    int target_angle;

    if(!Question_3_IsRunning())
    {
        return;
    }

    g_question_3_elapsed_ms = now_ms - question_3_start_ms;

    if(g_question_3_elapsed_ms < Q3_POINT_A_TIME_MS)
    {
        target_angle = Question_3_InterpolateAngle(
            Q3_SERVO_LEVEL,
            Q3_SERVO_POINT_A,
            g_question_3_elapsed_ms,
            Q3_POINT_A_TIME_MS);
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_MID_TO_A,
            target_angle);
    }
    else if(g_question_3_elapsed_ms < Q3_POINT_B_TIME_MS)
    {
        segment_elapsed_ms =
            g_question_3_elapsed_ms - Q3_POINT_A_TIME_MS;
        target_angle = Question_3_InterpolateAngle(
            Q3_SERVO_POINT_A,
            Q3_SERVO_POINT_B,
            segment_elapsed_ms,
            Q3_POINT_B_TIME_MS - Q3_POINT_A_TIME_MS);
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_A_TO_B,
            target_angle);
    }
    else if(g_question_3_elapsed_ms < Q3_POINT_C_TIME_MS)
    {
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_HOLD_B_TO_C,
            Q3_SERVO_POINT_C);
    }
    else if(g_question_3_elapsed_ms < Q3_POINT_D_TIME_MS)
    {
        segment_elapsed_ms =
            g_question_3_elapsed_ms - Q3_POINT_C_TIME_MS;
        target_angle = Question_3_InterpolateAngle(
            Q3_SERVO_POINT_C,
            Q3_SERVO_POINT_D,
            segment_elapsed_ms,
            Q3_POINT_D_TIME_MS - Q3_POINT_C_TIME_MS);
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_C_TO_D,
            target_angle);
    }
    else if(g_question_3_elapsed_ms < Q3_POINT_E_TIME_MS)
    {
        segment_elapsed_ms =
            g_question_3_elapsed_ms - Q3_POINT_D_TIME_MS;
        target_angle = Question_3_InterpolateAngle(
            Q3_SERVO_POINT_D,
            Q3_SERVO_POINT_E,
            segment_elapsed_ms,
            Q3_POINT_E_TIME_MS - Q3_POINT_D_TIME_MS);
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_D_TO_E,
            target_angle);
    }
    else if(g_question_3_elapsed_ms < Q3_POINT_F_TIME_MS)
    {
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_HOLD_E_TO_F,
            Q3_SERVO_POINT_F);
    }
    else if(g_question_3_elapsed_ms < Q3_POINT_G_TIME_MS)
    {
        segment_elapsed_ms =
            g_question_3_elapsed_ms - Q3_POINT_F_TIME_MS;
        target_angle = Question_3_InterpolateAngle(
            Q3_SERVO_POINT_F,
            Q3_SERVO_POINT_G,
            segment_elapsed_ms,
            Q3_POINT_G_TIME_MS - Q3_POINT_F_TIME_MS);
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_F_TO_G,
            target_angle);
    }
    else if(g_question_3_elapsed_ms < Q3_POINT_H_TIME_MS)
    {
        segment_elapsed_ms =
            g_question_3_elapsed_ms - Q3_POINT_G_TIME_MS;
        target_angle = Question_3_InterpolateAngle(
            Q3_SERVO_POINT_G,
            Q3_SERVO_POINT_H,
            segment_elapsed_ms,
            Q3_POINT_H_TIME_MS - Q3_POINT_G_TIME_MS);
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_G_TO_H,
            target_angle);
    }
    else
    {
        Question_3_ApplyOutput(
            QUESTION_3_PHASE_FINAL_HOLD,
            Q3_SERVO_POINT_H);
    }
}

uint8 Question_3_IsRunning(void)
{
    return (uint8)(
        g_question_3_state == QUESTION_3_STATE_RUNNING);
}
