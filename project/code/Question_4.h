#ifndef QUESTION_4_H
#define QUESTION_4_H

#include "zf_common_typedef.h"

#define QUESTION_4_STATE_IDLE           0
#define QUESTION_4_STATE_RUNNING        1
#define QUESTION_4_STATE_DONE           2
#define QUESTION_4_STATE_LINE_LOST      3
#define QUESTION_4_STATE_GRAY_ERROR     4
#define QUESTION_4_STATE_ENCODER_ERROR  5

#define QUESTION_4_PHASE_IDLE           0
#define QUESTION_4_PHASE_ACCELERATING   1
#define QUESTION_4_PHASE_CRUISING       2
#define QUESTION_4_PHASE_DECELERATING   3
#define QUESTION_4_PHASE_STOPPED        4

extern int g_question_4_state;
extern int g_question_4_motion_phase;
extern int g_question_4_speed_limit;
extern uint32 g_question_4_distance_mm;
extern int g_question_4_yaw_delta_deg;

void Question_4_Start(uint32 now_ms);
void Question_4_Stop(void);
void Question_4_Control_5ms(uint32 now_ms);
void Question_4_Control_10ms(void);
uint8 Question_4_IsRunning(void);

#endif
