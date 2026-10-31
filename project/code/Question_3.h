#ifndef QUESTION_3_H
#define QUESTION_3_H

#include "zf_common_typedef.h"

#define QUESTION_3_STATE_IDLE       0
#define QUESTION_3_STATE_RUNNING    1
#define QUESTION_3_STATE_DONE       2
#define QUESTION_3_STATE_STOPPED    3

#define QUESTION_3_PHASE_IDLE              0
#define QUESTION_3_PHASE_MID_TO_A          1
#define QUESTION_3_PHASE_A_TO_B            2
#define QUESTION_3_PHASE_HOLD_B_TO_C       3
#define QUESTION_3_PHASE_C_TO_D            4
#define QUESTION_3_PHASE_D_TO_E            5
#define QUESTION_3_PHASE_HOLD_E_TO_F       6
#define QUESTION_3_PHASE_F_TO_G            7
#define QUESTION_3_PHASE_G_TO_H            8
#define QUESTION_3_PHASE_FINAL_HOLD        9
#define QUESTION_3_PHASE_STOPPED          10

extern int g_question_3_state;
extern int g_question_3_phase;
extern int g_question_3_servo_angle;
extern uint32 g_question_3_elapsed_ms;

void Question_3_Start(uint32 now_ms);
void Question_3_Stop(void);
void Question_3_Control_5ms(uint32 now_ms);
uint8 Question_3_IsRunning(void);

#endif
