#ifndef QUESTION5_H
#define QUESTION5_H

#include "zf_common_typedef.h"
#include "Questoin_2.h"

#define QUESTION5_STATE_IDLE           0
#define QUESTION5_STATE_RUNNING        1
#define QUESTION5_STATE_DONE           2
#define QUESTION5_STATE_GRAY_ERROR     3
#define QUESTION5_STATE_ENCODER_ERROR  4

extern int g_question5_state;
/* extern int g_question5_servo_angle; */ /* Servo control disabled. */

const line_follow_profile_t *Question5_GetLineProfile(void);
void Question5_Start(void);
void Question5_Stop(void);
void Question5_Control_5ms(uint32 elapsed_ms);
void Question5_Control_10ms(void);
uint8 Question5_IsRunning(void);

#endif
