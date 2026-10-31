#ifndef MENU_H
#define MENU_H

#include "zf_common_typedef.h"

typedef enum
{
    MENU_STATE_HOME = 0,
    MENU_STATE_SELECT,
    MENU_STATE_RUNNING,
    MENU_STATE_RESULT
} menu_state_t;

typedef enum
{
    MENU_QUESTION_2 = 2,
    MENU_QUESTION_3,
    MENU_QUESTION_4,
    MENU_QUESTION_5,
    MENU_QUESTION_6
} menu_question_t;

typedef enum
{
    MENU_RESULT_NONE = 0,
    MENU_RESULT_DONE,
    MENU_RESULT_STOPPED,
    MENU_RESULT_GRAY_ERROR,
    MENU_RESULT_ENCODER_ERROR,
    MENU_RESULT_LINE_LOST,
    MENU_RESULT_NOT_READY
} menu_result_t;

void Menu_Init(void);
void Menu_ProcessKeys(uint32 now_ms);
void Menu_Display(uint32 now_ms);

uint8 Menu_TakeStartRequest(void);
uint8 Menu_TakeStopRequest(void);

menu_state_t Menu_GetState(void);
menu_question_t Menu_GetSelectedQuestion(void);
uint32 Menu_GetElapsedMs(uint32 now_ms);

void Menu_SetResult(menu_result_t result, uint32 now_ms);

#endif
