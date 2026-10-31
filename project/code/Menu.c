#include "zf_common_headfile.h"
#include "Menu.h"
/* #include "Question5.h" */ /* Q5 servo display disabled. */

#define MENU_FIRST_QUESTION    MENU_QUESTION_2
#define MENU_LAST_QUESTION     MENU_QUESTION_6

static menu_state_t menu_state = MENU_STATE_HOME;
static menu_question_t selected_question = MENU_QUESTION_2;
static menu_result_t menu_result = MENU_RESULT_NONE;

static uint8 start_request = 0;
static uint8 stop_request = 0;
static uint8 display_dirty = 1;

static uint32 task_start_ms = 0;
static uint32 task_finish_ms = 0;

static void Menu_DrawElapsedTime(uint8 row, uint32 elapsed_ms)
{
    double elapsed_s = (double)elapsed_ms / 1000.0;

    ips200_show_string(0, 16 * row, "TIME:");
    ips200_show_float(8 * 6, 16 * row, elapsed_s, 4, 2);
    ips200_show_string(8 * 14, 16 * row, "s");
}

static void Menu_ClearKeyFlags(void)
{
    key1_flag = 0;
    key2_flag = 0;
    key3_flag = 0;
    key4_flag = 0;
}

static void Menu_DrawHome(void)
{
    ips200_clear();
    ips200_show_string(8 * 4, 16 * 2, "BALL CAR CONTROL");
    ips200_show_string(8 * 3, 16 * 6, "PRESS K1 FOR MENU");
    ips200_show_string(8 * 2, 16 * 9, "K4: EMERGENCY STOP");
}

static void Menu_DrawQuestionLine(
    uint8 row,
    menu_question_t question,
    const char *text)
{
    if(selected_question == question)
    {
        ips200_show_string(0, 16 * row, ">");
    }
    else
    {
        ips200_show_string(0, 16 * row, " ");
    }

    ips200_show_string(8 * 2, 16 * row, text);
}

static void Menu_DrawSelect(void)
{
    ips200_clear();
    ips200_show_string(8 * 5, 16 * 1, "SELECT QUESTION");

    Menu_DrawQuestionLine(4, MENU_QUESTION_2, "Q2 LAP FOLLOW");
    Menu_DrawQuestionLine(6, MENU_QUESTION_3, "Q3 BALL +5/-5");
    Menu_DrawQuestionLine(8, MENU_QUESTION_4, "Q4 A-B + CENTER");
    Menu_DrawQuestionLine(10, MENU_QUESTION_5, "Q5 LAP + CENTER");
    Menu_DrawQuestionLine(12, MENU_QUESTION_6, "Q6 LAP + HOLD POS");

    ips200_show_string(0, 16 * 16, "K2 UP       K3 DOWN");
    ips200_show_string(0, 16 * 18, "K1 START    K4 BACK");
}

static void Menu_DrawRunning(uint32 now_ms)
{
    uint32 elapsed_ms = Menu_GetElapsedMs(now_ms);

    if(display_dirty)
    {
        ips200_clear();
        ips200_show_string(8 * 7, 16 * 1, "TASK RUNNING");
        ips200_show_string(0, 16 * 3, "QUESTION:");
        ips200_show_int(8 * 10, 16 * 3, (int32)selected_question, 1);
        ips200_show_string(0, 16 * 18, "K4: EMERGENCY STOP");
    }

    if(selected_question != MENU_QUESTION_4)
    {
        Menu_DrawElapsedTime(4, elapsed_ms);
    }

    #if 0
    if(selected_question == MENU_QUESTION_5)
    {
        ips200_show_string(0, 16 * 7, "SERVO:");
        ips200_show_int(
            8 * 7, 16 * 7, g_question5_servo_angle, 3);
    }
    #endif
}

static const char *Menu_ResultText(void)
{
    switch(menu_result)
    {
        case MENU_RESULT_DONE:
            return "TASK COMPLETE";

        case MENU_RESULT_STOPPED:
            return "TASK STOPPED";

        case MENU_RESULT_GRAY_ERROR:
            return "GRAY SENSOR ERROR";

        case MENU_RESULT_ENCODER_ERROR:
            return "ENCODER ERROR";

        case MENU_RESULT_LINE_LOST:
            return "LINE LOST";

        case MENU_RESULT_NOT_READY:
            return "TASK NOT READY";

        default:
            return "UNKNOWN RESULT";
    }
}

static void Menu_DrawResult(void)
{
    uint32 elapsed_ms = task_finish_ms - task_start_ms;

    ips200_clear();
    ips200_show_string(8 * 7, 16 * 2, "TASK RESULT");
    ips200_show_string(8 * 5, 16 * 6, Menu_ResultText());
    ips200_show_string(0, 16 * 9, "QUESTION:");
    ips200_show_int(8 * 10, 16 * 9, (int32)selected_question, 1);
    if(selected_question != MENU_QUESTION_4)
    {
        Menu_DrawElapsedTime(11, elapsed_ms);
    }
    ips200_show_string(0, 16 * 16, "K1: MENU");
    ips200_show_string(0, 16 * 18, "K4: HOME");
}

void Menu_Init(void)
{
    menu_state = MENU_STATE_HOME;
    selected_question = MENU_QUESTION_2;
    menu_result = MENU_RESULT_NONE;
    start_request = 0;
    stop_request = 0;
    display_dirty = 1;
    task_start_ms = 0;
    task_finish_ms = 0;
    Menu_ClearKeyFlags();
}

void Menu_ProcessKeys(uint32 now_ms)
{
    uint8 key1_pressed = key1_flag;
    uint8 key2_pressed = key2_flag;
    uint8 key3_pressed = key3_flag;
    uint8 key4_pressed = key4_flag;

    Menu_ClearKeyFlags();

    /* K4 always has priority over the other keys. */
    if(key4_pressed)
    {
        if(menu_state == MENU_STATE_RUNNING)
        {
            stop_request = 1;
            menu_result = MENU_RESULT_STOPPED;
            task_finish_ms = now_ms;
            menu_state = MENU_STATE_RESULT;
        }
        else
        {
            menu_state = MENU_STATE_HOME;
        }

        display_dirty = 1;
        return;
    }

    switch(menu_state)
    {
        case MENU_STATE_HOME:
            if(key1_pressed)
            {
                menu_state = MENU_STATE_SELECT;
                display_dirty = 1;
            }
            break;

        case MENU_STATE_SELECT:
            if(key2_pressed)
            {
                if(selected_question <= MENU_FIRST_QUESTION)
                {
                    selected_question = MENU_LAST_QUESTION;
                }
                else
                {
                    selected_question =
                        (menu_question_t)(selected_question - 1);
                }
                display_dirty = 1;
            }

            if(key3_pressed)
            {
                if(selected_question >= MENU_LAST_QUESTION)
                {
                    selected_question = MENU_FIRST_QUESTION;
                }
                else
                {
                    selected_question =
                        (menu_question_t)(selected_question + 1);
                }
                display_dirty = 1;
            }

            if(key1_pressed)
            {
                start_request = 1;
                menu_result = MENU_RESULT_NONE;
                task_start_ms = now_ms;
                task_finish_ms = now_ms;
                menu_state = MENU_STATE_RUNNING;
                display_dirty = 1;
            }
            break;

        case MENU_STATE_RUNNING:
            /* Only K4 can interrupt a running task. */
            break;

        case MENU_STATE_RESULT:
            if(key1_pressed)
            {
                menu_state = MENU_STATE_SELECT;
                display_dirty = 1;
            }
            break;

        default:
            menu_state = MENU_STATE_HOME;
            display_dirty = 1;
            break;
    }
}

void Menu_Display(uint32 now_ms)
{
    if(!display_dirty && menu_state != MENU_STATE_RUNNING)
    {
        return;
    }

    switch(menu_state)
    {
        case MENU_STATE_HOME:
            Menu_DrawHome();
            break;

        case MENU_STATE_SELECT:
            Menu_DrawSelect();
            break;

        case MENU_STATE_RUNNING:
            Menu_DrawRunning(now_ms);
            break;

        case MENU_STATE_RESULT:
            Menu_DrawResult();
            break;

        default:
            Menu_DrawHome();
            break;
    }

    display_dirty = 0;
}

uint8 Menu_TakeStartRequest(void)
{
    uint8 request = start_request;
    start_request = 0;
    return request;
}

uint8 Menu_TakeStopRequest(void)
{
    uint8 request = stop_request;
    stop_request = 0;
    return request;
}

menu_state_t Menu_GetState(void)
{
    return menu_state;
}

menu_question_t Menu_GetSelectedQuestion(void)
{
    return selected_question;
}

uint32 Menu_GetElapsedMs(uint32 now_ms)
{
    if(menu_state == MENU_STATE_RUNNING)
    {
        return now_ms - task_start_ms;
    }

    return task_finish_ms - task_start_ms;
}

void Menu_SetResult(menu_result_t result, uint32 now_ms)
{
    menu_result = result;
    task_finish_ms = now_ms;
    menu_state = MENU_STATE_RESULT;
    display_dirty = 1;
}
