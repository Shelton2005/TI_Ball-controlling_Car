#include "zf_common_headfile.h"
#include "Encoder.h"

volatile int32 g_encoder_left_total = 0;
volatile int32 g_encoder_right_total = 0;
int32 g_encoder_left_speed = 0;
int32 g_encoder_right_speed = 0;

static uint8 left_last_state = 0;
static uint8 right_last_state = 0;
static int32 left_last_sample = 0;
static int32 right_last_sample = 0;

/*
 * Valid quadrature transitions are +/-1. Invalid two-bit jumps are ignored,
 * which rejects many contact/noise glitches without blocking the ISR.
 */
static const int8 quadrature_step[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8 Encoder_read_left_state(void)
{
    return (uint8)(
        (gpio_get_level(ENCODER_LEFT_A_PIN) << 1)
        | gpio_get_level(ENCODER_LEFT_B_PIN));
}

static uint8 Encoder_read_right_state(void)
{
    return (uint8)(
        (gpio_get_level(ENCODER_RIGHT_A_PIN) << 1)
        | gpio_get_level(ENCODER_RIGHT_B_PIN));
}

static void Encoder_left_callback(uint32 event, void *ptr)
{
    uint8 current_state;

    (void)event;
    (void)ptr;

    current_state = Encoder_read_left_state();
    g_encoder_left_total +=
        quadrature_step[(left_last_state << 2) | current_state]
        * ENCODER_LEFT_SIGN;
    left_last_state = current_state;
}

static void Encoder_right_callback(uint32 event, void *ptr)
{
    uint8 current_state;

    (void)event;
    (void)ptr;

    current_state = Encoder_read_right_state();
    g_encoder_right_total +=
        quadrature_step[(right_last_state << 2) | current_state]
        * ENCODER_RIGHT_SIGN;
    right_last_state = current_state;
}

void Encoder_reset(void)
{
    interrupt_disable(GPIOA_INT_IRQn);

    g_encoder_left_total = 0;
    g_encoder_right_total = 0;
    g_encoder_left_speed = 0;
    g_encoder_right_speed = 0;
    left_last_sample = 0;
    right_last_sample = 0;
    left_last_state = Encoder_read_left_state();
    right_last_state = Encoder_read_right_state();

    interrupt_enable(GPIOA_INT_IRQn);
}

void Encoder_init(void)
{
    gpio_init(ENCODER_LEFT_B_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(ENCODER_RIGHT_B_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    exti_init(
        ENCODER_LEFT_A_PIN,
        EXTI_TRIGGER_BOTH,
        Encoder_left_callback,
        NULL);
    exti_init(
        ENCODER_LEFT_B_PIN,
        EXTI_TRIGGER_BOTH,
        Encoder_left_callback,
        NULL);
    exti_init(
        ENCODER_RIGHT_A_PIN,
        EXTI_TRIGGER_BOTH,
        Encoder_right_callback,
        NULL);
    exti_init(
        ENCODER_RIGHT_B_PIN,
        EXTI_TRIGGER_BOTH,
        Encoder_right_callback,
        NULL);

    interrupt_set_priority(GPIOA_INT_IRQn, 1);
    Encoder_reset();
}

void Encoder_sample_10ms(void)
{
    int32 left_now = g_encoder_left_total;
    int32 right_now = g_encoder_right_total;

    g_encoder_left_speed = left_now - left_last_sample;
    g_encoder_right_speed = right_now - right_last_sample;
    left_last_sample = left_now;
    right_last_sample = right_now;
}
