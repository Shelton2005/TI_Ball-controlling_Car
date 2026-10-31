#ifndef ENCODER_H
#define ENCODER_H

#define ENCODER_LEFT_A_PIN       B6
#define ENCODER_LEFT_B_PIN       B7
#define ENCODER_RIGHT_A_PIN      B12
#define ENCODER_RIGHT_B_PIN      B13

/*
 * Signed counts are kept for direction diagnostics. The speed PI converts
 * each wheel speed to an absolute pulse-count magnitude before control.
 */
#define ENCODER_LEFT_SIGN        (-1)
#define ENCODER_RIGHT_SIGN       (-1)

extern volatile int32 g_encoder_left_total;
extern volatile int32 g_encoder_right_total;
extern int32 g_encoder_left_speed;
extern int32 g_encoder_right_speed;

void Encoder_init(void);
void Encoder_reset(void);
void Encoder_sample_10ms(void);

#endif
