#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

/* CH1-CH8 are ordered from car-left to car-right. */
#define GRAY_SENSOR_NUM          8

/*
 * Ganv no-MCU sensor connection:
 * P13 A0/A1/B14 drive the CD4051 address inputs.
 * P14 A24 reads the multiplexed analog OUT signal.
 * EN is tied low (or left open, using the sensor's internal pull-down).
 */
#define GRAY_AD0_PIN             A0
#define GRAY_AD1_PIN             A1
#define GRAY_AD2_PIN             B14
#define GRAY_ADC_PIN             ADC0_CH3_A24

#define GRAY_MUX_SETTLE_US       10
#define GRAY_ADC_AVG_COUNT       4
#define GRAY_CAL_MIN_GAP         20

extern uint16 gray_adc_raw[GRAY_SENSOR_NUM];
extern uint16 gray_white[GRAY_SENSOR_NUM];
extern uint16 gray_black[GRAY_SENSOR_NUM];
extern int gray_level[GRAY_SENSOR_NUM];
extern uint8 gray_sensor_ok;
extern uint8 gray_digital_raw;

/*
 * Compatibility aliases keep Question 2/4/5 unchanged.
 * gray_iic_raw is now the post-hysteresis digital bit mask, not I2C data.
 */
#define gray_iic_ok              gray_sensor_ok
#define gray_iic_raw             gray_digital_raw

void Gray_sensor_init(void);
void Gray_scan(void);
void Gray_text(void);
void Gray_sensor_test(void);

#endif
