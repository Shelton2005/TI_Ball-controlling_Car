#include "zf_common_headfile.h"

uint16 gray_adc_raw[GRAY_SENSOR_NUM] = {0};

/*
 * Replace these values with the K1/K2 values printed by
 * Gray_sensor_test() so calibration survives a power cycle.
 */
uint16 gray_white[GRAY_SENSOR_NUM] = {0};
uint16 gray_black[GRAY_SENSOR_NUM] = {0};

int gray_level[GRAY_SENSOR_NUM] = {0};
uint8 gray_sensor_ok = 0;
uint8 gray_digital_raw = 0;

static void Gray_select_channel(uint8 channel)
{
    gpio_set_level(GRAY_AD0_PIN, (channel >> 0) & 0x01);
    gpio_set_level(GRAY_AD1_PIN, (channel >> 1) & 0x01);
    gpio_set_level(GRAY_AD2_PIN, (channel >> 2) & 0x01);
    system_delay_us(GRAY_MUX_SETTLE_US);
}

static uint16 Gray_read_channel(uint8 channel)
{
    Gray_select_channel(channel);

    /* Discard the first sample after changing the analog mux channel. */
    (void)adc_convert(GRAY_ADC_PIN);
    return adc_mean_filter_convert(GRAY_ADC_PIN, GRAY_ADC_AVG_COUNT);
}

static void Gray_read_all_analog(void)
{
    uint8 i;

    for(i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        gray_adc_raw[i] = Gray_read_channel(i);
    }
}

static uint8 Gray_validate_calibration(void)
{
    uint8 i;

    for(i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        /*
         * Zero means that a sample has not been recorded yet.  A full-scale
         * white sample is rejected as ADC clipping.  Normal operation
         * specified by the manual is white > black.
         */
        if(gray_black[i] == 0
            || gray_white[i] >= 4095
            || gray_white[i] <= gray_black[i]
            || gray_white[i] - gray_black[i] < GRAY_CAL_MIN_GAP)
        {
            return 0;
        }
    }

    return 1;
}

static void Gray_update_digital(void)
{
    uint8 i;
    uint16 enter_black;
    uint16 enter_white;

    gray_digital_raw = 0;

    if(!gray_sensor_ok)
    {
        for(i = 0; i < GRAY_SENSOR_NUM; i++)
        {
            gray_level[i] = 0;
        }
        return;
    }

    for(i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        /*
         * Manual hysteresis thresholds:
         * enter black = (2 * black + white) / 3
         * enter white = (black + 2 * white) / 3
         */
        enter_black = (uint16)(
            (2u * gray_black[i] + gray_white[i]) / 3u);
        enter_white = (uint16)(
            (gray_black[i] + 2u * gray_white[i]) / 3u);

        if(gray_adc_raw[i] <= enter_black)
        {
            gray_level[i] = 1;
        }
        else if(gray_adc_raw[i] >= enter_white)
        {
            gray_level[i] = 0;
        }
        /* Between the two thresholds, retain the previous state. */

        if(gray_level[i])
        {
            gray_digital_raw |= (uint8)(1u << i);
        }
    }
}

static void Gray_print_calibration(
    const char *name,
    const uint16 values[GRAY_SENSOR_NUM])
{
    printf(
        "%s,{%u,%u,%u,%u,%u,%u,%u,%u}\r\n",
        name,
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
        values[6],
        values[7]);
}

void Gray_sensor_init(void)
{
    gpio_init(GRAY_AD0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_AD1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAY_AD2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    adc_init(GRAY_ADC_PIN, ADC_12BIT);

    system_delay_ms(20);

    gray_digital_raw = 0;
    gray_sensor_ok = Gray_validate_calibration();
}

void Gray_scan(void)
{
    /* Raw ADC data remains available even before calibration is valid. */
    Gray_read_all_analog();
    Gray_update_digital();
}

void Gray_text(void)
{
    uint8 i;

    for(i = 0; i < GRAY_SENSOR_NUM; i++)
    {
        ips200_show_uint(8 * 0, 16 * (i + 1), i + 1, 1);
        ips200_show_uint(8 * 4, 16 * (i + 1), gray_adc_raw[i], 4);
        ips200_show_uint(8 * 11, 16 * (i + 1), gray_level[i], 1);
    }

    ips200_show_uint(8 * 5, 16 * 9, gray_digital_raw, 3);

    if(gray_sensor_ok)
    {
        ips200_show_string(8 * 16, 16 * 9, "OK ");
    }
    else
    {
        ips200_show_string(8 * 16, 16 * 9, "NO ");
    }
}

void Gray_sensor_test(void)
{
    uint8 i;
    uint8 refresh_divider = 0;
    uint8 print_divider = 0;

    Gray_sensor_init();
    ips200_clear();
    ips200_show_string(0, 16 * 0, "CH  ADC12  D");
    ips200_show_string(0, 16 * 9, "MASK:    CAL:");
    ips200_show_string(0, 16 * 11, "K1: SAMPLE WHITE");
    ips200_show_string(0, 16 * 12, "K2: SAMPLE BLACK");
    ips200_show_string(0, 16 * 13, "K4: EXIT");
    ips200_show_string(0, 16 * 14, "LAST: NONE ");

    while(1)
    {
        /* key_scan() is designed for a 10 ms call period. */
        key_scan();

        if(key1_flag)
        {
            key1_flag = 0;
            Gray_read_all_analog();
            for(i = 0; i < GRAY_SENSOR_NUM; i++)
            {
                gray_white[i] = gray_adc_raw[i];
            }
            gray_sensor_ok = Gray_validate_calibration();
            Gray_update_digital();
            Gray_print_calibration("white", gray_white);
            ips200_show_string(0, 16 * 14, "LAST: WHITE");
        }

        if(key2_flag)
        {
            key2_flag = 0;
            Gray_read_all_analog();
            for(i = 0; i < GRAY_SENSOR_NUM; i++)
            {
                gray_black[i] = gray_adc_raw[i];
            }
            gray_sensor_ok = Gray_validate_calibration();
            Gray_update_digital();
            Gray_print_calibration("black", gray_black);
            ips200_show_string(0, 16 * 14, "LAST: BLACK");
        }

        if(key4_flag)
        {
            key4_flag = 0;
            return;
        }

        /* Read ADC and refresh the display every 100 ms. */
        if(refresh_divider == 0)
        {
            Gray_scan();
            Gray_text();

            /* Limit serial traffic to one line every 500 ms. */
            if(print_divider == 0)
            {
                printf(
                    "gray,%u,%u,%u,%u,%u,%u,%u,%u,mask,%u,cal,%u\r\n",
                    gray_adc_raw[0],
                    gray_adc_raw[1],
                    gray_adc_raw[2],
                    gray_adc_raw[3],
                    gray_adc_raw[4],
                    gray_adc_raw[5],
                    gray_adc_raw[6],
                    gray_adc_raw[7],
                    gray_digital_raw,
                    gray_sensor_ok);
            }

            print_divider++;
            if(print_divider >= 5)
            {
                print_divider = 0;
            }
        }

        refresh_divider++;
        if(refresh_divider >= 10)
        {
            refresh_divider = 0;
        }

        system_delay_ms(10);
    }
}
