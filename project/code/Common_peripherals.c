#include "zf_common_headfile.h"

//------------------------------------------------------蜂鸣器

void Buzzer_init(void)
{
		 gpio_init(Buzzer_pin, GPO, 0, GPO_PUSH_PULL);
}

void Buzzer_check(int TIME)
{
		 (void)TIME;
		 gpio_set_level(Buzzer_pin,0);//不响
}



//------------------------------------------------------按键

uint8 key1_state = 1;                                                               // 按键动作状态
uint8 key2_state = 1;                                                               // 按键动作状态
uint8 key3_state = 1;                                                               // 按键动作状态
uint8 key4_state = 1;                                                               // 按键动作状态

uint8 key1_state_last = 0;                                                          // 上一次按键动作状态
uint8 key2_state_last = 0;                                                          // 上一次按键动作状态
uint8 key3_state_last = 0;                                                          // 上一次按键动作状态
uint8 key4_state_last = 0;                                                          // 上一次按键动作状态

uint8 key1_flag;																																		//触发标志位
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;


void Key_init(void)
{
		 gpio_init(Key_1, GPI, 1, GPI_PULL_UP);
		 gpio_init(Key_2, GPI, 1, GPI_PULL_UP);
		 gpio_init(Key_3, GPI, 1, GPI_PULL_UP);
		 gpio_init(Key_4, GPI, 1, GPI_PULL_UP);
}



void key_scan(void)
{
    static uint8 key1_press_count = 0;
    static uint8 key2_press_count = 0;
    static uint8 key3_press_count = 0;
    static uint8 key4_press_count = 0;

    key1_state_last = key1_state;
    key2_state_last = key2_state;
    key3_state_last = key3_state;
    key4_state_last = key4_state;

    key1_state = gpio_get_level(Key_1);
    key2_state = gpio_get_level(Key_2);
    key3_state = gpio_get_level(Key_3);
    key4_state = gpio_get_level(Key_4);

    if(!key1_state) { if(key1_press_count < 3) key1_press_count++; }
    else { if(!key1_state_last && key1_press_count >= 3) key1_flag = 1; key1_press_count = 0; }

    if(!key2_state) { if(key2_press_count < 3) key2_press_count++; }
    else { if(!key2_state_last && key2_press_count >= 3) key2_flag = 1; key2_press_count = 0; }

    if(!key3_state) { if(key3_press_count < 3) key3_press_count++; }
    else { if(!key3_state_last && key3_press_count >= 3) key3_flag = 1; key3_press_count = 0; }

    if(!key4_state) { if(key4_press_count < 3) key4_press_count++; }
    else { if(!key4_state_last && key4_press_count >= 3) key4_flag = 1; key4_press_count = 0; }
}

void Key_text(void)
{
		if(key1_flag)
		{
				key1_flag=0;
				Buzzer_check(20);
		}

		if(key2_flag)
		{
				key2_flag=0;
				Buzzer_check(40);
		}
		
		if(key3_flag)
		{
				key3_flag=0;
				Buzzer_check(80);
		}
		
		if(key4_flag)
		{
				key4_flag=0;
				Buzzer_check(160);
		}

}
		
//------------------------------------------------------显示屏

void Isp_text(void)
{
	 ips200_clear();
        ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_seekfree_logo, 240, 80, 240, 80, 0);    // 显示一个RGB565色彩图片 原图240*80 显示240*80 低位在前
        system_delay_ms(1500);

        ips200_full(RGB565_GRAY);
        ips200_show_string( 0 , 16*7,   "SEEKFREE");                            // 显示字符串
        ips200_show_chinese(80, 16*7,   16, (const uint8 *)chinese_test, 4, RGB565_BLUE);               // 显示汉字

        // 显示的 flaot 数据 最多显示 8bit 位整数 最多显示 6bit 位小数
        ips200_show_float(  0 , 16*8,   -13.141592,     1, 6);                  // 显示 float 数据 1bit 整数 6bit 小数 应当显示 -3.141592 总共会有 9 个字符的显示占位
        ips200_show_float(  80, 16*8,   13.141592,      8, 4);                  // 显示 float 数据 8bit 整数 4bit 小数 应当显示 13.1415 总共会有 14 个字符的显示占位 后面会有 5 个字符的空白占位

        ips200_show_int(    0 , 16*9,   -127,           2);                     // 显示 int8 数据
        ips200_show_uint(   80, 16*9,   255,            4);                     // 显示 uint8 数据

        ips200_show_int(    0 , 16*10,  -32768,         4);                     // 显示 int16 数据
        ips200_show_uint(   80, 16*10,  65535,          6);                     // 显示 uint16 数据

        ips200_show_int(    0 , 16*11,  -2147483648,    8);                     // 显示 int32 数据 8bit 整数 应当显示 -47483648
        ips200_show_uint(   80, 16*11,  4294967295,     8);                     // 显示 uint32 数据 10bit 整数 应当显示 4294967295

        system_delay_ms(1000);


     
        ips200_full(RGB565_RED);
        system_delay_ms(500);
        ips200_full(RGB565_GREEN);
        system_delay_ms(500);
        ips200_full(RGB565_BLUE);
        system_delay_ms(500);
        ips200_full(RGB565_WHITE);
        system_delay_ms(500);

}


//-----------------------------------------------------舵机

void Steer_init(void)
{
    pwm_init(
        SERVO_MOTOR_PWM,
        SERVO_MOTOR_FREQ,
        SERVO_MOTOR_DUTY(SERVO_MOTOR_MID));
}


static int32 Steer_limit(int32 angle)
{
    if(angle > SERVO_MOTOR_RMAX)
    {
        angle = SERVO_MOTOR_RMAX;
    }
    else if(angle < SERVO_MOTOR_LMAX)
    {
        angle = SERVO_MOTOR_LMAX;
    }

    return angle;
}

void Steer_set(int32 angle)
{
    angle = Steer_limit(angle);
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(angle));
}

void Steer_test(void)
{
    static int32 angle = SERVO_MOTOR_MID;
    static uint8 display_initialized = 0;

    key_scan();

    if(key1_flag)
    {
        key1_flag = 0;
        angle += 22;
    }
    if(key2_flag)
    {
        key2_flag = 0;
        angle -= 8;
    }
    if(key3_flag)
    {
        key3_flag = 0;
        angle -= 16;
    }
    if(key4_flag)
    {
        key4_flag = 0;
        angle = SERVO_MOTOR_MID;
    }

    angle = Steer_limit(angle);
    Steer_set(angle);

    if(!display_initialized)
    {
        display_initialized = 1;
        ips200_clear();
        ips200_show_string(0, 16 * 1, "SERVO TEST - B4");
        ips200_show_string(0, 16 * 3, "K1:+22  K2:-8");
        ips200_show_string(0, 16 * 4, "K3:-16  K4:MID");
        ips200_show_string(0, 16 * 6, "MIN:15 MID:59 MAX:105");
        ips200_show_string(0, 16 * 8, "VALUE:");
    }

    ips200_show_int(8 * 7, 16 * 8, angle, 5);
}

/* 保留旧函数名，原来的调用代码无需修改。 */
void Steer_text(void)
{
    Steer_test();
}

//-----------------------------------------------------电机

//---------------------------DRV8701

void DRV8701_init(void)
{
		 gpio_init(L_DIR, GPO, 1, GPO_PUSH_PULL);
		 pwm_init(L_PWM, 17000, 0);

		 gpio_init(R_DIR, GPO, 1, GPO_PUSH_PULL);
		 pwm_init(R_PWM, 17000, 0);

}

void DRV8701_D_Motor_ctrl(int32 L_SPEED,int32 R_SPEED)
{
	L_SPEED=L_SPEED>M_MAX?M_MAX:(L_SPEED<M_MIN)?M_MIN:L_SPEED;
	R_SPEED=R_SPEED>M_MAX?M_MAX:(R_SPEED<M_MIN)?M_MIN:R_SPEED;

	if(L_SPEED>0)//正
	{
			gpio_set_level(L_DIR,1);
	    pwm_set_duty(L_PWM,L_SPEED);
	}
	else{
			gpio_set_level(L_DIR,0);
  	    pwm_set_duty(L_PWM,-L_SPEED);
	}
	
  if(R_SPEED>0)//正
	{
			gpio_set_level(R_DIR,1);
	    pwm_set_duty(R_PWM,R_SPEED);
	}
	else{
			gpio_set_level(R_DIR,0);
	    pwm_set_duty(R_PWM,-R_SPEED);
	}


}

void Motor_text(void)//电机测试
{

       static int32 L_SPEED=0,R_SPEED=0;

       key_scan();

          if(key1_flag)
             {
                 key1_flag=0;
                 L_SPEED+=500;
             }
          if(key2_flag)
             {
                 key2_flag=0;
                 L_SPEED-=500;
             }
          if(key3_flag)
             {
                 key3_flag=0;
                 R_SPEED+=500;
             }
          if(key4_flag)
             {
                 key4_flag=0;
                 R_SPEED-=500;
             }


          ips200_show_string(8*0,16*1, "Motor_text");

          ips200_show_string(8*0,16*3, "L:");          ips200_show_int(8*5,  16*3,L_SPEED, 5);
          ips200_show_string(8*0,16*4, "R:");          ips200_show_int(8*5,  16*4,R_SPEED, 5);

          DRV8701_D_Motor_ctrl(L_SPEED,R_SPEED);
}

//---------------------------TB6612


void TB6612_init(void)
{
		 gpio_init(AIN_1, GPO, 1, GPO_PUSH_PULL);
		 gpio_init(AIN_2, GPO, 1, GPO_PUSH_PULL);
		 pwm_init(PWM_A, 17000, 0);

		 gpio_init(BIN_1, GPO, 1, GPO_PUSH_PULL);
		 gpio_init(BIN_2, GPO, 1, GPO_PUSH_PULL);
		 pwm_init(PWM_B, 17000, 0);
}



//STBY--要找个高电平接上去
void TB6612_D_Motor_ctrl(int32 L_SPEED,int32 R_SPEED)
{
	L_SPEED=L_SPEED>M_MAX?M_MAX:(L_SPEED<M_MIN)?M_MIN:L_SPEED;
	R_SPEED=R_SPEED>M_MAX?M_MAX:(R_SPEED<M_MIN)?M_MIN:R_SPEED;

	if(L_SPEED>0)//正
	{
			gpio_set_level(AIN_1,1);
			gpio_set_level(AIN_2,0);

	    pwm_set_duty(PWM_A,L_SPEED);
	}
	else{
			gpio_set_level(AIN_1,0);
			gpio_set_level(AIN_2,1);
	    pwm_set_duty(PWM_A,-L_SPEED);
	}
	
  if(R_SPEED>0)//正
	{
			gpio_set_level(BIN_1,1);
			gpio_set_level(BIN_2,0);
	    pwm_set_duty(PWM_B,R_SPEED);
	}
	else{
			gpio_set_level(BIN_1,0);
			gpio_set_level(BIN_2,1);
	    pwm_set_duty(PWM_B,-R_SPEED);
	}


}

		
void TB6612_Motor_test(void)
{
    static int32 test_speed = 0;
    static uint8 test_mode = 0;
    int32 left_speed = 0;
    int32 right_speed = 0;

    key_scan();

    if(key1_flag)
    {
        key1_flag = 0;
        test_speed += 300;
    }

    if(key2_flag)
    {
        key2_flag = 0;
        test_speed -= 300;
    }

    if(key3_flag)
    {
        key3_flag = 0;
        test_mode++;
        if(test_mode > 2)
        {
            test_mode = 0;
        }
    }

    if(key4_flag)
    {
        key4_flag = 0;
        test_speed = 0;
    }

    test_speed = test_speed > M_MAX ? M_MAX : (test_speed < M_MIN) ? M_MIN : test_speed;

    if(test_mode == 0)
    {
        left_speed = test_speed;
        right_speed = 0;
    }
    else if(test_mode == 1)
    {
        left_speed = 0;
        right_speed = test_speed;
    }
    else
    {
        left_speed = test_speed;
        right_speed = test_speed;
    }

    ips200_show_string(8*0, 16*1, "TB6612 TEST");
    ips200_show_string(8*0, 16*3, "MODE:");
    if(test_mode == 0)
    {
        ips200_show_string(8*7, 16*3, "LEFT ");
    }
    else if(test_mode == 1)
    {
        ips200_show_string(8*7, 16*3, "RIGHT");
    }
    else
    {
        ips200_show_string(8*7, 16*3, "BOTH ");
    }

    ips200_show_string(8*0, 16*4, "SPD:");
    ips200_show_int(8*6, 16*4, test_speed, 5);
    ips200_show_string(8*0, 16*5, "L:");
    ips200_show_int(8*3, 16*5, left_speed, 5);
    ips200_show_string(8*0, 16*6, "R:");
    ips200_show_int(8*3, 16*6, right_speed, 5);
    ips200_show_string(8*0, 16*8, "K1+ K2- K3M K4STOP");

    TB6612_D_Motor_ctrl(left_speed, right_speed);
}
