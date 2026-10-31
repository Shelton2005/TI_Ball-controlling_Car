

#define Buzzer_pin   A14

#define Key_1  			 A30
#define Key_2   		 A31
#define Key_3   		 B0
#define Key_4   		 B1


#define SERVO_MOTOR_PWM              (PWM_TIM_A1_CH0_B4) // 主板舵机接口 B4
#define SERVO_MOTOR_FREQ             (50U)

/*
 * 当前舵机标定范围：最小值 15，中值 59，最大值 105。
 */
#define SERVO_MOTOR_LMAX             (15)
#define SERVO_MOTOR_MID              (59)
#define SERVO_MOTOR_RMAX             (105)

#define SERVO_MOTOR_DUTY(x) \
    ((uint32)((float)PWM_DUTY_MAX \
        / (1000.0f / (float)SERVO_MOTOR_FREQ) \
        * (0.5f + (float)(x) / 90.0f)))

#define L_DIR		A27
#define L_PWM		PWM_TIM_G8_CH0_A26

#define R_DIR		B11
#define R_PWM		PWM_TIM_G8_CH0_B10

#define M_MAX        5000
#define M_MIN       -5000


#define AIN_1		A27
#define AIN_2		B11
#define PWM_A		PWM_TIM_G8_CH0_A26

#define BIN_1		B8
#define BIN_2		B9
#define PWM_B		PWM_TIM_G6_CH0_B10

#define M_MAX        5000
#define M_MIN       -5000



extern uint8 key1_flag;																																		
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;


void Buzzer_init(void);
void Buzzer_check(int TIME);

void Key_init(void);
void key_scan(void);
void Key_text(void);

void Isp_text(void);

void Steer_init(void);
void Steer_set(int32 angle);
void Steer_test(void);
void Steer_text(void);

void DRV8701_init(void);
void DRV8701_D_Motor_ctrl(int32 L_SPEED,int32 R_SPEED);
void Motor_text(void);


void TB6612_init(void);
void TB6612_D_Motor_ctrl(int32 L_SPEED,int32 R_SPEED);
void TB6612_Motor_test(void);
