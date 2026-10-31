#include "zf_common_headfile.h"

//对于PID算法来说,
//1.就是你要查阅大量的帖子
//2.合理使用AI---更多不是让AI去给你生成代码，也不是让AI去给你解决什么实际问题
//3.对于PID参数而言,不要指望我或者说AI给你一个具体的参数--多调试


//常用的PID有两种：一种增量式/位置式
//位置式是PID算法标准形式--数学通式
//增量式是位置式一种数学推导
//没有说什么什么器件一定用哪种PID最好，而是看使用场景

//位置式----小车转向---响应优先，精准其次--------------------覆盖

//增量式----控制速度或者说你要把某个目标值达到很精准的地步----增量

//PID都是有一个目标量和一个真实量作为输入的

pid_param_t	Ioc_pid;
pid_param_t	Inc_pid;


void  PID_paramt_init(void)
{
		Ioc_pid.kp=0;
		Ioc_pid.kd=0;

		Inc_pid.kp=0;
		Inc_pid.ki=0; 

}


//限幅
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}



//位置式PID--为了满足标准形式--一般都是期望-实际---我们是去掉I项，只保留PD
float PidLocCtrl(pid_param_t * pid, float error)
{

    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;


    return pid->out;
}

//增量式PID-----PI式扔掉了D
float PidIncCtrl(pid_param_t * pid, float error)
{

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}


//PidLocCtrl(&loc_pid,T-A);