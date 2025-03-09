# include "pid.h"
# include "main.h"
# include "motor.h"

#define MID 0.5

PID pid_speed;
PID pid_angle;
float KP_speed=2.6;//0.5
float KI_speed=0.013;
float KD_speed=0;

int KP_angle=-210;//-350
int KI_angle=0;
float KD_angle=1.24;//-6000

extern float pitch, roll , yaw;
extern int encoder_left , encoder_right;
extern int encoder_sum;
extern short gyrox, gyroy, gyroz;
int angle_output;
float speed_output;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

void PID_Init(void)//PID参数初始化
{
    pid_speed.err = 0;
    pid_speed.integral = 0;
    pid_speed.maxIntegral = 20000;
    pid_speed.maxOutput = 7200;
    pid_speed.lastErr = 0;
    pid_speed.output = 0;
    pid_speed.kp = KP_speed;
    pid_speed.ki = KI_speed;
    pid_speed.kd = KD_speed;

    pid_angle.err = 0;
    pid_angle.integral = 0;
    pid_angle.maxIntegral = 0;
    pid_angle.maxOutput = 7200;
    pid_angle.lastErr = 0;
    pid_angle.output = 0;
    pid_angle.kp = KP_angle;//这几个宏定义要自己补充
    pid_angle.ki = KI_angle;
    pid_angle.kd = KD_angle;
}
/****************************************
 * 作用：速度环PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
float Speed_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    pid->err = target - feedback;
    pid->err = 0.3*pid->err+0.7*pid->lastErr;

    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = (pid->ki==0)?0:-pid->maxIntegral / pid->ki;
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = (pid->ki==0)?0:pid->maxIntegral / pid->ki;


    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral);//全量式PID

    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;

    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;
    return pid->output;
}

/****************************************
 * 作用：角度环PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
int Angle_PID_Realize(PID* pid,float target,float angle,short gyro)//一次PID计算
{
    pid->err = target - angle;
    pid->output = (pid->kp * pid->err) + (pid->kd * gyro);//PD


    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;

    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;
    return pid->output;
}

void control(float speed_target)
{
	speed_output=Speed_PID_Realize(&pid_speed,speed_target,encoder_sum);
	angle_output=Angle_PID_Realize(&pid_angle,speed_output+MID,roll,gyrox);
	load(angle_output,angle_output);
}


