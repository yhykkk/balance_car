/*
 * pid.h
 *
 *  Created on: Mar 8, 2025
 *      Author: yhy
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct _PID//PID参数结构体
{
    float kp,ki,kd;
    float err,lastErr;
    float integral,maxIntegral; //积分值
    float output,maxOutput;
}PID;

void control(float speed_target);
void turn (float speed_target);

#endif /* INC_PID_H_ */
