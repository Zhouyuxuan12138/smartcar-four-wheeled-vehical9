#ifndef TEAM_CTR_HPP_
#define TEAM_CTR_HPP_
#include "hitsic_common.h"
#include "sc_ftm.h"
#include "image.h"
extern int Motorspeed[3] = {20,0,50};
float error_n=0;
float error_n_1=0;
const float servo_mid=7.5;
float servo_pwm=7.5;

typedef struct _pid{
    float setmid;            //定义设定值
    float actualmid;        //定义实际值
    float err;                //定义偏差值
    float err_next;            //定义上一个偏差值
    float err_last;            //定义最上前的偏差值
    float Kp,Kd;            //定义比例、积分、微分系数
}pid;

void Motor_ctr(void)//电机控制，暂时匀速
{
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, Motorspeed[0]);//右轮正转kFTM_Chnl_0> kFTM_Chnl_1
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, Motorspeed[0]);//左轮正转kFTM_Chnl_3> kFTM_Chnl_2
}
void servo_pid()
{
    float pwm_error=0;
    error_n=get_error();
    pwm_error=0.015*error_n+0.01*(error_n-error_n_1);
    servo_pwm=servo_mid+pwm_error;
    if(servo_pwm<6.8)
        servo_pwm=6.8;
    else if(servo_pwm>8.2)
        servo_pwm=8.2;

    error_n_1=error_n;
};

void servo()
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,servo_pwm);
}






#endif
