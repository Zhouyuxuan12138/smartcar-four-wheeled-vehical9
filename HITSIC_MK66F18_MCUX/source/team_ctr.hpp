#ifndef TEAM_CTR_HPP_
#define TEAM_CTR_HPP_
#include "hitsic_common.h"
#include "sc_ftm.h"
extern int Motorspeed[3] = {20,0,50};

extern float pulse=7.6;

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
void PID_init(pid* p,uint8_t *m){
    p->setmid=96.0;
    p->actualmid = m[100];
    p->err=0.0;
    p->err_last=0.0;
    p->err_next=0.0;
    p->Kp=0.2;
    p->Kd=0.2;
}

void pd_ctr(pid *p)
{
        p->setmid = 96;
        p->err=p->setmid-p->actualmid;
        float increment=p->Kp*(p->err-p->err_next)+p->Kd*(p->err-2*p->err_next+p->err_last);
        p->actualmid+=increment;
        p->err_last=p->err_next;
        p->err_next=p->err;
}

float pd_generate_pulse(pid *p)
{
    pulse = (0.05*(p->err)) + 7.6f;
    if((pulse<=6.6)||(pulse>=8.6)) pulse = 7.6;

}

void PWM_clr_servo(void)
{

    SCFTM_PWM_ChangeHiRes(FTM3, kFTM_Chnl_7, 50U, pulse);
}








#endif
