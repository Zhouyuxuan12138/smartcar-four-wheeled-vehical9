#ifndef TEAM_CTR_HPP_
#define TEAM_CTR_HPP_
#include "hitsic_common.h"
#include "sc_ftm.h"
extern int Motorspeed[3] = {20,0,50};
void Motor_ctr(void)//电机控制，暂时匀速
{
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, Motorspeed[0]);//右轮正转kFTM_Chnl_0> kFTM_Chnl_1
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, Motorspeed[0]);//左轮正转kFTM_Chnl_3> kFTM_Chnl_2
}









#endif
