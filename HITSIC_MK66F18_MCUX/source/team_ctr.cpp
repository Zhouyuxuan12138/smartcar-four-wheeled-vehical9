#include "team_ctr.hpp"

bool delay_runcar = 0;//延迟发车标志位
float error_n = 0;
float error_n_1 = 0;

cardata c_data[2]=
{
        {{22,0,50},7.55,7.55,0.019,0.012,100},
        {{22,0,50},7.55,7.55,0.019,0.012,100},
};
void Motor_ctr(void)//电机控制，暂时匀速
{
    if(ADC[1]<=40&&ADC[7]<=40||delay_runcar==0)
    {
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, 0U);//右轮正转kFTM_Chnl_0> kFTM_Chnl_1
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, 0U);//左轮正转kFTM_Chnl_3> kFTM_Chnl_2
    }
    else
    {
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, c_data[0].Motorspeed[0]);//右轮正转kFTM_Chnl_0> kFTM_Chnl_1
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0U);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, c_data[0].Motorspeed[0]);//左轮正转kFTM_Chnl_3> kFTM_Chnl_2
    }
}
void servo_init(float *pwm)
{

    *pwm = c_data[0].servo_mid;

}
void servo_pid()
{
    //servo_init(&(c_data[0].servo_pwm));
    float pwm_error = 0;
    //error_n = get_error(c_data[0].foresight);
    error_n = Get_erro();
    //error_n = (float)(AD[0]-AD[1])/(AD[1]*AD[0]);
    pwm_error = c_data[0].Kp*error_n+c_data[0].Kd*(error_n-error_n_1);
    c_data[0].servo_pwm=c_data[0].servo_mid+pwm_error;
    if(c_data[0].servo_pwm<6.8)
        c_data[0].servo_pwm=6.8;
    else if(c_data[0].servo_pwm>8.2)
        c_data[0].servo_pwm=8.2;

    error_n_1 = error_n;
}

void servo()
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,c_data[0].servo_pwm);
}

