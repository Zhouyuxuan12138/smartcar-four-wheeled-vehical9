#ifndef TEAM_CTR_HPP_
#define TEAM_CTR_HPP_
#include "hitsic_common.h"
#include "sc_ftm.h"
#include "image.h"
#include "team_elec.hpp"
extern float error_n;
typedef struct _cardata{

    int Motorspeed[3]= {22,0,50};
    float servo_mid=7.55;            //定义舵机中值
    float servo_pwm=7.55;        //定义舵机pwm值
    float Kp = 0.019;                //定义比例系数
    float Kd = 0.012;     //定义积分系数
    int foresight = 100;//定义前瞻，摄像头以多远为标准
}cardata;


extern cardata c_data[2];

void Motor_ctr(void);//电机控制，暂时匀速

void servo_init(float *pwm);

void servo_pid(void);

void servo(void);

#endif
