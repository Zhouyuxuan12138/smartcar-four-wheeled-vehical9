#include "team_elec.hpp"



uint32_t ADC[8];//两个AD通道读取电感值
uint32_t LV_Temp[2][SampleTimes];//电感处理函数
float LV[2]={0,0};//数组
float AD[2]={0,0};//保存滤波后数据数组

void LV_Sample(void)                             // ad采集函数
{
    ADC[1]=SCADC_Sample(ADC0,0,12);
    ADC[7]=SCADC_Sample(ADC0,0,18);

  for(uint8_t i=0 ; i < SampleTimes; i++)
  {
    /*获取采样初值*/
    LV_Temp[0][i]=(uint32_t)SCADC_Sample(ADC0,0,12);//左电感
    LV_Temp[1][i]=(uint32_t)SCADC_Sample(ADC0,0,18);//右电感

  }
}

void LV_Get_Val(void)//约0.3mS                  //对采集的值滤波
{
 // 有时会在0-65535(255)间跳动
  for(uint8_t i=0;i<2;i++)
  {
    for(uint8_t j=0; j < SampleTimes;j++)
    {
         if(LV_Temp[i][j]>240)//剔除毛刺信号
         {
             LV_Temp[i][j] = 240;
         }
    }
  }
}

void LV_Sort(void)
{
    for(uint8_t k=0;k<2;k++)
     {
       for(uint8_t i=0;i < SampleTimes-1;i++)
       {
         for(uint8_t j=i+1;j < SampleTimes;j++)
         {
             if(LV_Temp[k][i]>LV_Temp[k][j])
               Swap(&LV_Temp[k][i],&LV_Temp[k][j]);//交换，swap函数自己写
         }
       }
     }
}

void Swap(uint32_t*a,uint32_t*b)
{
    uint32_t temp;
    temp = *b;
    *b = *a;
    *a = temp;

}
void Normalized(void)//归一化函数加入车无法运行
{
    for(uint8_t i=0;i<2;i++)
    {
        uint32_t max = LV_Temp[i][SampleTimes-1];
        uint32_t min = LV_Temp[i][0];
        for(uint8_t j=0; j < SampleTimes;j++)
        {
            uint32_t thisdata = LV_Temp[i][j];
            LV_Temp[i][j] = (((thisdata-min)/(max - min))*100U);
        }
    }

}
float *LV_average(void)
{
    for(uint8_t k=0;k<2;k++)
     {
       LV[k]=0;
       for(uint8_t i=3;i<SampleTimes-3;i++)
       {
            LV[k]+=(float)LV_Temp[k][i];
       }
       LV[k]=LV[k]/(SampleTimes-6);
       if( LV[k] < MinLVGot )
       {
          LV[k] = MinLVGot;
       }
     }
    for(uint8_t j = 0; j<2 ;j++)
    {
        AD[j] = LV[j];
    }
    return AD;

}
float Get_erro(void)
{
    LV_Sample();
    LV_Get_Val();
    LV_Sort();
    //Normalized();
    float *p;
    p = LV_average();
    return (p[0]-p[1])/(p[1]*p[0]);
}
