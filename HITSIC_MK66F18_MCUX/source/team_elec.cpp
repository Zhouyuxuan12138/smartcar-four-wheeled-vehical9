#include "team_elec.hpp"

#define SampleTimes 16
#define MinLVGot 5

uint32_t LV_Temp[2][SampleTimes];
uint32_t LV[2]={0,0};
uint32_t AD[2]={0,0};

void LV_Sample(void)                             // ad采集函数
{
  for(uint8_t i=0 ; i < SampleTimes; i++)
  {
    /*获取采样初值*/
    LV_Temp[0][i]=SCADC_Sample(ADC0,0,23);//这里只有两个电感，所以这个只有两行
    LV_Temp[1][i]=SCADC_Sample(ADC0,0,12);
  }
}

void LV_Get_Val(void)//约0.3mS                  //对采集的值滤波
{
 // 有时会在0-65535(255)间跳动
  for(uint8_t i=0;i<2;i++)
  {
    for(uint8_t j=0; j < SampleTimes;j++)
    {
         if(LV_Temp[i][j]>500)//剔除毛刺信号
         {
             LV_Temp[i][j] = 500;
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
void Normalized(void)
{
    for(uint8_t i=0;i<2;i++)
    {
        uint32_t max = LV_Temp[i][SampleTimes-1];
        uint32_t min = LV_Temp[i][0];
        for(uint8_t j=0; j < SampleTimes;j++)
        {
            uint32_t thisdata = LV_Temp[i][j];
            LV_Temp[i][j] = ((thisdata-min)/(max - min));
        }
    }

}
uint32_t *LV_average(void)
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
uint32_t Get_erro(void)
{
    uint32_t *p;
    p = LV_average();
    return(p[0]-p[1]);
}
