#ifndef MENU_TEST_HPP_
#define MENU_TEST_HPP_

#include "hitsic_common.h"
#include "app_menu.hpp"
#include "team_ctr.hpp"
#include "image.h"//图像处理代码库
#include "team_elec.hpp"
/**********************************************************************************************************************
*  @brief      摄像头打印赛道函数暂时没有用
*  @return     void
*  @since      v1.1
*  Sample usage:                 team_camtoled();
**********************************************************************************************************************/
void team_camtoled(void*)
{
    cam_zf9v034_configPacket_t cameraCfg;
           CAM_ZF9V034_GetDefaultConfig(&cameraCfg);                                   //设置摄像头配置
           CAM_ZF9V034_CfgWrite(&cameraCfg);                                   //写入配置
           dmadvp_config_t dmadvpCfg;
           CAM_ZF9V034_GetReceiverConfig(&dmadvpCfg, &cameraCfg);    //生成对应接收器的配置数据，使用此数据初始化接受器并接收图像数据。
           DMADVP_Init(DMADVP0, &dmadvpCfg);
           dmadvp_handle_t dmadvpHandle;
           DMADVP_TransferCreateHandle(&dmadvpHandle, DMADVP0, CAM_ZF9V034_UnitTestDmaCallback);
           uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
           //uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];
           uint8_t *fullBuffer = NULL;
           disp_ssd1306_frameBuffer_t *dispBuffer = new disp_ssd1306_frameBuffer_t;
           DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer0);
           //DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer1);
           DMADVP_TransferStart(DMADVP0, &dmadvpHandle);
           while(true)
           {
               while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));


                  dispBuffer->Clear();
                          const uint8_t imageTH = 100;
                          for (int i = 0; i < cameraCfg.imageRow; i += 2)
                          {
                              int16_t imageRow = i >> 1;//除以2 为了加速;
                              for (int j = 0; j < cameraCfg.imageCol; j += 2)
                              {
                                  int16_t dispCol = j >> 1;
                                  if (fullBuffer[i * cameraCfg.imageCol + j] > imageTH)
                                  {
                                      dispBuffer->SetPixelColor(dispCol, imageRow, 1);
                                  }
                              }
                          }

                          DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
                          DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, fullBuffer);
                          //DMADVP_TransferStart(DMADVP0,&dmadvpHandle);
                          if(GPIO_PinRead(GPIOE, 10) == 0) break;
           }
}
/**********************************************************************************************************************
*  @brief                        菜单构建函数
*  @param      *b                menu_list_t指针，待插入菜单父菜单节点
*  @return                       void insert后的菜单
*  @since                        v1.1
*  Sample usage:                 our_menu_test(menu_menuRoot);
**********************************************************************************************************************/
void our_menu_test(menu_list_t *menu)
{
    static menu_list_t *Motor = MENU_ListConstruct("Motorpar", 10, menu);
    assert(Motor);
    static menu_list_t *Servo = MENU_ListConstruct("Servopar", 10, menu);
    assert(Servo);
    static menu_list_t *camera = MENU_ListConstruct("Camerapara", 10, menu);
    assert(Servo);
    MENU_ListInsert(menu, MENU_ItemConstruct(menuType, Motor, "Motorpar", 0, 0));
    {
        MENU_ListInsert(Motor, MENU_ItemConstruct(nullType, NULL, "Motorpara", 0, 0));
        MENU_ListInsert(Motor,
                MENU_ItemConstruct(variType, &(c_data[0].Motorspeed[0]), "Motor_speed", 0, menuItem_data_region|menuItem_dataExt_HasMinMax));
        MENU_ListInsert(Motor,
                MENU_ItemConstruct(variType, &mot_left, "Motor_left", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(Motor,
                MENU_ItemConstruct(variType, &mot_right, "Motor_right", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(Motor,
                MENU_ItemConstruct(varfType, &M_right_drs, "MoDres", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(Motor,
                MENU_ItemConstruct(varfType, &M_right_pwm, "MoPwm", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(Motor,
                        MENU_ItemConstruct(varfType, &(c_data[0].M_Kp), "M_kp",1, menuItem_data_region));
        MENU_ListInsert(Motor,
                        MENU_ItemConstruct(varfType, &(c_data[0].M_Ki), "M_Ki",2, menuItem_data_region));
    }
    MENU_ListInsert(menu, MENU_ItemConstruct(menuType, Servo, "servopara", 0, 0));
    {   MENU_ListInsert(Servo, MENU_ItemConstruct(nullType, NULL, "servo", 0, 0));
        MENU_ListInsert(Servo,
                MENU_ItemConstruct(varfType, &(c_data[0].Kp), "pid[0]Kp",11, menuItem_data_region));
        MENU_ListInsert(Servo,
                MENU_ItemConstruct(varfType, &(c_data[0].Kd), "pid[0]Kd", 12, menuItem_data_region));
        MENU_ListInsert(Servo,
                     MENU_ItemConstruct(varfType, &(c_data[0].servo_mid), "servo_mid", 13, menuItem_data_region));
        MENU_ListInsert(Servo,
                     MENU_ItemConstruct(varfType, &(c_data[0].servo_pwm), "servo_pwm", 14, menuItem_data_region));
    }
    MENU_ListInsert(menu, MENU_ItemConstruct(menuType, camera, "camerapar", 0, 0));
    {
        MENU_ListInsert(camera, MENU_ItemConstruct(nullType, NULL, "camerorelec", 0, 0));
        MENU_ListInsert(camera,
                       MENU_ItemConstruct(variType, &(foresight), "foresight",21, menuItem_data_region));
        MENU_ListInsert(camera,
                       MENU_ItemConstruct(variType, &(threshold), "threshold",22, menuItem_data_region));
        MENU_ListInsert(camera, MENU_ItemConstruct(nullType, NULL, "elec", 0, 0));
        /*MENU_ListInsert(TestList,
                               MENU_ItemConstruct(varfType, &(ADC[0]), "adc0",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));*/
        MENU_ListInsert(camera,
                               MENU_ItemConstruct(variType, &(ADC[1]), "adc1lef",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
       /* MENU_ListInsert(TestList,
                               MENU_ItemConstruct(varfType, &(ADC[2]), "adc2",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(TestList,
                               MENU_ItemConstruct(varfType, &(ADC[3]), "adc3",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(TestList,
                               MENU_ItemConstruct(varfType, &(ADC[4]), "adc4",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
              MENU_ListInsert(TestList,
                               MENU_ItemConstruct(varfType, &(ADC[5]), "adc5",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
              MENU_ListInsert(TestList,
                               MENU_ItemConstruct(varfType, &(ADC[6]), "adc6",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));*/
                     MENU_ListInsert(camera,
                               MENU_ItemConstruct(variType, &(ADC[7]), "adc7rig",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
                     //MENU_ListInsert(TestList,
                                              //MENU_ItemConstruct(varfType, &(error_n), "erron",0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    }
}
#endif // ! HITSIC_USE_APP_MENU
