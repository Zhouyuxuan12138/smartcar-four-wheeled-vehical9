#ifndef MENU_TEST_HPP_
#define MENU_TEST_HPP_

#include "hitsic_common.h"
#include "app_menu.hpp"
#include "team_ctr.hpp"
#include "image.h"//图像处理代码库

int speed1[3]={1,2,3};
float speed2[3]={0.1f,0.2f,0.3f};
pid pid1[3];
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
           mid_line[100] = 94;
           PID_init(pid1,mid_line);//pid 初始化
           while(true)
           {
               while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));


                  dispBuffer->Clear();
                          const uint8_t imageTH = 100;
                          for (int i = 0; i < cameraCfg.imageRow; i += 2)
                          {
                              int16_t imageRow = i >> 1;//除以2 为了加速;
                              int16_t dispRow = (imageRow / 8) + 1, dispShift = (imageRow % 8);
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
                          DISP_SSD1306_Printf_F6x8(5,10,"%d",mid_line[100])
                          DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, fullBuffer);
                          DMADVP_TransferStart(DMADVP0,&dmadvpHandle);
                          get_mid_line();
                          pd_ctr(pid1);
                          pd_generate_pulse(pid1);
                          if(GPIO_PinRead(GPIOE, 10) == 0) break;
           }
}

void our_menu_test(menu_list_t *menu)
{
    static menu_list_t *TestList = MENU_ListConstruct("9th_testMenu", 20, menu);
    assert(TestList);
    MENU_ListInsert(menu, MENU_ItemConstruct(menuType, TestList, "9th_testMenu", 0, 0));
    {
        MENU_ListInsert(TestList, MENU_ItemConstruct(nullType, NULL, "Motor speed", 0, 0));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(variType, &Motorspeed[0], "Motorspeed", 10, menuItem_data_global|menuItem_dataExt_HasMinMax));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(variType, &speed1[1], "int1", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(variType, &speed1[2], "int2", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));

        MENU_ListInsert(TestList, MENU_ItemConstruct(nullType, NULL, "float_data", 0, 0));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(varfType, &speed2[0], "float0",11, menuItem_data_global));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(varfType, &speed2[1], "float1", 12, menuItem_data_global));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(varfType, &speed2[2], "float2", 1,  menuItem_data_region));
        MENU_ListInsert(TestList, MENU_ItemConstruct(nullType, NULL, "function", 0, 0));
        MENU_ListInsert(TestList, MENU_ItemConstruct(procType,team_camtoled, "cam_toled", 0, menuItem_proc_uiDisplay));
    }
}
#endif // ! HITSIC_USE_APP_MENU
