/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright 2018 - 2020 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hitsic_common.h"

/** HITSIC_Module_DRV */
#include "drv_ftfx_flash.hpp"
#include "drv_disp_ssd1306.hpp"
#include "drv_imu_invensense.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"

/** HITSIC_Module_SYS */
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sys_uartmgr.hpp"
#include "cm_backtrace.h"
//#include "easyflash.h"

/** HITSIC_Module_APP */
#include "app_menu.hpp"
#include "app_svbmp.hpp"

/** FATFS */
#include "ff.h"
#include "sdmmc_config.h"
FATFS fatfs;                                   //逻辑驱动器的工作区

#include "sc_adc.h"
#include "sc_ftm.h"

/** HITSIC_Module_TEST */
#include "drv_cam_zf9v034_test.hpp"
#include "app_menu_test.hpp"
#include "drv_imu_invensense_test.hpp"
#include "sys_fatfs_test.hpp"
#include "sys_fatfs_diskioTest.hpp"

/** SCLIB_TEST */
#include "sc_test.hpp"

#include "image.h"
#include"team_menu.hpp"

cam_zf9v034_configPacket_t cameraCfg;
dmadvp_config_t dmadvpCfg;
dmadvp_handle_t dmadvpHandle;
disp_ssd1306_frameBuffer_t *dispBuffer;
inv::i2cInterface_t imu_i2c(nullptr, IMU_INV_I2cRxBlocking, IMU_INV_I2cTxBlocking);
inv::mpu6050_t imu_6050(imu_i2c);
uint8_t mode_flag = 0;//状态切换标志位变量
uint8_t *p_mflag = NULL;//状态切换指针
uint8_t prem_flag = 0;//状态切换标志位变量2，previous标志位
void run_car(dmadvp_handle_t *dmadvpHandle,disp_ssd1306_frameBuffer_t *dispBuffer);//摄像头跑车函数
void elec_runcar(void);//电磁跑车函数
void mode_switch(void);//模式切换中断回调函数
void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);//摄像头初始化回调函数
void main(void)
{
    /** 初始化阶段，关闭总中断 */
    HAL_EnterCritical();
    /** 初始化时钟 */
    RTECLK_HsRun_180MHz();
    /** 初始化引脚路由 */
    RTEPIN_Basic();
    RTEPIN_Digital();
    RTEPIN_Analog();
    RTEPIN_LPUART0_DBG();
    RTEPIN_UART0_WLAN();
    /** 初始化外设 */
    RTEPIP_Basic();
    RTEPIP_Device();
    /** 初始化调试串口 */
    DbgConsole_Init(0U, 921600U, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Welcome to HITSIC !\n");
    PRINTF("GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    /** 初始化CMBackTrace */
    cm_backtrace_init("HITSIC_MK66F18", "2020-v3.0", "v4.1.1");
    /** 初始化ftfx_Flash */
    FLASH_SimpleInit();
    /** 初始化EasyFlash */
    //easyflash_init();
    /** 初始化PIT中断管理器 */
    p_mflag = &mode_flag;
    pitMgr_t::init();
    pitMgr_t::insert(5U, 1U, Motor_ctr, pitMgr_t::enable);//电机中断
    pitMgr_t::insert(20U, 3U, servo, pitMgr_t::enable);//舵机中断
    pitMgr_t::insert(1000U, 7U, mode_switch, pitMgr_t::enable);//状态切换
    /** 初始化I/O中断管理器 */
    extInt_t::init();
    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    //DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
   /** 初始化菜单 */
        MENU_Init();
        MENU_Data_NvmReadRegionConfig();
        MENU_Data_NvmRead(menu_currRegionNum);
   /** 菜单挂起 */
        MENU_Suspend();
    /** 初始化摄像头 */
        /*摄像头在主函数中初始化*/
    //TODO: 在这里初始化摄像头
    /** 初始化IMU */
    //TODO: 在这里初始化IMU（MPU6050）
    /** 菜单就绪 */
    //MENU_Resume();
    /** 控制环初始化 */
    //TODO: 在这里初始化控制环


    /** 初始化结束，开启总中断 */
    HAL_ExitCritical();
    //SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,7.45);

    //float f = arm_sin_f32(0.6f);//暂时不知道功能
    while (true)
    {
        switch(mode_flag)//菜单模式
        {
        case 0x00: {
            {
                MENU_Resume();
            while(true)
             {
                prem_flag = mode_flag;
                Get_erro();
                if(prem_flag != mode_flag) break;
              }
            }
                break;
        }break;
        case 0x01://摄像头跑车模式
        {
            MENU_Suspend();
            CAM_ZF9V034_GetDefaultConfig(&cameraCfg);                                   //设置摄像头配置
               CAM_ZF9V034_CfgWrite(&cameraCfg);                                   //写入配置
               CAM_ZF9V034_GetReceiverConfig(&dmadvpCfg, &cameraCfg);    //生成对应接收器的配置数据，使用此数据初始化接受器并接收图像数据。
               DMADVP_Init(DMADVP0, &dmadvpCfg);
               DMADVP_TransferCreateHandle(&dmadvpHandle, DMADVP0,CAM_ZF9V034_DmaCallback);
               uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
               dispBuffer = new disp_ssd1306_frameBuffer_t;
               //uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];
               DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer0);
               //DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer1);
           DMADVP_TransferStart(DMADVP0, &dmadvpHandle);
        while(true)
            {
                prem_flag = mode_flag;
                run_car(&dmadvpHandle,dispBuffer);
                if(prem_flag != mode_flag) break;
            }
        delete imageBuffer0;
        delete &dispBuffer;

        }
        break;
        case 0x02://百年校庆图标模式
                {
                    MENU_Suspend();
                    DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
                while(true)
                 {
                    prem_flag = mode_flag;
                    SDK_DelayAtLeastUs(2000000,180*1000*1000);
                    if(prem_flag != mode_flag) break;
                  }
                }
                    break;
        case 0x03://电磁跑车模式
                {
                    MENU_Suspend();//延迟发车
                    DISP_SSD1306_Fill(0);
                    SDK_DelayAtLeastUs(5000000,180*1000*1000);
                    delay_runcar = 1;
                while(true)
                 {
                    prem_flag = mode_flag;
                    elec_runcar();
                    DISP_SSD1306_Printf_F6x8(30,5,"%c","elecmode");
                    if(prem_flag != mode_flag) break;
                  }
                }
                delay_runcar = 0;
                    break;
        default: break;//其他模式，待定
        }
        //TODO: 在这里添加车模保护代码
    }
}


void MENU_DataSetUp(void)
{
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "EXAMPLE", 0, 0));
    our_menu_test(menu_menuRoot);


    //TODO: 在这里添加子菜单和菜单项
}
void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    dmadvp_handle_t *dmadvpHandle = (dmadvp_handle_t*)userData;
    DMADVP_EdmaCallbackService(dmadvpHandle, transferDone);
    //TODO: 补完本回调函数

    //TODO: 添加图像处理（转向控制也可以写在这里）
}
void run_car(dmadvp_handle_t *dmadvpHandle,disp_ssd1306_frameBuffer_t *dispBuffer)
{
    while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, dmadvpHandle,&fullBuffer));
                     THRE();
                     //head_clear();
                     image_main();
                     servo_pid();
                             dispBuffer->Clear();
                             const uint8_t imageTH = 120;
                             for (int i = 0; i < cameraCfg.imageRow; i += 2)
                             {
                                 int16_t imageRow = i >> 1;//除以2 为了加速;
                                 for (int j = 0; j < cameraCfg.imageCol; j += 2)
                                 {
                                     int16_t dispCol = j >> 1;
                                     if (IMG[i][j]>imageTH)//fullBuffer[i * cameraCfg.imageCol + j] >
                                     {
                                         dispBuffer->SetPixelColor(dispCol, imageRow, 1);
                                     }
                                 }
                             }

                             DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
                             DMADVP_TransferSubmitEmptyBuffer(DMADVP0, dmadvpHandle, fullBuffer);
                             DMADVP_TransferStart(DMADVP0, dmadvpHandle);
}
void elec_runcar(void)//电磁跑车函数
{
    servo_pid();
}
void mode_switch(void)//模式切换中断回调函数
{
    (GPIO_PinRead(GPIOA, 9) == 0)? ((*p_mflag) |= 0x01):((*p_mflag) &= 0xfe);
    (GPIO_PinRead(GPIOA,11) == 0)? ((*p_mflag) |= 0x02):((*p_mflag) &= 0xfd);
    (GPIO_PinRead(GPIOA,13) == 0)? ((*p_mflag) |= 0x04):((*p_mflag) &= 0xfb);
    (GPIO_PinRead(GPIOA,15) == 0)? ((*p_mflag) |= 0x08):((*p_mflag) &= 0xf7);
}
