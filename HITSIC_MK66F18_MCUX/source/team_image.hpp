#ifndef TEAM_IMAGE_HPP_
#define TEAM_IMAGE_HPP_
#include "hitsic_common.h"
#include "team_menu_test.hpp"
#include "team_ctr.hpp"
#include "image.h"

void team_getmidline(void)
{
    while(GPIO_PinRead(GPIOE, 10) != 0)
    {
    /*DMADVP_TransferSubmitEmptyBuffer(DMADVP0, dmadvpHandle, fullBuffer);
    DMADVP_TransferStart(DMADVP0,dmadvpHandle);*/
    THRE();
    head_clear();
    image_main();
    }
}


#endif
