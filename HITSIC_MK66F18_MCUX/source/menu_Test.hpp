#ifndef MENU_TEST_HPP_
#define MENU_TEST_HPP_

#include "hitsic_common.h"
#include "app_menu.hpp"

int speed1[3]={1,2,3};
float speed2[3]={0.1f,0.2f,0.3f};
void our_menu_test(menu_list_t *menu)
{
    static menu_list_t *TestList = MENU_ListConstruct("9th_testMenu", 20, menu);
    assert(TestList);
    MENU_ListInsert(menu, MENU_ItemConstruct(menuType, TestList, "9th_testMenu", 0, 0));
    {
        MENU_ListInsert(TestList, MENU_ItemConstruct(nullType, NULL, "int_data", 0, 0));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(variType, &speed1[0], "int0", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(variType, &speed1[1], "int1", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(variType, &speed1[2], "int2", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));

        MENU_ListInsert(TestList, MENU_ItemConstruct(nullType, NULL, "float_data", 0, 0));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(varfType, &speed2[0], "float0", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(varfType, &speed2[1], "float1", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(TestList,
                MENU_ItemConstruct(varfType, &speed2[2], "float2", 0,  menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    }
}
#endif // ! HITSIC_USE_APP_MENU
