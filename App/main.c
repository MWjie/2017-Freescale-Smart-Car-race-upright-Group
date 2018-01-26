 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,山外科技
  *     All rights reserved.
  *     技术讨论：山外论坛 http://www.vcan123.com
  *
  *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
  *     修改内容时必须保留山外科技的版权声明。
  *
  * @file       main.c
  * @brief      山外K60 平台主程序
  * @author     山外科技
  * @version    v5.0
  * @date       2013-08-28
  */

#include "include.h"


void main(void)
{
    DisableInterrupts;                             //禁止总中断
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);//中断优先级分组
/******************** 初始化操作 *****************/    
    ADC_init();
    LCD_init();
    discern_init();
    Motor_init();
    E6A2_init();
/*************************************************/     
    ZeroDrift_correct();                           //校正零点漂移
    read_DIPswitch();                              //读取拨码开关
    EnableInterrupts;                              //中断允许
    
    while(1)
    {
        control_result = discern();
        
    }
}

