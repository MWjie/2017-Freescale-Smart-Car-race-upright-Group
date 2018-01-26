 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,ɽ��Ƽ�
  *     All rights reserved.
  *     �������ۣ�ɽ����̳ http://www.vcan123.com
  *
  *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
  *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
  *
  * @file       main.c
  * @brief      ɽ��K60 ƽ̨������
  * @author     ɽ��Ƽ�
  * @version    v5.0
  * @date       2013-08-28
  */

#include "include.h"


void main(void)
{
    DisableInterrupts;                             //��ֹ���ж�
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);//�ж����ȼ�����
/******************** ��ʼ������ *****************/    
    ADC_init();
    LCD_init();
    discern_init();
    Motor_init();
    E6A2_init();
/*************************************************/     
    ZeroDrift_correct();                           //У�����Ư��
    read_DIPswitch();                              //��ȡ���뿪��
    EnableInterrupts;                              //�ж�����
    
    while(1)
    {
        control_result = discern();
        
    }
}

