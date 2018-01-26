#include "include.h"

/* 中断优先级  PORTA_IRQHandler > PIT0_IRQHandler > DMA0_IRQHandler */

/* 定时器中断 */
volatile uint8_t timer_1ms = 0;            //用作5ms分段控制
volatile uint8_t timer_5ms = 0;            //用作速度环100ms
volatile uint8_t timer_10ms = 0;           //用作10ms转向环
volatile uint8_t OvoidCount_Direct = 0;    //平滑输出计数
volatile uint8_t OvoidCount_Speed = 0; 
uint32_t Timer = 0;               //用作系统计时

void PIT0_IRQHandler(void)
{
    led_turn(LED0);
    AD_Calculate(timer_1ms);     //AD采样加滤波
    
    timer_1ms++;
    if(timer_1ms == 1)           //平滑输出
    {
        OvoidCount_Direct++;
        OvoidCount_Speed++;
        ovoid_SpeedControl(OvoidCount_Speed); 
        ovoid_DirectionControl(OvoidCount_Direct);
    }
    if(timer_1ms == 2)           //速度计算输出
    {
        Speed_Calculate();                    
    }
    if(timer_1ms == 3)           //速度环
    {
        timer_5ms++;
        if(timer_5ms >= 20)
        {
            E6A2_countPulse();
            SpeedControl();
            OvoidCount_Speed = 0;
            timer_5ms = 0;
        }
    }
    if(timer_1ms == 4)           //转向环
    {
        timer_10ms++;
        if(timer_10ms >= 2)
        {
            DirectionControl();
            OvoidCount_Speed = 0;
            timer_10ms = 0;
        }
    }
    if(timer_1ms >= 5)
    {
        Timer++;
        if(Timer >= 120000) Timer = 0;  //10min一次清零
        timer_1ms = 0;                  //5ms一次清零                                           
    } 
    
    PIT_Flag_Clear(PIT0);                       //清中断标志位
}

