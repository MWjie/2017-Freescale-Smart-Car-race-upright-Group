#include "include.h"

/* �ж����ȼ�  PORTA_IRQHandler > PIT0_IRQHandler > DMA0_IRQHandler */

/* ��ʱ���ж� */
volatile uint8_t timer_1ms = 0;            //����5ms�ֶο���
volatile uint8_t timer_5ms = 0;            //�����ٶȻ�100ms
volatile uint8_t timer_10ms = 0;           //����10msת��
volatile uint8_t OvoidCount_Direct = 0;    //ƽ���������
volatile uint8_t OvoidCount_Speed = 0; 
uint32_t Timer = 0;               //����ϵͳ��ʱ

void PIT0_IRQHandler(void)
{
    led_turn(LED0);
    AD_Calculate(timer_1ms);     //AD�������˲�
    
    timer_1ms++;
    if(timer_1ms == 1)           //ƽ�����
    {
        OvoidCount_Direct++;
        OvoidCount_Speed++;
        ovoid_SpeedControl(OvoidCount_Speed); 
        ovoid_DirectionControl(OvoidCount_Direct);
    }
    if(timer_1ms == 2)           //�ٶȼ������
    {
        Speed_Calculate();                    
    }
    if(timer_1ms == 3)           //�ٶȻ�
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
    if(timer_1ms == 4)           //ת��
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
        if(Timer >= 120000) Timer = 0;  //10minһ������
        timer_1ms = 0;                  //5msһ������                                           
    } 
    
    PIT_Flag_Clear(PIT0);                       //���жϱ�־λ
}

