#include "include.h"

uint16_t WantSpeed = 0;
int16_t Pulse = 0;

void E6A2_init(void)      //初始化 编码器
{
    ftm_quad_init(ENCODER_QUAD1);                     //FTM1 正交解码初始化 右
    ftm_quad_init(ENCODER_QUAD2);                     //FTM2 正交解码初始化 左
    LCD_str((Site_t){0, 78},"Left",FCOLOUR,BCOLOUR);
    LCD_str((Site_t){0, 96},"Right",FCOLOUR,BCOLOUR);
    LCD_str((Site_t){0, 110},"Total",FCOLOUR,BCOLOUR);   
}

void E6A2_countPulse(void)
{
    int16 PulseLeft = ftm_quad_get(ENCODER_QUAD2);    
    ftm_quad_clean(ENCODER_QUAD2);
    int16 PulseRight = ftm_quad_get(ENCODER_QUAD1);
    ftm_quad_clean(ENCODER_QUAD1);      
    if(PulseRight < 0) PulseRight = -PulseRight;
    if(PulseLeft < 0) PulseLeft = -PulseLeft;
    Pulse = (PulseLeft+PulseRight)/2;
    LCD_num_BC((Site_t){50, 78},PulseLeft,5,FCOLOUR,BCOLOUR);
    LCD_num_BC((Site_t){50, 94},PulseRight,5,FCOLOUR,BCOLOUR);
    LCD_num_BC((Site_t){50, 110},Pulse,5,FCOLOUR,BCOLOUR);
    int16_t cnt1 = FTM1_CNT;                                     //正反转识别
    int16_t cnt2 = FTM2_CNT;
    if(cnt2>0||cnt1<0) LCD_char((Site_t){100, 110},'-',FCOLOUR,BCOLOUR);
    if(cnt2<0||cnt1>0) LCD_char((Site_t){100, 110},'+',FCOLOUR,BCOLOUR);
}

//**************************************************************************
//                              速度闭环
//**************************************************************************
/* g 全局变量 * f float */

float g_fSpeedControlIntegral;
float g_fSpeedControlOutOld=0, g_fSpeedControlOutNew=0, g_fSpeedControlOut;
void SpeedControl(void)               
{
    float fI;
    float fP, fDelta;
    fDelta = WantSpeed-Pulse; 

    fP = fDelta * P_SPEED;
    fI = fDelta * I_SPEED;
    g_fSpeedControlIntegral += fI; 

    if(g_fSpeedControlIntegral > (0.2*fP))
        g_fSpeedControlIntegral = (0.2*fP);
    else if(g_fSpeedControlIntegral < -(0.2*fP))
        g_fSpeedControlIntegral = -(0.2*fP);
  
    g_fSpeedControlOutOld = g_fSpeedControlOutNew;
//    g_fSpeedControlOutNew = fP;
    g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral;
}

void ovoid_SpeedControl(uint8_t speed_count)   //速度平滑输出
{ 
    float fValue;
    fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld; 
    g_fSpeedControlOut = fValue * (speed_count ) /20 + g_fSpeedControlOutOld;
    if(g_fSpeedControlOut > SpeedControlMAX)
        g_fSpeedControlOut = SpeedControlMAX;
    if(g_fSpeedControlOut < SpeedControlMIN)
        g_fSpeedControlOut = SpeedControlMIN;
}

