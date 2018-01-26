#include "include.h"
#include "math.h"

volatile uint16_t GYRO_VAL1,GYRO_VAL2,ANGLE_Z_VAL;                 //��������ֵ ���ٶȼ���ֵ
volatile uint16_t x,y,z,gyro1,gyro2;                   //����ADCת����� 
float Gyro_1[5] ,Gyro_2[5] ,Z[5];
int16_t OutData[7] = { 0 };                   //����������

void ADC_init(void)
{
    led_init(LED0);                                //PIT0�ж��õ�LED0
    pit_init_ms(PIT0, 1);                                   //��ʼ��PIT0����ʱʱ��Ϊ�� 1ms
    set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);        //����PIT0���жϸ�λ����Ϊ PIT0_IRQHandler
    NVIC_SetPriority(PIT0_IRQn,1);
//    adc_init(XOUT);          //X����ٶ�   /* �ɲ��� */
//    adc_init(YOUT);          //Y����ٶ�   /* �ɲ��� */  
    adc_init(ZOUT);          //Z����ٶ�
    adc_init(Gyro2);         //�ǼӶ�  Angular2
    adc_init(Gyro1);         //�ǼӶ�  Angular1
//    adc_init(Ang);           //Ӳ���˲�����Ƕ�   /* ���������� */
    gpio_init(PTE26,GPO,HIGH);  //G_SEL����
    enable_irq(PIT0_IRQn);                                   //ʹ��PIT0�ж�   
}

void Rd_Ad_Value(void)
{
//    x = adc_once(XOUT, ADC_12bit);         //X  /* �ɲ��� */
//    y = adc_once(YOUT, ADC_12bit);            // Y  /* �ɲ��� */  
    z = adc_once(ZOUT, ADC_12bit);            // Z 
    gyro1 = adc_once(Gyro1, ADC_12bit);       // gyro1
    gyro2 = adc_once(Gyro2, ADC_12bit);       // gyro2    
    // real_angle = adc_once(Ang,ADC_12bit); // ang//����ʹ������˲�����˲����� Ӳ���ںϽǶ�   
}

void ZeroDrift_correct(void)        //�������Ư��
{
    uint16_t i,num=0;
    uint16_t gyro_cur[3],gyro_last[3],gyro_int[3];
    uint32_t gyro_bias[3];
    for(i=0; i<=1000; i++)
    {
        gyro_cur[0] = adc_once(Gyro1, ADC_12bit);   
        gyro_cur[1] = adc_once(Gyro2, ADC_12bit);
        gyro_cur[2] = adc_once(ZOUT, ADC_12bit);
        DELAY_MS(1);
        if(i == 0)
        {
            gyro_last[0] = gyro_cur[0];
            gyro_last[1] = gyro_cur[1];
            gyro_last[2] = gyro_cur[2];
        }
        for(uint8_t t=0; t<3; t++)
        {
            gyro_int[t] = ABS(gyro_cur[t] - gyro_last[t]);
        }
        if(gyro_int[0]<=50 || gyro_int[1]<=50 || gyro_int[2]<=50)
        {
            gyro_bias[0] += gyro_cur[0];
            gyro_bias[1] += gyro_cur[1];
            gyro_bias[2] += gyro_cur[2];
            num++;
        }
        gyro_last[0] = gyro_cur[0];
        gyro_last[1] = gyro_cur[1];
        gyro_last[2] = gyro_cur[2];
    }
    gyro_bias[0] /= num;
    gyro_bias[1] /= num;
    gyro_bias[2] /= num;
    GYRO_VAL1 = gyro_bias[0];
    GYRO_VAL2 = gyro_bias[1];
    ANGLE_Z_VAL = gyro_bias[2];
}

//**************************************************************************
//                              Kalman�˲�
//**************************************************************************

float angle, angle_dot;           //����������� �Ƕ�/���ٶ�
static float P[2][2] = {
    { 1, 0 },
    { 0, 1 }
};                                //Э����
static float Pdot[4] ={0,0,0,0}; //����Э��������΢�־���
static const uint8_t C_0 = 1;
static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
/**
 * q_bias ���Ź���ֵ��ƫ������Ƴ����������ǵ�Ư����
 * angle_err ʵ��Ƕ��������ǻ��ֽǶȵĲ�ֵ
 * E ����Ĺ�����
 * K_0 ���п��������������һ�����������ڼ������Ź���ֵ
 * K_1 ���п���������ĺ��������ڼ������Ź���ֵ��ƫ��
*/

void Kalman_Filter(float angle_m, float gyro_m)          //gyro_m:gyro_measure
{ 
    angle += (gyro_m-q_bias) * DT;         //�������
    
    Pdot[0] = Q_angle - P[0][1] - P[1][0]; //����������Э�����΢��
    Pdot[1] =- P[1][1];
    Pdot[2] =- P[1][1];
    Pdot[3] = Q_gyro;
    
    P[0][0] += Pdot[0] * DT;               //����������Э����΢�ֵĻ���
    P[0][1] += Pdot[1] * DT;             
    P[1][0] += Pdot[2] * DT;
    P[1][1] += Pdot[3] * DT;
    
    angle_err = angle_m - angle;           //����ƫ��
    
    PCt_0 = C_0 * P[0][0];                 //�������������
    PCt_1 = C_0 * P[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;                           //����������Э�������
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;                
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
        
    angle += K_0 * angle_err;             //����������ŽǶ�ֵ
    q_bias += K_1 * angle_err;            //�������Ź���ֵ��ƫ��
    angle_dot = gyro_m-q_bias;            //�������Ž��ٶ�ֵ
}

void AD_Filter(float Gyro_Average ,float Angle_Z)  //������ת����Ľ��ٶȣ����ٶȼ�ת����ĽǶ�
{
    float Gyro_Now ,angle_offset_vertical;
    Gyro_Now = (GYRO_VAL1 - Gyro_Average) * Gyro_ratio;                //�����ǲɼ����Ľ��ٶȹ�һ��
    angle_offset_vertical = (ANGLE_Z_VAL - Angle_Z) * MMA7361_ratio;  //�����ٶȼƲɼ����ĽǶȹ�һ��
//    angle_offset_vertical = (180/PI)*(double)(atan2(Angle_Z, Angle_X));  //�����нǶȹ�һ
    
    angle_offset_vertical = -angle_offset_vertical;         //����
    Kalman_Filter(angle_offset_vertical, Gyro_Now);
    
    if(angle >= 36) angle = 36;         //�Ƕ��޷�
    if(angle <= -36) angle = -36;
    
    /*****************************���ڿ����Σ�ѡ��ʹ�ã�****************************/
#if  0           //�������˲�
    OutData[0] = (int16_t)angle_dot;
    OutData[1] = (int16_t)Gyro_Now;
    OutData[2] = (int16_t)angle_offset_vertical ;
    OutData[3] = (int16_t)angle;
    OutData[4] = (int16_t)z;
    OutData[5] = (int16_t)gyro1;
    OutData[6] = (int16_t)gyro2;
    vcan_sendware((uint8_t *)OutData,sizeof(OutData));
#endif	
    
    if(angle < 0) LCD_char((Site_t){100, 61},'-',FCOLOUR,BCOLOUR);
    else LCD_char((Site_t){100, 61},'+',FCOLOUR,BCOLOUR);
    LCD_num_BC((Site_t){80, 61},(uint16_t)angle,2,FCOLOUR,BCOLOUR);      
}

//**************************************************************************
//                              ת��ջ�
//**************************************************************************
/* g ȫ�ֱ��� * f float */
float P_Direct, D_Direct;
float GYRO2;
float g_fDirectControlOutOld=0, g_fDirectControlOutNew=0, g_fDirectControlOut;

void DirectionControl(void)    //��ת�� ��ת��
{    
    static int16_t angle_Old, angle_New;
    static int16_t err;
    static float fP,fD_New,fD_Old;
    float Gyro_Now;
    
    angle_New =control_result.angle;
    err= angle_New-angle_Old;
    angle_Old = angle_New;    
    fP = angle_New*P_Direct + err*D_Direct;
    
    Gyro_Now = (GYRO_VAL2-GYRO2)*Gyro_ratio; //��������ת�� ��ת��
    fD_Old = fD_New;
    fD_New = Gyro_Now*Gyro_Now/120000;     
    fD_New = 0.7*fD_New + 0.3*fD_Old;     //��ͨ�˲�
    
    g_fDirectControlOutOld=g_fDirectControlOutNew;
    g_fDirectControlOutNew=fP+fD_New;
//    g_fDirectControlOutNew=fP;
}

void ovoid_DirectionControl(uint8_t direct_count)   //�ٶ�ƽ�����
{ 
    float fValue;
    fValue = g_fDirectControlOutNew - g_fDirectControlOutOld; 
    g_fDirectControlOut = fValue * (direct_count) /2 + g_fDirectControlOutOld;
    if(g_fDirectControlOut > DirectionControlMAX)
        g_fDirectControlOut = DirectionControlMAX;
    if(g_fDirectControlOut < DirectionControlMIN)
        g_fDirectControlOut = DirectionControlMIN;
}

void Speed_Calculate(void)    
{
    float SpeedStart;          //ֱ���ٶ�
    if(Timer <= 1000)           //����ֱ��5s
    {
        SpeedStart = (angle*P_ANGLE) + (angle_dot*D_ANGLE);  //ֱ��ʱ��Ҫ���ٶ�
        Motor_out(SpeedStart, 0);
    }
    else
    {
        SpeedStart = (angle*P_ANGLE - g_fSpeedControlOut) + (angle_dot*D_ANGLE);  //ֱ��ʱ��Ҫ���ٶ�
        Motor_out(SpeedStart, g_fDirectControlOut);
    }
}

void AD_Calculate(uint8_t N_AD)         //�ɼ�
{
    float Gyro1_Final=0.0 ,Gyro2_Final=0.0 ,Z_Final=0.0;
    Rd_Ad_Value();
    if(N_AD < 5)
    {
        Gyro_1[N_AD] = (float)gyro1;
        Gyro_2[N_AD] = (float)gyro1;
        Z[N_AD] = (float)z;
    }
    if(N_AD >= 4)
    {
        for(uint8_t i=0 ;i<5 ;i++)
        {
            Gyro1_Final += Gyro_1[i];
            Gyro2_Final += Gyro_2[i];
            Z_Final += Z[i];
        }
        Gyro1_Final /= 5.0;
        Gyro2_Final /= 5.0;
        Z_Final /= 5.0;
        GYRO2 = Gyro2_Final;
        AD_Filter(Gyro1_Final, Z_Final);  //ʹ���㷨�����˲�
    }    
}

double weight[CAMERA_H] = {0};
static const double weight_003_1[];
static const double weight_004_1[];
static const double weight_005_1[];
static const double weight_006_1[];
static const double weight_007_1[];
static const double weight_008_1[];
static const double weight_009_1[];
static const double weight_010_1[];
static const double weight_011_1[];
static const double weight_012_1[];
static const double weight_013_1[];
static const double weight_014_1[];
static const double weight_015_1[];
static const double weight_016_1[];

void read_DIPswitch(void)
{
    gpio_init(PTC16,GPI,0);
    gpio_init(PTC17,GPI,0);
    gpio_init(PTC18,GPI,0);
    gpio_init(PTC19,GPI,0);
    uint8_t mode;
    mode=GPIO_GET_NBIT(4,PTC16);
    switch(mode)
    {
        /* ��Բ�� */
        case HIGH_SPEED_Ring:
            P_Direct = 130; D_Direct = 0;
            WantSpeed = 2500;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_013_1[i];
            LCD_str((Site_t){0, 61},"High_R",FCOLOUR,BCOLOUR);
            break;
        
        case MEDIUM_SPEED_Ring:
            P_Direct = 75; D_Direct = 0;
            WantSpeed = 2000;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_005_1[i];
            LCD_str((Site_t){0, 61},"Medium_R",FCOLOUR,BCOLOUR);
            break;

        case LOW_SPEED_Ring:
            P_Direct = 45; D_Direct = 12;
            WantSpeed = 1500;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_003_1[i];
            LCD_str((Site_t){0, 61},"Low_R",FCOLOUR,BCOLOUR);
            break;
            
        case TEST_Ring:
            P_Direct = 170; D_Direct = 0;
            WantSpeed = 2800;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_014_1[i];
            LCD_str((Site_t){0, 61},"Test_R",FCOLOUR,BCOLOUR);
            break;
            
        /* ��Բ�� */   
        case HIGH_SPEED:
            P_Direct = 110; D_Direct = 0;
            WantSpeed = 2500;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_009_1[i];
            LCD_str((Site_t){0, 61},"High",FCOLOUR,BCOLOUR);
            break;
        
        case MEDIUM_SPEED:
            P_Direct = 65; D_Direct = 0;
            WantSpeed = 2000;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_004_1[i];
            LCD_str((Site_t){0, 61},"Medium",FCOLOUR,BCOLOUR);
            break;

        case LOW_SPEED:
            P_Direct = 45; D_Direct = 12;
            WantSpeed = 1500;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_003_1[i];
            LCD_str((Site_t){0, 61},"Low",FCOLOUR,BCOLOUR);
            break;
            
        case TEST:
            P_Direct = 215; D_Direct = 10;
            WantSpeed = 3000;
            for(uint8_t i=0; i<CAMERA_H; i++)
                weight[i] = weight_016_1[i];
            LCD_str((Site_t){0, 61},"Test Mode",FCOLOUR,BCOLOUR);
            break;
            
        default:
            LCD_str((Site_t){0, 61},"Error",FCOLOUR,BCOLOUR);
            while(1);
    }
}
/* Ȩֵ�� */
static const double weight_003_1[CAMERA_H]={
2.77,2.74,2.71,2.68,2.65,2.62,2.59,2.56,2.53,2.5,
2.47,2.44,2.41,2.38,2.35,2.32,2.29,2.26,2.23,2.2,
2.17,2.14,2.11,2.08,2.05,2.02,1.99,1.96,1.93,1.9,
1.87,1.84,1.81,1.78,1.75,1.72,1.69,1.66,1.63,1.6,
1.57,1.54,1.51,1.48,1.45,1.42,1.39,1.36,1.33,1.3,
1.27,1.24,1.21,1.18,1.15,1.12,1.09,1.06,1.03,1
    };//y=0.03x+1

static const double weight_004_1[CAMERA_H]={
3.36,3.32,3.28,3.24,3.2,3.16,3.12,3.08,3.04,3,
2.96,2.92,2.88,2.84,2.8,2.76,2.72,2.68,2.64,2.6,
2.56,2.52,2.48,2.44,2.4,2.36,2.32,2.28,2.24,2.2,
2.16,2.12,2.08,2.04,2,1.96,1.92,1.88,1.84,1.8,
1.76,1.72,1.68,1.64,1.6,1.56,1.52,1.48,1.44,1.4,
1.36,1.32,1.28,1.24,1.2,1.16,1.12,1.08,1.04,1
    };//y=0.04x+1

static const double weight_005_1[CAMERA_H]={
3.95,3.9,3.85,3.8,3.75,3.7,3.65,3.6,3.55,3.5,
3.45,3.4,3.35,3.3,3.25,3.2,3.15,3.1,3.05,3,
2.95,2.9,2.85,2.8,2.75,2.7,2.65,2.6,2.55,2.5,
2.45,2.4,2.35,2.3,2.25,2.2,2.15,2.1,2.05,2,
1.95,1.9,1.85,1.8,1.75,1.7,1.65,1.6,1.55,1.5,
1.45,1.4,1.35,1.3,1.25,1.2,1.15,1.1,1.05,1
    };//y=0.05x+1

static const double weight_006_1[CAMERA_H]={
4.54,4.48,4.42,4.36,4.3,4.24,4.18,4.12,4.06,4,
3.94,3.88,3.82,3.76,3.7,3.64,3.58,3.52,3.46,3.4,
3.34,3.28,3.22,3.16,3.1,3.04,2.98,2.92,2.86,2.8,
2.74,2.68,2.62,2.56,2.5,2.44,2.38,2.32,2.26,2.2,
2.14,2.08,2.02,1.96,1.9,1.84,1.78,1.72,1.66,1.6,
1.54,1.48,1.42,1.36,1.3,1.24,1.18,1.12,1.06,1
    };//y=0.06x+1

static const double weight_007_1[CAMERA_H]={
5.13,5.06,4.99,4.92,4.85,4.78,4.71,4.64,4.57,4.5,
4.43,4.36,4.29,4.22,4.15,4.08,4.01,3.94,3.87,3.8,
3.73,3.66,3.59,3.52,3.45,3.38,3.31,3.24,3.17,3.1,
3.03,2.96,2.89,2.82,2.75,2.68,2.61,2.54,2.47,2.4,
2.33,2.26,2.19,2.12,2.05,1.98,1.91,1.84,1.77,1.7,
1.63,1.56,1.49,1.42,1.35,1.28,1.21,1.14,1.07,1
    };//y=0.07x+1

static const double weight_008_1[CAMERA_H]={
5.72,5.64,5.56,5.48,5.4,5.32,5.24,5.16,5.08,5,
4.92,4.84,4.76,4.68,4.6,4.52,4.44,4.36,4.28,4.2,
4.12,4.04,3.96,3.88,3.8,3.72,3.64,3.56,3.48,3.4,
3.32,3.24,3.16,3.08,3,2.92,2.84,2.76,2.68,2.6,
2.52,2.44,2.36,2.28,2.2,2.12,2.04,1.96,1.88,1.8,
1.72,1.64,1.56,1.48,1.4,1.32,1.24,1.16,1.08,1
    };//y=0.08x+1

static const double weight_009_1[CAMERA_H]={
6.31,6.22,6.13,6.04,5.95,5.86,5.77,5.68,5.59,5.5,
5.41,5.32,5.23,5.14,5.05,4.96,4.87,4.78,4.69,4.6,
4.51,4.42,4.33,4.24,4.15,4.06,3.97,3.88,3.79,3.7,
3.61,3.52,3.43,3.34,3.25,3.16,3.07,2.98,2.89,2.8,
2.71,2.62,2.53,2.44,2.35,2.26,2.17,2.08,1.99,1.9,
1.81,1.72,1.63,1.54,1.45,1.36,1.27,1.18,1.09,1
    };//y=0.09x+1

static const double weight_010_1[CAMERA_H]={
6.9,6.8,6.7,6.6,6.5,6.4,6.3,6.2,6.1,6,
5.9,5.8,5.7,5.6,5.5,5.4,5.3,5.2,5.1,5,
4.9,4.8,4.7,4.6,4.5,4.4,4.3,4.2,4.1,4,
3.9,3.8,3.7,3.6,3.5,3.4,3.3,3.2,3.1,3,
2.9,2.8,2.7,2.6,2.5,2.4,2.3,2.2,2.1,2,
1.9,1.8,1.7,1.6,1.5,1.4,1.3,1.2,1.1,1
    };//y=0.1x+1

static const double weight_011_1[CAMERA_H]={
7.49,7.38,7.27,7.16,7.05,6.94,6.83,6.72,6.61,6.5,
6.39,6.28,6.17,6.06,5.95,5.84,5.73,5.62,5.51,5.4,
5.29,5.18,5.07,4.96,4.85,4.74,4.63,4.52,4.41,4.3,
4.19,4.08,3.97,3.86,3.75,3.64,3.53,3.42,3.31,3.2,
3.09,2.98,2.87,2.76,2.65,2.54,2.43,2.32,2.21,2.1,
1.99,1.88,1.77,1.66,1.55,1.44,1.33,1.22,1.11,1
    };//y=0.11x+1

static const double weight_012_1[CAMERA_H]={
8.08,7.96,7.84,7.72,7.6,7.48,7.36,7.24,7.12,7,
6.88,6.76,6.64,6.52,6.4,6.28,6.16,6.04,5.92,5.8,
5.68,5.56,5.44,5.32,5.2,5.08,4.96,4.84,4.72,4.6,
4.48,4.36,4.24,4.12,4,3.88,3.76,3.64,3.52,3.4,
3.28,3.16,3.04,2.92,2.8,2.68,2.56,2.44,2.32,2.2,
2.08,1.96,1.84,1.72,1.6,1.48,1.36,1.24,1.12,1
    };//y=0.12x+1

static const double weight_013_1[CAMERA_H]={
8.67,8.54,8.41,8.28,8.15,8.02,7.89,7.76,7.63,7.5,
7.37,7.24,7.11,6.98,6.85,6.72,6.59,6.46,6.33,6.2,
6.07,5.94,5.81,5.68,5.55,5.42,5.29,5.16,5.03,4.9,
4.77,4.64,4.51,4.38,4.25,4.12,3.99,3.86,3.73,3.6,
3.47,3.34,3.21,3.08,2.95,2.82,2.69,2.56,2.43,2.3,
2.17,2.04,1.91,1.78,1.65,1.52,1.39,1.26,1.13,1
    };//y=0.13x+1

static const double weight_014_1[CAMERA_H]={
9.26,9.12,8.98,8.84,8.7,8.56,8.42,8.28,8.14,8,
7.86,7.72,7.58,7.44,7.3,7.16,7.02,6.88,6.74,6.6,
6.46,6.32,6.18,6.04,5.9,5.76,5.62,5.48,5.34,5.2,
5.06,4.92,4.78,4.64,4.5,4.36,4.22,4.08,3.94,3.8,
3.66,3.52,3.38,3.24,3.1,2.96,2.82,2.68,2.54,2.4,
2.26,2.12,1.98,1.84,1.7,1.56,1.42,1.28,1.14,1
    };//y=0.14x+1

static const double weight_015_1[CAMERA_H]={
9.85,9.7,9.55,9.4,9.25,9.1,8.95,8.8,8.65,8.5,
8.35,8.2,8.05,7.9,7.75,7.6,7.45,7.3,7.15,7,
6.85,6.7,6.55,6.4,6.25,6.1,5.95,5.8,5.65,5.5,
5.35,5.2,5.05,4.9,4.75,4.6,4.45,4.3,4.15,4,
3.85,3.7,3.55,3.4,3.25,3.1,2.95,2.8,2.65,2.5,
2.35,2.2,2.05,1.9,1.75,1.6,1.45,1.3,1.15,1
    };//y=0.15x+1

static const double weight_016_1[CAMERA_H]={
10.44,10.28,10.12,9.96,9.8,9.64,9.48,9.32,9.16,9,
8.84,8.68,8.52,8.36,8.2,8.04,7.88,7.72,7.56,7.4,
7.24,7.08,6.92,6.76,6.6,6.44,6.28,6.12,5.96,5.8,
5.64,5.48,5.32,5.16,5,4.84,4.68,4.52,4.36,4.2,
4.04,3.88,3.72,3.56,3.4,3.24,3.08,2.92,2.76,2.6,
2.44,2.28,2.12,1.96,1.8,1.64,1.48,1.32,1.16,1
    };//y=0.16x+1
