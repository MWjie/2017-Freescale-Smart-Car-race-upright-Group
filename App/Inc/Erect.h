#ifndef __Erect_H__
#define __Erect_H__

#define XOUT    ADC1_DM0
#define YOUT    ADC0_SE16
#define ZOUT    ADC0_SE17

#define Gyro1   ADC1_DP0
#define Gyro2   ADC1_SE16
#define Ang     ADC0_SE18

/***********************直立控制参数********************/

#define Gyro_ratio                   0.28    //3.3*1000/4096/0.67/10 0.12
#define MMA7361_ratio                0.36    //1315(60c)  1055(125c)  90/260         1338   825   180/
/* 卡尔曼滤波参数 */
#define Q_angle                      0.001     //0.001//角度过程噪声的协方差  
#define Q_gyro                       0.003     //0.003//角速度过程噪声的协方差
#define R_angle                      0.69    //0.5//测量噪声的协方差（即是测量偏差）
#define DT                           0.005
/* PD角度控制参数 */   
#define P_ANGLE                      1200
#define D_ANGLE                      40

#define DirectionControlMAX          5000
#define DirectionControlMIN          -5000
void ADC_init(void);
void ZeroDrift_correct(void);
void Speed_Calculate(void);
void AD_Calculate(uint8_t N_AD);         //采集

void DirectionControl(void);
void ovoid_DirectionControl(uint8_t direct_count);   //速度平滑输出
void read_DIPswitch(void);

extern double weight[];

#endif  //__Erect_H__
