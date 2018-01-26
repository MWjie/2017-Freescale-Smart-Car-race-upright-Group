#include "include.h"

_Bool MotorSTOP = 0;     //ͣ����־λ

void Motor_init(void)      //��ʼ�� ���
{
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,FTM0_PRECISON);      //�� ��
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,FTM0_PRECISON);      //�� ��
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,FTM0_PRECISON);      //�� ��
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,FTM0_PRECISON);      //�� ��
}

void Motor_out(float speed_Start ,float speed_Direct)     //ֱ�� �ջ� ����
{
    int32_t LeftMotorPWM ,RightMotorPWM;
    uint32_t LeftMotor ,RightMotor;

/* ���������ֵ */  
    LeftMotorPWM = (int32_t)(speed_Start - speed_Direct); //��ת�� ��ת�� ����Ҽ�
    RightMotorPWM = (int32_t)(speed_Start + speed_Direct);
    
/* ������ѹ */   
    if(LeftMotorPWM > 0)
        LeftMotorPWM += MOTOR_DEAD_PLUS_L;
    else
        LeftMotorPWM -= MOTOR_DEAD_MINUS_L;
    if(RightMotorPWM > 0)
        RightMotorPWM += MOTOR_DEAD_PLUS_R;
    else
        RightMotorPWM -= MOTOR_DEAD_MINUS_R;
    
/* ����޷� */   
    if(LeftMotorPWM > MotorMAXOut)
        LeftMotorPWM = MotorMAXOut;
    if(LeftMotorPWM < MotorMINOut)
        LeftMotorPWM = MotorMINOut;
    if(RightMotorPWM > MotorMAXOut)
        RightMotorPWM = MotorMAXOut;
    if(RightMotorPWM < MotorMINOut)
        RightMotorPWM = MotorMINOut;
    
/* ������� */
    if(LeftMotorPWM >= 0)
        LeftMotor = FTM0_PRECISON - LeftMotorPWM;
    else 
        LeftMotor = FTM0_PRECISON + LeftMotorPWM;
    if(RightMotorPWM >= 0)       
        RightMotor = FTM0_PRECISON - RightMotorPWM;
    else                        
        RightMotor = FTM0_PRECISON + RightMotorPWM;

/* ͣ����־ */    
    if(MotorSTOP == 1)
    {
        LeftMotor = FTM0_PRECISON;
        RightMotor = FTM0_PRECISON;
    }

/* PWM��� */    
    if(RightMotorPWM >= 0)    //angle����0����ǰ��С��0�����
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR2_PWM,FTM0_PRECISON);
        ftm_pwm_duty(MOTOR_FTM,MOTOR1_PWM,RightMotor);    
    }
    else
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR1_PWM,FTM0_PRECISON);
        ftm_pwm_duty(MOTOR_FTM,MOTOR2_PWM,RightMotor);   
    }

    if(LeftMotorPWM >= 0)    //angle����0����ǰ��С��0�����
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR3_PWM,FTM0_PRECISON);
        ftm_pwm_duty(MOTOR_FTM,MOTOR4_PWM,LeftMotor);    
    }
    else
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR4_PWM,FTM0_PRECISON);
        ftm_pwm_duty(MOTOR_FTM,MOTOR3_PWM,LeftMotor);  
    }
}



