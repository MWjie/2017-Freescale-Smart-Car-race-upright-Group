#include "include.h"

_Bool MotorSTOP = 0;     //停车标志位

void Motor_init(void)      //初始化 电机
{
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,FTM0_PRECISON);      //反 右
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,FTM0_PRECISON);      //正 右
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,FTM0_PRECISON);      //正 左
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,FTM0_PRECISON);      //反 左
}

void Motor_out(float speed_Start ,float speed_Direct)     //直立 闭环 方向
{
    int32_t LeftMotorPWM ,RightMotorPWM;
    uint32_t LeftMotor ,RightMotor;

/* 左右轮输出值 */  
    LeftMotorPWM = (int32_t)(speed_Start - speed_Direct); //右转正 左转负 左加右减
    RightMotorPWM = (int32_t)(speed_Start + speed_Direct);
    
/* 死区电压 */   
    if(LeftMotorPWM > 0)
        LeftMotorPWM += MOTOR_DEAD_PLUS_L;
    else
        LeftMotorPWM -= MOTOR_DEAD_MINUS_L;
    if(RightMotorPWM > 0)
        RightMotorPWM += MOTOR_DEAD_PLUS_R;
    else
        RightMotorPWM -= MOTOR_DEAD_MINUS_R;
    
/* 输出限幅 */   
    if(LeftMotorPWM > MotorMAXOut)
        LeftMotorPWM = MotorMAXOut;
    if(LeftMotorPWM < MotorMINOut)
        LeftMotorPWM = MotorMINOut;
    if(RightMotorPWM > MotorMAXOut)
        RightMotorPWM = MotorMAXOut;
    if(RightMotorPWM < MotorMINOut)
        RightMotorPWM = MotorMINOut;
    
/* 输出反向 */
    if(LeftMotorPWM >= 0)
        LeftMotor = FTM0_PRECISON - LeftMotorPWM;
    else 
        LeftMotor = FTM0_PRECISON + LeftMotorPWM;
    if(RightMotorPWM >= 0)       
        RightMotor = FTM0_PRECISON - RightMotorPWM;
    else                        
        RightMotor = FTM0_PRECISON + RightMotorPWM;

/* 停车标志 */    
    if(MotorSTOP == 1)
    {
        LeftMotor = FTM0_PRECISON;
        RightMotor = FTM0_PRECISON;
    }

/* PWM输出 */    
    if(RightMotorPWM >= 0)    //angle大于0，向前，小于0，向后
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR2_PWM,FTM0_PRECISON);
        ftm_pwm_duty(MOTOR_FTM,MOTOR1_PWM,RightMotor);    
    }
    else
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR1_PWM,FTM0_PRECISON);
        ftm_pwm_duty(MOTOR_FTM,MOTOR2_PWM,RightMotor);   
    }

    if(LeftMotorPWM >= 0)    //angle大于0，向前，小于0，向后
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



