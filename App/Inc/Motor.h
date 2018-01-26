#ifndef __Motor_H__
#define __Motor_H__

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH3
#define MOTOR2_PWM  FTM_CH4
#define MOTOR3_PWM  FTM_CH5
#define MOTOR4_PWM  FTM_CH6

#define MOTOR1_PWM_IO  FTM0_CH3
#define MOTOR2_PWM_IO  FTM0_CH4
#define MOTOR3_PWM_IO  FTM0_CH5
#define MOTOR4_PWM_IO  FTM0_CH6

#define MOTOR_HZ    10000

#define MOTOR_DEAD_PLUS_L   320    //左右轮正转反转死区电压 320/100
#define MOTOR_DEAD_MINUS_L  320
#define MOTOR_DEAD_PLUS_R   100 
#define MOTOR_DEAD_MINUS_R  100

#define MotorMAXOut         9000   //输出限幅
#define MotorMINOut         -9000

extern _Bool MotorSTOP;     //停车标志位

void Motor_init(void);
void Motor_out(float speed_Start, float speed_Direct);

#endif  //__Motor_H__
