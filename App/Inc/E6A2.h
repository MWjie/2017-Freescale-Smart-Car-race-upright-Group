#ifndef __E6A2_H__
#define __E6A2_H__

#define ENCODER_QUAD1   FTM1
#define ENCODER_QUAD2   FTM2

/* 模式选择 */
#define HIGH_SPEED_Ring      0x0E
#define HIGH_SPEED           0x01
#define MEDIUM_SPEED_Ring    0x0D
#define MEDIUM_SPEED         0x02
#define LOW_SPEED_Ring       0x0B
#define LOW_SPEED            0x04
#define TEST_Ring            0x07
#define TEST                 0x08

/* 速度闭环参数 */
#define P_SPEED         6
#define I_SPEED         0.6

#define SpeedControlMAX    18000
#define SpeedControlMIN    -18000

extern uint16_t WantSpeed;
extern float g_fSpeedControlOut;     //速度环输出

void E6A2_init(void);
void E6A2_countPulse(void);
void SpeedControl(void);
void ovoid_SpeedControl(uint8_t speed_count);   //速度平滑输出

#endif  //__E6A2_H__
