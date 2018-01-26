#ifndef __Hawkeye_H__
#define __Hawkeye_H__

typedef struct
{
    uint16_t angle;
    uint16_t speed;
}discern_result_t;

typedef enum
{
    curve,      //弯道
    beeline,    //直线
    crossing,   //十字
    obstacle    //障碍
}traffic;

extern volatile discern_result_t control_result;

void discern_init(void);
discern_result_t discern(void);

#endif  //__Hawkeye_H__
