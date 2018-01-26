#ifndef __Hawkeye_H__
#define __Hawkeye_H__

typedef struct
{
    uint16_t angle;
    uint16_t speed;
}discern_result_t;

typedef enum
{
    curve,      //���
    beeline,    //ֱ��
    crossing,   //ʮ��
    obstacle    //�ϰ�
}traffic;

extern volatile discern_result_t control_result;

void discern_init(void);
discern_result_t discern(void);

#endif  //__Hawkeye_H__
