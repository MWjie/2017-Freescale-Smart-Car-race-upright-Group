#include "include.h"
#include "Algorithm.h"

_Bool f_start = 0;
volatile discern_result_t control_result={0,0};

static uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
static uint8 img[CAMERA_H*CAMERA_W];

static void PORTA_IRQHandler(void)
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

static void DMA0_IRQHandler(void)
{
    camera_dma();
}

#define start_line 20
#define end_line (CAMERA_H-1)
#define base_line (CAMERA_W/2-4)
#define edge_offset (base_line+base_edge_offset[count])

static void Lines(uint8_t start,uint8_t end,uint8_t *lines)
{
    static Site_t line_site[CAMERA_H];
    register uint8_t count;
    for(count=start;count<end;count++)
    {
        line_site[count].x = lines[count];
        line_site[count].y = count;
    }
    LCD_points(&line_site[start],end-start+1,RED);
}

void discern_init(void)
{
    camera_init(imgbuff);
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //摄像头场中断
    NVIC_SetPriority(PORTA_IRQn,0);
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //摄像头DMA中断
    NVIC_SetPriority(DMA0_IRQn,2);
    led_init(LED1);                                        //起跑线指示灯
//    led_init(LED2);                                        //右障碍 左通行
//    led_init(LED3);                                        //左障碍 右通行
}
/*
static const int base_edge_offset[CAMERA_H]={
	8,8,8,8,9,9,9,10,10,10,
	12,12,12,13,13,13,14,14,14,15,
	15,15,16,16,16,17,17,17,18,18,
	18,19,19,19,20,20,20,21,21,21,
	22,22,23,23,24,24,25,25,26,26,
	26,27,27,28,28,28,29,29,29,30
};
*/
static const int base_edge_offset[CAMERA_H]={
	14,14,14,15,15,15,16,16,16,17,
	17,17,17,18,18,18,19,19,19,20,
	20,20,20,21,21,21,22,22,22,23,
	23,23,23,24,24,24,25,25,25,26,
	26,26,26,27,27,27,28,28,28,29,
	29,29,29,30,30,30,31,31,31,32
};

static uint8_t* get_midline(uint8 *image)
{
    static uint8_t mids[CAMERA_H]={0};
    /*获取基本信息*/
    boundary_t left_edge,right_edge;
    left_edge = serch_left_edge(image,start_line,end_line,base_line);
    right_edge = serch_right_edge(image,start_line,end_line,base_line); 
    register uint8_t count;
//    uint8_t left_lost=0,right_lost=0;
//    int staggered=0;
    /*补出中线*/
    for(count=start_line;count<sizeof(mids);count++)
    {
        _Bool get_left = left_edge.done[count];
        _Bool get_right = right_edge.done[count];
        if(get_left && get_right)//都找到
        {
            if(right_edge.edge[count] > left_edge.edge[count])
            {
                mids[count] = (uint8_t)(left_edge.edge[count]+right_edge.edge[count])/2;
            }
//            else
//            {
//                staggered++;
//            }
        }
        else if((!get_left) && get_right)//左丢失
        {
            mids[count] = (uint8_t)(right_edge.edge[count]-edge_offset/2);
            left_edge.edge[count] = mids[count]-edge_offset/2;
            if(mids[count] >= 80)
            {
                mids[count] = 0;
            }
//            left_lost++;
        }
        else if(get_left && (!get_right))//右丢失
        {
            mids[count] = (uint8_t)(left_edge.edge[count]+edge_offset/2);
            right_edge.edge[count] = mids[count]+edge_offset/2;
            if(mids[count] >= 80)
            {
                mids[count] = 79;
            }
//            right_lost++;
        }
//        else//全丢失
//        {
//            left_lost++;
//            right_lost++;
//        }
    }
       
    return mids;
}
/*
static uint16_t speed_choose(uint16_t average_mid)
{
    uint16_t speed=0;
    if(average_mid>5)
    {
        speed=200;
    }
    else
    {
        speed=260;
    }
    return speed;
}
*/

static uint8_t get_average_mid(uint8_t start,uint8_t end,uint8_t *mids,uint8 *image)
{
/*
    static const double weight[CAMERA_H]={
2.77,2.74,2.71,2.68,2.65,2.62,2.59,2.56,2.53,2.5,
2.47,2.44,2.41,2.38,2.35,2.32,2.29,2.26,2.23,2.2,
2.17,2.14,2.11,2.08,2.05,2.02,1.99,1.96,1.93,1.9,
1.87,1.84,1.81,1.78,1.75,1.72,1.69,1.66,1.63,1.6,
1.57,1.54,1.51,1.48,1.45,1.42,1.39,1.36,1.33,1.3,
1.27,1.24,1.21,1.18,1.15,1.12,1.09,1.06,1.03,1
    };//y=0.06x+1
*/
    register uint8_t count;
    double mid_result = 0;
    double div = 0;
//    mid_result = (mids[end]*weight[end] + mids[end-1]*weight[end-1]);
//    div = weight[end]+weight[end-1];
    for(count=end;count>=start;count--)
    {
        mid_result += (mids[count] * weight[count]);
        div += weight[count];
    }
    Lines(count+1,end,mids);//显示中线
    double result = mid_result/div+0.5;
    if(result < 0)
    {
        return 0;
    }
    else if(result > CAMERA_W-1)
    {
        return CAMERA_W-1;
    }
    else
    {
        return (uint8_t)result;
    }
}

static discern_result_t compute_result(uint8_t *mids,uint8 *image)
{
    discern_result_t result={0,0};
    five_point_smooth(start_line,end_line,mids);
    result.angle=get_average_mid(start_line,end_line,mids,image)-base_line;   //右转正 左转负
//    result.speed=speed_choose(result.angle);
    return result;
}

discern_result_t discern(void)
{
    _Bool Zebra = 0;
    camera_get_img();//获取图像
    img_extract(img, imgbuff, CAMERA_SIZE);//解压为灰度图像

//    vcan_sendimg(img,CAMERA_H*CAMERA_W);
    LCD_Img_gray((Site_t){0, 0}, (Size_t){CAMERA_W, CAMERA_H}, img);

    uint8_t *mids;//中线
    mids=get_midline(img);
    
    static discern_result_t result={0,250};//初始偏角，初始速度

    if(mids==NULL)
    {
        return result;
    }

    result=compute_result(mids,img);//计算偏角，速度选择
    
    if(Timer >= 4000)         //大于20s才检测起跑线
    {
        Zebra = is_start(img, end_line);//起跑线检测
        if(Zebra == 1)
        {
            DELAY_MS(200);
            MotorSTOP = 1;
            led_turn(LED1);
        }
    }
    
    return result;//返回识别结果
}


