#include "include.h"
#include "Algorithm.h"
#include <string.h>

static uint8 serch_left_black_line(uint8 *image,uint8_t line,uint8_t median)
{
    image += line*CAMERA_W;
    register uint16_t x;
    for(x=median;x>1;x--)
    {
        if(image[x]==BLACK)
        {
            if(image[x-1]==BLACK &&
               image[x-2]==BLACK)
            {
                if(image[x+1]==WHITE &&
                   image[x+2]==WHITE)
                {
                    return x;
                }
            }
            else
            {
                x--;
                continue;
            }
        }
    }
    return CAMERA_W;
}

static uint8 serch_right_black_line(uint8 *image,uint8_t line,uint8_t median)
{
    image += line*CAMERA_W;
    register uint16_t x;
    for(x=median;x<CAMERA_W-2;x++)
    {
        if(image[x]==BLACK)
        {
            if(image[x+1]==BLACK &&
               image[x+2]==BLACK)
            {
                if(image[x-1]==WHITE &&
                   image[x-2]==WHITE)
                {
                    return x;
                }
            }
            else
            {
                x++;
                continue;
            }
        }
    }
    return CAMERA_W;
}

boundary_t serch_left_edge(uint8 *image,uint8_t start,uint8_t end,uint8_t median)
{
    static uint8_t left_edge[CAMERA_H] = {0};
    static _Bool left_edge_flag[CAMERA_H] = {0};
    static boundary_t left_edges = {left_edge,left_edge_flag};
    register uint16_t y;
    memset(left_edge_flag,0,sizeof(_Bool)*CAMERA_H);
    
    for(y=start;y<=end;y++)
    {
        uint8_t temp = serch_left_black_line(image,y,median);
        if(temp>=CAMERA_W)
        {
            continue;
        }
        else
        {
            left_edge[y] = temp;
            left_edge_flag[y] = 1;
        }
    }
    return left_edges;
}

boundary_t serch_right_edge(uint8 *image,uint8_t start,uint8_t end,uint8_t median)
{
    static uint8_t right_edge[CAMERA_H] = {0};
    static _Bool right_edge_flag[CAMERA_H] = {0};
    static boundary_t right_edges = {right_edge,right_edge_flag};
    register uint8_t y;
    memset(right_edge_flag,0,sizeof(_Bool)*CAMERA_H);
    
    for(y=start;y<=end;y++)
    {
        uint8_t temp = serch_right_black_line(image,y,median);
        if(temp>=CAMERA_W)
        {
            continue;
        }
        else
        {
            right_edge[y] = temp;
            right_edge_flag[y] = 1;
        }
    }
    return right_edges;
}

void five_point_smooth(uint8_t start,uint8_t end,uint8_t *mids)
{
    register uint8_t count;
    for(count=start;count<=end-4;++count)
    {
        mids[count] = (mids[count]+mids[count+1]+mids[count+2]+mids[count+3]+mids[count+4])/5;
    }
}

double least_square(uint8_t end,uint8_t start,uint8_t map_start,uint8_t map_end,uint8_t *mids)//start为最低端行数，与其他函数相反
{
    double rowba;
    double midba;
    int midsum;
    register uint8_t row;
    for(row=start,midsum=0;row>=end;row--)//计算中点和
    {
  	    midsum += mids[row];
    }
    midba = (double)midsum/(start-end+1);//计算mid均值
    rowba = (double)(end+start)/2.0;//计算row均值
    
    double sumx1;
    double sumx2;
    for(row=start,sumx1=0,sumx2=0;row>=end;row--)//计算斜率的分子(sumx1)/分母(sumx2)
    {
	    double temp;
	    temp = row-rowba;
	    sumx1 += temp*(mids[row]-midba);
	    sumx2 += pow(temp,2);
    }
    
    double k,b;
    k = sumx1/sumx2;//计算斜率
    b = midba-k*rowba;//计算截距
    for(row=map_start;row<=map_end;row++)//映射到对应区间
    {
	    mids[row] = (uint8_t)(k*row+b+0.5);
    }
    return k;
}

static _Bool serch_left_black_block(uint8 *image,uint8_t line,uint8_t median)
{
    uint8_t status;
    status = serch_left_black_line(image,line,median);
    if(status>=CAMERA_W || status<median-25)//左黑方块右侧丢失
    {
        return 0;
    }
    
    image += line*CAMERA_W;
    register uint8_t x;
    for(x=status;x>status-20&&x>1;x--)//寻找左黑块左侧
    {
        if(image[x]==WHITE && image[x-1]==WHITE && image[x-2]==WHITE)
        {
            status=x;
            break;
        }
    }
    if(x==status-20 || x<=1)//左黑方块左侧丢失
    {
        return 0;
    }
    
    for(x=status;x>1;x--)//寻找左边界
    {
        if(image[x]==BLACK && image[x-1]==BLACK && image[x-2]==BLACK)
        {
            return 1;
        }
    }
    return 0;
}

static _Bool serch_right_black_block(uint8 *image,uint8_t line,uint8_t median)
{
    uint8_t status;
    status = serch_right_black_line(image,line,median);
    if(status>=CAMERA_W || status>median+25)//右黑方块左侧丢失
    {
        return 0;
    }
    
    image += line*CAMERA_W;
    register uint8_t x;
    for(x=status;x<status+20&&x<=CAMERA_W-1;x++)//寻找右黑块右侧
    {
        if(image[x]==WHITE && image[x+1]==WHITE && image[x+2]==WHITE)
        {
            status=x;
            break;
        }
    }
    if(x==status+20 || x>=CAMERA_W-1)//右黑方块右侧丢失
    {
        return 0;
    }
    
    for(x=status;x<CAMERA_W-2;x++)//寻找右边界
    {
        if(image[x]==BLACK && image[x+1]==BLACK && image[x+2]==BLACK)
        {
            return 1;
        }
    }
    return 0;
}

/*障碍物*/
static _Bool is_left_obstacle(uint8 *image,uint8_t start,uint8_t end)
{
    uint8_t y;
    int lines = 0;
    for(y=end;y>=start;y--)
    {
        if(serch_left_black_block(image,y,CAMERA_W/2))
        {
            lines++;
        }
    }
    if(lines >= 3)
    {
        led(LED2,LED_ON);
        led(LED3,LED_OFF);
        return 1;
    }
    else
    {
        return 0;
    }
}

static _Bool is_right_obstacle(uint8 *image,uint8_t start,uint8_t end)
{
    uint8_t y;
    int lines = 0;
    for(y=end;y>=start;y--)
    {
        if(serch_right_black_block(image,y,CAMERA_W/2))
        {
            lines++;
        }
    }
    if(lines >= 3)
    {
        led(LED2,LED_OFF);
        led(LED3,LED_ON);
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t is_obstacle(uint8 *image,uint8_t start,uint8_t end,uint8_t *mids)
{
    static uint8_t obstacle_type=NO_OBSTACLE;
    if(is_left_obstacle(image,start,end))
    {
        obstacle_type = LEFT_OBSTACLE;
    }
    else if(is_right_obstacle(image,start,end))
    {
        obstacle_type = RIGHT_OBSTACLE;
    }
    else
    {
        led(LED2,LED_OFF);
        led(LED3,LED_OFF);
        obstacle_type = NO_OBSTACLE;
    }
    return obstacle_type;
}

/*起跑线识别*/
#define end_offset 3    //起跑线仅识别最下面三行

_Bool is_start(uint8 *image,uint8_t end)
{
    uint8_t x,y;
    uint8_t count_x = 0, count_y = 0;
    for(y=end-end_offset;y<end;y++)
    {
        image += y*CAMERA_W;
        for(x=0; x<CAMERA_W; x++)
        {
            if(image[x]==BLACK && image[x+1]==WHITE) count_x++;
            if(count_x>=6) 
            {
                count_y++;
                count_x=0;
            }
        }
        if(count_y>=3)
        {
            count_y = 0;
            return 1;
        }
        image -= y*CAMERA_W;
    }  
    return 0;
}

/* 膨胀腐蚀 */
void Erosion(uint8 *src, uint8 *dst)//腐蚀算法
{
    uint8_t x,y;
    for(y=1; y<CAMERA_H-1; y++)//对图像内部进行置一处理
        for(x=1; x<CAMERA_W-1; x++)
            if(src[x+(y-1)*CAMERA_W]==BLACK || src[x+(y+1)*CAMERA_W]==BLACK || src[(x-1)+y*CAMERA_W]==BLACK || src[(x+1)+y*CAMERA_W]==BLACK)  
                dst[x+y*CAMERA_W]=BLACK;
    for(x=1; x<CAMERA_W-1; x++)
    {
        if(src[x-1]==BLACK || src[x+1]==BLACK)//对图像第一行数据置一处理
            dst[x]=BLACK;
        if(src[(x-1)+CAMERA_W*(CAMERA_H-2)]==BLACK || src[(x+1)+CAMERA_W*(CAMERA_H-2)]==BLACK)//对图像最后一行置一处理
            dst[x+CAMERA_W*(CAMERA_H-2)]=BLACK;
    }
    for(y=1; y<CAMERA_H-1; y++)
    {
        if(src[CAMERA_W*(y-1)]==BLACK || src[CAMERA_W*(y+1)]==BLACK)//对第一列进行置一处理
             dst[CAMERA_W*y]=BLACK;
        if(src[CAMERA_W*(y-1)-1]==BLACK || src[CAMERA_W*(y+1)-1]==BLACK)//对最后一列进行置一处理
             dst[CAMERA_W*y-1]=BLACK;
    }
    if(src[1]==BLACK || src[CAMERA_W]==BLACK)//对四个角进行置一处理
        dst[0]=BLACK;
    if(src[CAMERA_W-2]==BLACK || src[2*CAMERA_W-1]==BLACK)
        dst[CAMERA_W-1]=BLACK;
    if(src[CAMERA_W*(CAMERA_H-2)]==BLACK || src[CAMERA_W*(CAMERA_H-1)+1]==BLACK)
        dst[CAMERA_W*(CAMERA_H-1)]=BLACK;
    if(src[CAMERA_W*(CAMERA_H-1)-1]==BLACK || src[CAMERA_W*CAMERA_H-2]==BLACK)
        dst[CAMERA_W*CAMERA_H-1]=BLACK;
}

void Dilation(uint8 *src, uint8 *dst)//膨胀算法
{
    uint8_t x,y;
    for(y=1; y<CAMERA_H-1; y++)//对图像内部进行置一处理
        for(x=1; x<CAMERA_W-1; x++)
            if(src[x+(y-1)*CAMERA_W]==WHITE || src[x+(y+1)*CAMERA_W]==WHITE || src[(x-1)+y*CAMERA_W]==WHITE || src[(x+1)+y*CAMERA_W]==WHITE)  
                dst[x+y*CAMERA_W]=WHITE;
    for(x=1; x<CAMERA_W-1; x++)
    {
        if(src[x-1]==WHITE || src[x+1]==WHITE)//对图像第一行数据置一处理
            dst[x]=WHITE;
        if(src[(x-1)+CAMERA_W*(CAMERA_H-2)]==WHITE || src[(x+1)+CAMERA_W*(CAMERA_H-2)]==WHITE)//对图像最后一行置一处理
            dst[x+CAMERA_W*(CAMERA_H-2)]=WHITE;
    }
    for(y=1; y<CAMERA_H-1; y++)
    {
        if(src[CAMERA_W*(y-1)]==WHITE || src[CAMERA_W*(y+1)]==WHITE)//对第一列进行置一处理
             dst[CAMERA_W*y]=WHITE;
        if(src[CAMERA_W*(y-1)-1]==WHITE || src[CAMERA_W*(y+1)-1]==WHITE)//对最后一列进行置一处理
             dst[CAMERA_W*y-1]=WHITE;
    }
    if(src[1]==WHITE || src[CAMERA_W]==WHITE)//对四个角进行置一处理
        dst[0]=WHITE;
    if(src[CAMERA_W-2]==WHITE || src[2*CAMERA_W-1]==WHITE)
        dst[CAMERA_W-1]=WHITE;
    if(src[CAMERA_W*(CAMERA_H-2)]==WHITE || src[CAMERA_W*(CAMERA_H-1)+1]==WHITE)
        dst[CAMERA_W*(CAMERA_H-1)]=WHITE;
    if(src[CAMERA_W*(CAMERA_H-1)-1]==WHITE || src[CAMERA_W*CAMERA_H-2]==WHITE)
        dst[CAMERA_W*CAMERA_H-1]=WHITE;
}

void copy_image(uint8 *src, uint8 *reference)//复制一个参考图像
{
    for(uint16_t i=0; i<CAMERA_W*CAMERA_H; i++)
        reference[i] = src[i];
}
