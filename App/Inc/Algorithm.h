#ifndef __Algorithm_H__
#define __Algorithm_H__

#ifdef BLACK
#undef BLACK
#endif
#define BLACK 0

#ifdef WHITE
#undef WHITE
#endif
#define WHITE 255

#define NO_OBSTACLE     0
#define LEFT_OBSTACLE   1
#define RIGHT_OBSTACLE  2
#define GO_OUT_OBSTACLE 3

typedef struct
{
    uint8_t *edge;
    _Bool *done;
}boundary_t;

boundary_t serch_left_edge(uint8 *image,uint8_t start,uint8_t end,uint8_t median);
boundary_t serch_right_edge(uint8 *image,uint8_t start,uint8_t end,uint8_t median);
void five_point_smooth(uint8_t start,uint8_t end,uint8_t *mids);
double least_square(uint8_t end,uint8_t start,uint8_t map_start,uint8_t map_end,uint8_t *mids);
_Bool is_start(uint8 *image,uint8_t end);
uint8_t is_obstacle(uint8 *image,uint8_t start,uint8_t end,uint8_t *mids);
void Dilation(uint8 *src, uint8 *dst);
void Erosion(uint8 *src, uint8 *dst);
void copy_image(uint8 *src, uint8 *reference);

#endif  //__Algorithm_H__
