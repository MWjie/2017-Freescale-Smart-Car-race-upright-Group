 
# 2017-Freescale-Smart-Car-race-upright-Group

2017年飞思卡尔智能车 四川省赛 摄像头直立组 

工程于2017年1月开始构思，2017年5月底停止修改。

程序由定时器控制其逻辑运行，主循环只做图形处理。 \n
算法方面较为细致的使用了卡尔曼滤波对角度进行处理。 
速度环PI控制，直立环PD控制，转向环PD控制。 

------------------------------------------------------

时过半年再看这个程序

优点：
程序写的较为工整，定时器控制使程序有条理，驱动层写的较清晰。

缺点：
1、图像处理算法很简陋。这是没有用心的体现！也是比赛成绩差的主要原因。\n
以下是水平问题
2、图像数据处理方面，没有做到高效。
3、硬件上没有做电源管理，导致每次发车可能导致驱动电压不一样。也有可能是PID算法参数的问题。
4、直立车的平衡点很重要，轮胎与赛道的摩擦要做到练习和比赛一样的状况。
