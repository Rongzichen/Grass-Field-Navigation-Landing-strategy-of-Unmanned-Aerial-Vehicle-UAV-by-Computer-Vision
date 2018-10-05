#ifndef UAV_PROCESS_H
#define UAV_PROCESS_H
#include <highgui.h>
#include<iostream>
struct dji_drones //这里把之前所有版本的drones类改为了dji_drones类，主要是为了防止与DJI_SDK中发生冲突
{
	double Xg, Yg, Zg;//无人机重心坐标---无人机坐标
	double Xc, Yc, Zc;//无人机镜头内部的景物坐标
	double hieght, speed;//无人机传感器数据--高度和速度
	double destine_Xg, destine_Yg, destine_Zg;//无人机目标坐标
};
dji_drones get_UAV_info(void);


#endif


