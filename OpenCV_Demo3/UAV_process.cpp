#include "stdafx.h"
#include<iostream>
#include<opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<pic_Pre_Process.h>
#include<UAV_process.h>

dji_drones get_UAV_info(void)
{
	dji_drones UAV;
	UAV.destine_Xg = 0;
	UAV.destine_Yg = 0;
	UAV.destine_Zg = 0;
	UAV.hieght = 6;
	UAV.speed = 0;
	UAV.Xc = 0;
	UAV.Yc = 0;
	UAV.Zc = 0;
	UAV.Xc = 0;
	UAV.Yc = 0;
	UAV.Zc = 0;
	return UAV;
}