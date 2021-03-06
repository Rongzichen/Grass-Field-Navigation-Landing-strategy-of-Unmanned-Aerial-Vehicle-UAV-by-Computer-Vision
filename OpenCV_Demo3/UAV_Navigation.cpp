﻿#include "stdafx.h"
#include<iostream>
#include<opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<pic_Pre_Process.h>
#include<landing_strategy.h>
using namespace cv;


int main()
{
	IplImage* frame2;
	CvCapture* capture = cvCreateFileCapture("C:/Users/Rongzichen Song/Desktop/Temperary files/OpenCV_lib/UAV_Navigation_project/Sample_pics/Processed Pics/measure_playground/(1.85,1.35).mp4");//Put the path of the video here, so that it could be used. 
	frame2 = cvQueryFrame(capture);//此处进行第一帧的赋值！
	IplImage* frame1 = cvCreateImage(cvGetSize(frame2), frame2->depth, frame2->nChannels);
	main_preprocess(frame1, frame2, capture);
	IplImage* reverse(IplImage* src);
	return(0);
}
/*
1.首先我配置的模板需要加入#include<stdafx.h>
2.其次这里面加入了#include<opencv2\opencv.hpp>以及#include<opencv2\imgproc\imgproc.hpp>，所以可以使用imread和imshow等函数，而未加入这两个的无法使用这两个函数，必须用“学习OpenCV”里面的库函数
3.这个只配置了Debug模式，未配置Release模式，所以不全面，未来可能出现一些问题。
4.“0x00007FFB1FD54008 处(位于 OpenCV_Demo3.exe 中)引发的异常: Microsoft C++ 异”这种错误是因为图片未启用绝对路径，最开始只写了imread("图片.png");所以报错了，以后就写绝对路径，注意使用“/”，而不是\。
5.对于一些总是提示无法进行从IplImage*到 cv::....Arry类型转换的时候，其实是函数使用不正确，在其函数前面加上cv并将其它名称按照正确命名方法修改即可使用。
6.对于旋转，cvTranspose是将行与列进行兑换，所以对于转换的空图像，其行和列与原图像应该反过来。
7.每次复制一个新的工程时，需要对所用的所有头文件的文件夹进行再次的包含。
*/
