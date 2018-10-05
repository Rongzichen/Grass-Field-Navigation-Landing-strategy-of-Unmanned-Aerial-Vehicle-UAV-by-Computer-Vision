#ifndef pic_Pre_Process_H
#define pic_Pre_Process_H
#include <highgui.h>
#include<iostream>
/*用于存放图像预处理函数的部分*/
void double_thresh(CvArr* src, CvArr* dst, double threshold, double max_value, int threshold_type);//对图像中RGB部分求和后进行二值化处理函数
IplImage* sum_rgb(CvArr* src);//对RGB部分求和的函数
void find_connected_components(IplImage* mask,int poly1_hull0,double perimScale,int* num,CvRect* bbs,CvPoint* centers);//可以通过求连通的方式去除背景中的噪声。
IplImage* frame_diff(IplImage* frame1, IplImage* frame2, int threshold, int max_value, int threshold_type);//寻找图像的连通区域。
IplImage* pic_flip(IplImage* src, int mode);//对图像进行90度旋转，从而方便看图像的所有轮廓。
void main_preprocess(IplImage* frame1, IplImage* frame2, CvCapture* capture);//预处理的主函数
IplImage* skip_frame(CvCapture* capture, IplImage* frame2, int num);//该函数用于设置跳过的帧数，返回的就是略过一定数目之后的帧（num表示两帧之间间隔多少帧）（防止运动过慢识别不出帧差）
IplImage* reverse(IplImage* src);//将图像灰度求反的函数
CvPoint* find_center_point(CvSeq* mask, CvPoint* centers, int counter);//用于根据已经知道的轮廓信息找出轮廓的矩形框中心点
double cal_distance(CvPoint* center, CvPoint* boundary);//计算两像素点之间的距离
double find_largest(double *distance[10]);//计算10个double型数据之间最大的数值

#endif

/*
1. 头文件中也要include所有用到了的头文件.....
2. 头文件要想其可以使用，必须要在“属性-VC++目录-包含目录”中添加入头文件所在的文件夹
3. 报错的地方要点开，在网上可以搜索到解决方法的
4. 自己有时问题描述方式不够正确，不如点开报错搜索全网
*/