#ifndef LANDING_STRATEGY_H
#define LANDING_STRATEGY_H
#include <highgui.h>
#include<iostream>
#include<pic_Pre_Process.h>
double* coordinate_transfer(int u, int v, double f, double s, double x0, double X, double Y, double Z, double alpha, double omega, double theta, double k1, double k2, double p1, double p2, double detaX, double detaY, double u0, double v0);
double *find_intersection(double x1, double x2, double y1, double y2, double r1, double r2, int counter, double gap_length, double radius_UAV);
double select_point(int counter, double point1, double point2);//这里是在俩点之间进行随机的选取的函数
double* selectpoint_circle(int counter, double circleCenter_x, double circleCenter_y, float radius, double redundance);//该函数用于在一个圆中找出降落的随机点
#endif