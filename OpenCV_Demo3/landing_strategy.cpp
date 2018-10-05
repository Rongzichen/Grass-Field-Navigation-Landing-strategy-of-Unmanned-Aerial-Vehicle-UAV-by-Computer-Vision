#include "stdafx.h"
#include<iostream>
#include<opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<pic_Pre_Process.h>
#include<UAV_process.h>
#include<math.h>
using namespace cv;
using namespace std;

/*以后用这种方法来返回多个数值*/
double* coordinate_transfer(int u,int v,double f,double s,double x0,double X, double Y, double Z, double alpha, double omega, double theta, double k1, double k2, double p1, double p2, double detaX, double detaY, double u0,double v0)
{
	double *cordinator = new double[2];
	cordinator[0] = (-(X*f*s*cos(theta)*cos(omega) - Z * detaX*v*cos(theta)*cos(omega) + Z * detaX*v0*cos(theta)*cos(omega) + Y * f*s*cos(alpha)*sin(omega) + X * detaY*u*sin(theta)*cos(omega) - X * detaY*u0*sin(theta)*cos(omega) - Z * detaY*u*cos(alpha)*sin(omega) + Z * detaY*u0*cos(alpha)*sin(omega) - Y * detaX*v*sin(theta)*cos(omega) + Y * detaX*v0*sin(theta)*cos(omega) - f * s*x0*cos(theta)*cos(omega) - detaY * u*x0*sin(theta)*cos(omega) + detaY * u0*x0*sin(theta)*cos(omega) - X * detaY*u*cos(theta)*sin(alpha)*sin(omega) + X * detaY*u0*cos(theta)*sin(alpha)*sin(omega) + Y * detaX*v*cos(theta)*sin(alpha)*sin(omega) - Y * detaX*v0*cos(theta)*sin(alpha)*sin(omega) + X * f*s*sin(alpha)*sin(theta)*sin(omega) - Z * detaX*v*sin(alpha)*sin(theta)*sin(omega) + Z * detaX*v0*sin(alpha)*sin(theta)*sin(omega) + detaY * u*x0*cos(theta)*sin(alpha)*sin(omega) - detaY * u0*x0*cos(theta)*sin(alpha)*sin(omega) - f * s*x0*sin(alpha)*sin(theta)*sin(omega)) / (f*s*cos(alpha)*cos(theta)*(sin(omega)*sin(omega)) + detaY * u*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u0*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaY * u0*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) + detaX * v*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v0*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + detaX * v0*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + f * s*cos(alpha)*cos(theta)*(cos(omega)*cos(omega)) - detaX * v*cos(theta)*sin(alpha)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(alpha)*cos(omega)*sin(omega) - detaX * v*cos(theta)*sin(theta)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(theta)*cos(omega)*sin(omega)));
	cordinator[1] = ((Y*f*s*cos(alpha)*cos(omega) - Z * detaY*u*cos(alpha)*cos(omega) + Z * detaY*u0*cos(alpha)*cos(omega) - X * f*s*cos(theta)*sin(omega) + Z * detaX*v*cos(theta)*sin(omega) - Z * detaX*v0*cos(theta)*sin(omega) + X * detaY*u*sin(alpha)*sin(omega) - X * detaY*u0*sin(alpha)*sin(omega) - Y * detaX*v*sin(alpha)*sin(omega) + Y * detaX*v0*sin(alpha)*sin(omega) + f * s*x0*cos(theta)*sin(omega) - detaY * u*x0*sin(alpha)*sin(omega) + detaY * u0*x0*sin(alpha)*sin(omega) - X * detaY*u*cos(theta)*sin(alpha)*cos(omega) + X * detaY*u0*cos(theta)*sin(alpha)*cos(omega) + Y * detaX*v*cos(theta)*sin(alpha)*cos(omega) - Y * detaX*v0*cos(theta)*sin(alpha)*cos(omega) + X * f*s*sin(alpha)*sin(theta)*cos(omega) - Z * detaX*v*sin(alpha)*sin(theta)*cos(omega) + Z * detaX*v0*sin(alpha)*sin(theta)*cos(omega) + detaY * u*x0*cos(theta)*sin(alpha)*cos(omega) - detaY * u0*x0*cos(theta)*sin(alpha)*cos(omega) - f * s*x0*sin(alpha)*sin(theta)*cos(omega)) / (f*s*cos(alpha)*cos(theta)*(sin(omega)*sin(omega)) + detaY * u*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u0*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaY * u0*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) + detaX * v*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v0*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + detaX * v0*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + f * s*cos(alpha)*cos(theta)*(cos(omega)*cos(omega)) - detaX * v*cos(theta)*sin(alpha)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(alpha)*cos(omega)*sin(omega) - detaX * v*cos(theta)*sin(theta)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(theta)*cos(omega)*sin(omega)));
	return cordinator;
}

double select_point(int counter, double point1, double point2)//这里point1与point2的内容都是单纯的x或者y
{
	int selection;
	srand(counter);//feed seed，从而保证在一段时间内选出的点都是随机的
	selection = rand() % 2;//随机从俩点里面选出一个点，
	if (selection == 0)
	{
		return point1;
	}
	else
		return point2;
}

/*下面这个函数中的坐标的数值还需要进行由摄像头坐标到世界坐标的转换过程，所以该函数所用参数均为世界坐标*/
double *find_intersection(double x1, double x2, double y1, double y2,double r1,double r2,int counter,double gap_length,double radius_UAV)
{
	double k;//两圆心坐标之间的斜率
	double x1_sol, x2_sol, y1_sol, y2_sol;
	double radius_1, radius_2;//用于将半径放大以确保降落区域不会在障碍物上面
	double determined_x, determined_y;
	double *intersection = new double[2];
	radius_1 = r1+ gap_length+ radius_UAV;//距离物体中心的半径由"物体半径+无人机半径+冗余距离组成"
	radius_2 = r2 + gap_length + radius_UAV;
	if (y1 != y2&&x1 != x2)//对应于两圆心所在的直线斜率存在且不为0
	{
		k = (y2 - y1) / (x2 - x1);
		x1_sol = -(x1*y2 - x1 * y1 - x2 * y1 + x2 * y2 + k * radius_1*radius_1 - k * radius_2*radius_2 + k * y1*y1 + k * y2*y2 + k * sqrt(-k * k * radius_1*radius_1*radius_1*radius_1 + 2 * k*k * radius_1*radius_1 * radius_2*radius_2 + 2 * k*k * radius_1*radius_1 * y1*y1 - 4 * k*k * radius_1*radius_1 * y1*y2 + 2 * k*k * radius_1*radius_1 * y2*y2 - k * k * radius_2*radius_2*radius_2*radius_2 + 2 * k*k * radius_2*radius_2 * y1*y1 - 4 * k*k * radius_2*radius_2 * y1*y2 + 2 * k*k * radius_2*radius_2 * y2*y2 - k * k * y1*y1*y1*y1 + 4 * k*k * y1*y1*y1 * y2 - 6 * k*k * y1*y1 * y2*y2 + 4 * k*k * y1*y2*y2*y2 - k * k * y2*y2*y2*y2 - 2 * k*radius_1*radius_1 * x1*y1 + 2 * k*radius_1*radius_1 * x1*y2 + 2 * k*radius_1*radius_1 * x2*y1 - 2 * k*radius_1*radius_1 * x2*y2 + 2 * k*radius_2*radius_2 * x1*y1 - 2 * k*radius_2*radius_2 * x1*y2 - 2 * k*radius_2*radius_2 * x2*y1 + 2 * k*radius_2*radius_2 * x2*y2 - 2 * k*x1*y1*y1*y1 + 6 * k*x1*y1*y1 * y2 - 6 * k*x1*y1*y2*y2 + 2 * k*x1*y2*y2*y2 + 2 * k*x2*y1*y1*y1 - 6 * k*x2*y1*y1 * y2 + 6 * k*x2*y1*y2*y2 - 2 * k*x2*y2*y2*y2 + 4 * radius_1*radius_1 * y1*y1 - 8 * radius_1*radius_1 * y1*y2 + 4 * radius_1*radius_1 * y2*y2 - x1 * x1 * y1*y1 + 2 * x1*x1 * y1*y2 - x1 * x1 * y2*y2 + 2 * x1*x2*y1*y1 - 4 * x1*x2*y1*y2 + 2 * x1*x2*y2*y2 - x2 * x2 * y1*y1 + 2 * x2*x2 * y1*y2 - x2 * x2 * y2*y2) - 2 * k*k * x1*y1 + 2 * k*k * x1*y2 - 2 * k*y1*y2) / (2 * (y1 - y2 + k * k * y1 - k * k * y2));
		x2_sol = (x1*y1 - x1 * y2 + x2 * y1 - x2 * y2 - k * radius_1*radius_1 + k * radius_2*radius_2 - k * y1*y1 - k * y2*y2 + k * sqrt(-k * k * radius_1*radius_1*radius_1*radius_1 + 2 * k*k * radius_1*radius_1 * radius_2*radius_2 + 2 * k*k * radius_1*radius_1 * y1*y1 - 4 * k*k * radius_1*radius_1 * y1*y2 + 2 * k*k * radius_1*radius_1 * y2*y2 - k * k * radius_2*radius_2*radius_2*radius_2 + 2 * k*k * radius_2*radius_2 * y1*y1 - 4 * k*k * radius_2*radius_2 * y1*y2 + 2 * k*k * radius_2*radius_2 * y2*y2 - k * k * y1*y1*y1*y1 + 4 * k*k * y1*y1*y1 * y2 - 6 * k*k * y1*y1 * y2*y2 + 4 * k*k * y1*y2*y2*y2 - k * k * y2*y2*y2*y2 - 2 * k*radius_1*radius_1 * x1*y1 + 2 * k*radius_1*radius_1 * x1*y2 + 2 * k*radius_1*radius_1 * x2*y1 - 2 * k*radius_1*radius_1 * x2*y2 + 2 * k*radius_2*radius_2 * x1*y1 - 2 * k*radius_2*radius_2 * x1*y2 - 2 * k*radius_2*radius_2 * x2*y1 + 2 * k*radius_2*radius_2 * x2*y2 - 2 * k*x1*y1*y1*y1 + 6 * k*x1*y1*y1 * y2 - 6 * k*x1*y1*y2*y2 + 2 * k*x1*y2*y2*y2 + 2 * k*x2*y1*y1*y1 - 6 * k*x2*y1*y1 * y2 + 6 * k*x2*y1*y2*y2 - 2 * k*x2*y2*y2*y2 + 4 * radius_1*radius_1 * y1*y1 - 8 * radius_1*radius_1 * y1*y2 + 4 * radius_1*radius_1 * y2*y2 - x1 * x1 * y1*y1 + 2 * x1*x1 * y1*y2 - x1 * x1 * y2*y2 + 2 * x1*x2*y1*y1 - 4 * x1*x2*y1*y2 + 2 * x1*x2*y2*y2 - x2 * x2 * y1*y1 + 2 * x2*x2 * y1*y2 - x2 * x2 * y2*y2) + 2 * k*k * x1*y1 - 2 * k*k * x1*y2 + 2 * k*y1*y2) / (2 * (y1 - y2 + k * k * y1 - k * k * y2));		
		determined_x = select_point(counter,x1_sol,x2_sol);//这里point对应x坐标
		determined_y = -determined_x / k + (y1 + y2) / 2 + (x1 + x2) / (2 * k) + (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - 2 * y1);
		/*下面为测试所有的X与Y，需要把两个交点都给出*/
		determined_y = -x1_sol / k + (y1 + y2) / 2 + (x1 + x2) / (2 * k) + (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - 2 * y1);
		cout << "两圆交点之一 X= " << x1_sol << "；两圆交点之二 Y= " << determined_y << endl << endl;
		determined_y = -x2_sol / k + (y1 + y2) / 2 + (x1 + x2) / (2 * k) + (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - 2 * y1);
		cout << "两圆交点之二 X= " << x2_sol << "；两圆交点之二 Y= " << determined_y << endl << endl;

		//cout << endl << "find intersection函数中 x1_sol= " << x1_sol << endl;
		//cout << endl << "find intersection函数中 x2_sol= " << x2_sol << endl;
		intersection[0] = determined_x;
		intersection[1] = determined_y;
		return intersection;
	}
	if (y1 = y2 && x1 != x2)//对应两圆心斜率存在且为0
	{
		y1_sol = -(2 * x2*y1 - 2 * x1*y1 + sqrt(-(radius_1 + radius_2 + x1 - x2)*(radius_1 + radius_2 - x1 + x2)*(radius_1 - radius_2 + x1 - x2)*(radius_1 - radius_2 - x1 + x2))) / (2 * (x1 - x2));
		y2_sol =  (2 * x1*y1 - 2 * x2*y1 + sqrt(-(radius_1 + radius_2 + x1 - x2)*(radius_1 + radius_2 - x1 + x2)*(radius_1 - radius_2 + x1 - x2)*(radius_1 - radius_2 - x1 + x2))) / (2 * (x1 - x2));
		determined_y = select_point(counter,y1_sol,y2_sol);
		determined_x = (radius_1*radius_1 - radius_2 * radius_2) / (2 * (x2 - x1)) + (x1 + x2) / 2;
		/*下面为测试所有的X与Y，需要把两个交点都给出*/
		determined_x = (radius_1*radius_1 - radius_2 * radius_2) / (2 * (x2 - x1)) + (x1 + x2) / 2;
		cout << "两圆交点之一 X= " << determined_x << "；两圆交点之二 Y= " << y1_sol << endl << endl;
		cout << "两圆交点之二 X= " << determined_x << "；两圆交点之二 Y= " << y2_sol << endl << endl;

		intersection[0] = determined_x;
		intersection[1] = determined_y;
		return intersection;
	}
	if (x1 == x2 && y1 != y2)//对应两圆心斜率不存在且为无穷
	{
		x1_sol = -(sqrt(-(2 * radius_1*radius_1 - 2 * radius_1*y1 + 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)*(2 * radius_1*radius_1 + 2 * radius_1*y1 - 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)) - 2 * x1*y1 + 4 * x1*y2) / (2 * (y1 - 2 * y2));
		x2_sol =  (sqrt(-(2 * radius_1*radius_1 - 2 * radius_1*y1 + 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)*(2 * radius_1*radius_1 + 2 * radius_1*y1 - 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)) + 2 * x1*y1 - 4 * x1*y2) / (2 * (y1 - 2 * y2));
		determined_x = select_point(counter, x1_sol, x2_sol);//这里point对应x坐标
		determined_y = (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - y1) + (y1 + y2) / 2;
		/*下面为测试所有的X与Y，需要把两个交点都给出*/
		determined_y = (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - y1) + (y1 + y2) / 2;
		cout << "两圆交点之一 X= " << x1_sol << "；两圆交点之二 Y= " << determined_y << endl << endl;
		determined_y = (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - y1) + (y1 + y2) / 2;
		cout << "两圆交点之二 X= " << x2_sol << "；两圆交点之二 Y= " << determined_y << endl << endl;

		intersection[0] = determined_x;
		intersection[1] = determined_y;
		return intersection;
	}
	else//检测到两个圆心重合了--这种情况基本不会发生的
	{
		cout << "Error----Two Circles are coincided！！！ ";
	}
}

double* selectpoint_circle(int counter, double circleCenter_x, double circleCenter_y, float radius, double redundance)//该函数用于在一个圆中找出降落的随机点
{
	int angle;
	double angle_rad;//弧度形式的角度，用于三角函数的计算
	double radius_landing_point;//=radius+redundance
	double* coordinate = new double[2];
	srand(counter);
	angle = rand() % 360;//角度为0-259度之间随机选择
	angle_rad = 3.14159*angle / 180;//转换为弧度，用于三角函数的计算
	radius_landing_point = radius + redundance;
	coordinate[0] = circleCenter_x + radius_landing_point * cos(angle_rad);//X坐标
	coordinate[1] = circleCenter_y + radius_landing_point * sin(angle_rad);//Y坐标
	return coordinate;
}


