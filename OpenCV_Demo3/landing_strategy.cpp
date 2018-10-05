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

/*�Ժ������ַ��������ض����ֵ*/
double* coordinate_transfer(int u,int v,double f,double s,double x0,double X, double Y, double Z, double alpha, double omega, double theta, double k1, double k2, double p1, double p2, double detaX, double detaY, double u0,double v0)
{
	double *cordinator = new double[2];
	cordinator[0] = (-(X*f*s*cos(theta)*cos(omega) - Z * detaX*v*cos(theta)*cos(omega) + Z * detaX*v0*cos(theta)*cos(omega) + Y * f*s*cos(alpha)*sin(omega) + X * detaY*u*sin(theta)*cos(omega) - X * detaY*u0*sin(theta)*cos(omega) - Z * detaY*u*cos(alpha)*sin(omega) + Z * detaY*u0*cos(alpha)*sin(omega) - Y * detaX*v*sin(theta)*cos(omega) + Y * detaX*v0*sin(theta)*cos(omega) - f * s*x0*cos(theta)*cos(omega) - detaY * u*x0*sin(theta)*cos(omega) + detaY * u0*x0*sin(theta)*cos(omega) - X * detaY*u*cos(theta)*sin(alpha)*sin(omega) + X * detaY*u0*cos(theta)*sin(alpha)*sin(omega) + Y * detaX*v*cos(theta)*sin(alpha)*sin(omega) - Y * detaX*v0*cos(theta)*sin(alpha)*sin(omega) + X * f*s*sin(alpha)*sin(theta)*sin(omega) - Z * detaX*v*sin(alpha)*sin(theta)*sin(omega) + Z * detaX*v0*sin(alpha)*sin(theta)*sin(omega) + detaY * u*x0*cos(theta)*sin(alpha)*sin(omega) - detaY * u0*x0*cos(theta)*sin(alpha)*sin(omega) - f * s*x0*sin(alpha)*sin(theta)*sin(omega)) / (f*s*cos(alpha)*cos(theta)*(sin(omega)*sin(omega)) + detaY * u*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u0*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaY * u0*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) + detaX * v*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v0*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + detaX * v0*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + f * s*cos(alpha)*cos(theta)*(cos(omega)*cos(omega)) - detaX * v*cos(theta)*sin(alpha)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(alpha)*cos(omega)*sin(omega) - detaX * v*cos(theta)*sin(theta)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(theta)*cos(omega)*sin(omega)));
	cordinator[1] = ((Y*f*s*cos(alpha)*cos(omega) - Z * detaY*u*cos(alpha)*cos(omega) + Z * detaY*u0*cos(alpha)*cos(omega) - X * f*s*cos(theta)*sin(omega) + Z * detaX*v*cos(theta)*sin(omega) - Z * detaX*v0*cos(theta)*sin(omega) + X * detaY*u*sin(alpha)*sin(omega) - X * detaY*u0*sin(alpha)*sin(omega) - Y * detaX*v*sin(alpha)*sin(omega) + Y * detaX*v0*sin(alpha)*sin(omega) + f * s*x0*cos(theta)*sin(omega) - detaY * u*x0*sin(alpha)*sin(omega) + detaY * u0*x0*sin(alpha)*sin(omega) - X * detaY*u*cos(theta)*sin(alpha)*cos(omega) + X * detaY*u0*cos(theta)*sin(alpha)*cos(omega) + Y * detaX*v*cos(theta)*sin(alpha)*cos(omega) - Y * detaX*v0*cos(theta)*sin(alpha)*cos(omega) + X * f*s*sin(alpha)*sin(theta)*cos(omega) - Z * detaX*v*sin(alpha)*sin(theta)*cos(omega) + Z * detaX*v0*sin(alpha)*sin(theta)*cos(omega) + detaY * u*x0*cos(theta)*sin(alpha)*cos(omega) - detaY * u0*x0*cos(theta)*sin(alpha)*cos(omega) - f * s*x0*sin(alpha)*sin(theta)*cos(omega)) / (f*s*cos(alpha)*cos(theta)*(sin(omega)*sin(omega)) + detaY * u*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u0*cos(alpha)*sin(theta)*(cos(omega)*cos(omega)) - detaY * u*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaY * u0*cos(alpha)*sin(alpha)*(sin(omega)*sin(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(cos(omega)*cos(omega)) + detaX * v*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) + detaX * v*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v0*sin(alpha)*(sin(theta)*sin(theta)) * (cos(omega)*cos(omega)) - detaX * v0*(cos(theta)*cos(theta)) * sin(alpha)*(sin(omega)*sin(omega)) - detaX * v*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + detaX * v0*(sin(alpha)*sin(alpha)) * sin(theta)*(sin(omega)*sin(omega)) + f * s*cos(alpha)*cos(theta)*(cos(omega)*cos(omega)) - detaX * v*cos(theta)*sin(alpha)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(alpha)*cos(omega)*sin(omega) - detaX * v*cos(theta)*sin(theta)*cos(omega)*sin(omega) + detaX * v0*cos(theta)*sin(theta)*cos(omega)*sin(omega)));
	return cordinator;
}

double select_point(int counter, double point1, double point2)//����point1��point2�����ݶ��ǵ�����x����y
{
	int selection;
	srand(counter);//feed seed���Ӷ���֤��һ��ʱ����ѡ���ĵ㶼�������
	selection = rand() % 2;//�������������ѡ��һ���㣬
	if (selection == 0)
	{
		return point1;
	}
	else
		return point2;
}

/*������������е��������ֵ����Ҫ����������ͷ���굽���������ת�����̣����Ըú������ò�����Ϊ��������*/
double *find_intersection(double x1, double x2, double y1, double y2,double r1,double r2,int counter,double gap_length,double radius_UAV)
{
	double k;//��Բ������֮���б��
	double x1_sol, x2_sol, y1_sol, y2_sol;
	double radius_1, radius_2;//���ڽ��뾶�Ŵ���ȷ���������򲻻����ϰ�������
	double determined_x, determined_y;
	double *intersection = new double[2];
	radius_1 = r1+ gap_length+ radius_UAV;//�����������ĵİ뾶��"����뾶+���˻��뾶+����������"
	radius_2 = r2 + gap_length + radius_UAV;
	if (y1 != y2&&x1 != x2)//��Ӧ����Բ�����ڵ�ֱ��б�ʴ����Ҳ�Ϊ0
	{
		k = (y2 - y1) / (x2 - x1);
		x1_sol = -(x1*y2 - x1 * y1 - x2 * y1 + x2 * y2 + k * radius_1*radius_1 - k * radius_2*radius_2 + k * y1*y1 + k * y2*y2 + k * sqrt(-k * k * radius_1*radius_1*radius_1*radius_1 + 2 * k*k * radius_1*radius_1 * radius_2*radius_2 + 2 * k*k * radius_1*radius_1 * y1*y1 - 4 * k*k * radius_1*radius_1 * y1*y2 + 2 * k*k * radius_1*radius_1 * y2*y2 - k * k * radius_2*radius_2*radius_2*radius_2 + 2 * k*k * radius_2*radius_2 * y1*y1 - 4 * k*k * radius_2*radius_2 * y1*y2 + 2 * k*k * radius_2*radius_2 * y2*y2 - k * k * y1*y1*y1*y1 + 4 * k*k * y1*y1*y1 * y2 - 6 * k*k * y1*y1 * y2*y2 + 4 * k*k * y1*y2*y2*y2 - k * k * y2*y2*y2*y2 - 2 * k*radius_1*radius_1 * x1*y1 + 2 * k*radius_1*radius_1 * x1*y2 + 2 * k*radius_1*radius_1 * x2*y1 - 2 * k*radius_1*radius_1 * x2*y2 + 2 * k*radius_2*radius_2 * x1*y1 - 2 * k*radius_2*radius_2 * x1*y2 - 2 * k*radius_2*radius_2 * x2*y1 + 2 * k*radius_2*radius_2 * x2*y2 - 2 * k*x1*y1*y1*y1 + 6 * k*x1*y1*y1 * y2 - 6 * k*x1*y1*y2*y2 + 2 * k*x1*y2*y2*y2 + 2 * k*x2*y1*y1*y1 - 6 * k*x2*y1*y1 * y2 + 6 * k*x2*y1*y2*y2 - 2 * k*x2*y2*y2*y2 + 4 * radius_1*radius_1 * y1*y1 - 8 * radius_1*radius_1 * y1*y2 + 4 * radius_1*radius_1 * y2*y2 - x1 * x1 * y1*y1 + 2 * x1*x1 * y1*y2 - x1 * x1 * y2*y2 + 2 * x1*x2*y1*y1 - 4 * x1*x2*y1*y2 + 2 * x1*x2*y2*y2 - x2 * x2 * y1*y1 + 2 * x2*x2 * y1*y2 - x2 * x2 * y2*y2) - 2 * k*k * x1*y1 + 2 * k*k * x1*y2 - 2 * k*y1*y2) / (2 * (y1 - y2 + k * k * y1 - k * k * y2));
		x2_sol = (x1*y1 - x1 * y2 + x2 * y1 - x2 * y2 - k * radius_1*radius_1 + k * radius_2*radius_2 - k * y1*y1 - k * y2*y2 + k * sqrt(-k * k * radius_1*radius_1*radius_1*radius_1 + 2 * k*k * radius_1*radius_1 * radius_2*radius_2 + 2 * k*k * radius_1*radius_1 * y1*y1 - 4 * k*k * radius_1*radius_1 * y1*y2 + 2 * k*k * radius_1*radius_1 * y2*y2 - k * k * radius_2*radius_2*radius_2*radius_2 + 2 * k*k * radius_2*radius_2 * y1*y1 - 4 * k*k * radius_2*radius_2 * y1*y2 + 2 * k*k * radius_2*radius_2 * y2*y2 - k * k * y1*y1*y1*y1 + 4 * k*k * y1*y1*y1 * y2 - 6 * k*k * y1*y1 * y2*y2 + 4 * k*k * y1*y2*y2*y2 - k * k * y2*y2*y2*y2 - 2 * k*radius_1*radius_1 * x1*y1 + 2 * k*radius_1*radius_1 * x1*y2 + 2 * k*radius_1*radius_1 * x2*y1 - 2 * k*radius_1*radius_1 * x2*y2 + 2 * k*radius_2*radius_2 * x1*y1 - 2 * k*radius_2*radius_2 * x1*y2 - 2 * k*radius_2*radius_2 * x2*y1 + 2 * k*radius_2*radius_2 * x2*y2 - 2 * k*x1*y1*y1*y1 + 6 * k*x1*y1*y1 * y2 - 6 * k*x1*y1*y2*y2 + 2 * k*x1*y2*y2*y2 + 2 * k*x2*y1*y1*y1 - 6 * k*x2*y1*y1 * y2 + 6 * k*x2*y1*y2*y2 - 2 * k*x2*y2*y2*y2 + 4 * radius_1*radius_1 * y1*y1 - 8 * radius_1*radius_1 * y1*y2 + 4 * radius_1*radius_1 * y2*y2 - x1 * x1 * y1*y1 + 2 * x1*x1 * y1*y2 - x1 * x1 * y2*y2 + 2 * x1*x2*y1*y1 - 4 * x1*x2*y1*y2 + 2 * x1*x2*y2*y2 - x2 * x2 * y1*y1 + 2 * x2*x2 * y1*y2 - x2 * x2 * y2*y2) + 2 * k*k * x1*y1 - 2 * k*k * x1*y2 + 2 * k*y1*y2) / (2 * (y1 - y2 + k * k * y1 - k * k * y2));		
		determined_x = select_point(counter,x1_sol,x2_sol);//����point��Ӧx����
		determined_y = -determined_x / k + (y1 + y2) / 2 + (x1 + x2) / (2 * k) + (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - 2 * y1);
		/*����Ϊ�������е�X��Y����Ҫ���������㶼����*/
		determined_y = -x1_sol / k + (y1 + y2) / 2 + (x1 + x2) / (2 * k) + (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - 2 * y1);
		cout << "��Բ����֮һ X= " << x1_sol << "����Բ����֮�� Y= " << determined_y << endl << endl;
		determined_y = -x2_sol / k + (y1 + y2) / 2 + (x1 + x2) / (2 * k) + (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - 2 * y1);
		cout << "��Բ����֮�� X= " << x2_sol << "����Բ����֮�� Y= " << determined_y << endl << endl;

		//cout << endl << "find intersection������ x1_sol= " << x1_sol << endl;
		//cout << endl << "find intersection������ x2_sol= " << x2_sol << endl;
		intersection[0] = determined_x;
		intersection[1] = determined_y;
		return intersection;
	}
	if (y1 = y2 && x1 != x2)//��Ӧ��Բ��б�ʴ�����Ϊ0
	{
		y1_sol = -(2 * x2*y1 - 2 * x1*y1 + sqrt(-(radius_1 + radius_2 + x1 - x2)*(radius_1 + radius_2 - x1 + x2)*(radius_1 - radius_2 + x1 - x2)*(radius_1 - radius_2 - x1 + x2))) / (2 * (x1 - x2));
		y2_sol =  (2 * x1*y1 - 2 * x2*y1 + sqrt(-(radius_1 + radius_2 + x1 - x2)*(radius_1 + radius_2 - x1 + x2)*(radius_1 - radius_2 + x1 - x2)*(radius_1 - radius_2 - x1 + x2))) / (2 * (x1 - x2));
		determined_y = select_point(counter,y1_sol,y2_sol);
		determined_x = (radius_1*radius_1 - radius_2 * radius_2) / (2 * (x2 - x1)) + (x1 + x2) / 2;
		/*����Ϊ�������е�X��Y����Ҫ���������㶼����*/
		determined_x = (radius_1*radius_1 - radius_2 * radius_2) / (2 * (x2 - x1)) + (x1 + x2) / 2;
		cout << "��Բ����֮һ X= " << determined_x << "����Բ����֮�� Y= " << y1_sol << endl << endl;
		cout << "��Բ����֮�� X= " << determined_x << "����Բ����֮�� Y= " << y2_sol << endl << endl;

		intersection[0] = determined_x;
		intersection[1] = determined_y;
		return intersection;
	}
	if (x1 == x2 && y1 != y2)//��Ӧ��Բ��б�ʲ�������Ϊ����
	{
		x1_sol = -(sqrt(-(2 * radius_1*radius_1 - 2 * radius_1*y1 + 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)*(2 * radius_1*radius_1 + 2 * radius_1*y1 - 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)) - 2 * x1*y1 + 4 * x1*y2) / (2 * (y1 - 2 * y2));
		x2_sol =  (sqrt(-(2 * radius_1*radius_1 - 2 * radius_1*y1 + 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)*(2 * radius_1*radius_1 + 2 * radius_1*y1 - 4 * radius_1*y2 - 2 * radius_2*radius_2 + y1*y1 - 3 * y1*y2 + 2 * y2*y2)) + 2 * x1*y1 - 4 * x1*y2) / (2 * (y1 - 2 * y2));
		determined_x = select_point(counter, x1_sol, x2_sol);//����point��Ӧx����
		determined_y = (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - y1) + (y1 + y2) / 2;
		/*����Ϊ�������е�X��Y����Ҫ���������㶼����*/
		determined_y = (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - y1) + (y1 + y2) / 2;
		cout << "��Բ����֮һ X= " << x1_sol << "����Բ����֮�� Y= " << determined_y << endl << endl;
		determined_y = (radius_1*radius_1 - radius_2 * radius_2) / (2 * y2 - y1) + (y1 + y2) / 2;
		cout << "��Բ����֮�� X= " << x2_sol << "����Բ����֮�� Y= " << determined_y << endl << endl;

		intersection[0] = determined_x;
		intersection[1] = determined_y;
		return intersection;
	}
	else//��⵽����Բ���غ���--��������������ᷢ����
	{
		cout << "Error----Two Circles are coincided������ ";
	}
}

double* selectpoint_circle(int counter, double circleCenter_x, double circleCenter_y, float radius, double redundance)//�ú���������һ��Բ���ҳ�����������
{
	int angle;
	double angle_rad;//������ʽ�ĽǶȣ��������Ǻ����ļ���
	double radius_landing_point;//=radius+redundance
	double* coordinate = new double[2];
	srand(counter);
	angle = rand() % 360;//�Ƕ�Ϊ0-259��֮�����ѡ��
	angle_rad = 3.14159*angle / 180;//ת��Ϊ���ȣ��������Ǻ����ļ���
	radius_landing_point = radius + redundance;
	coordinate[0] = circleCenter_x + radius_landing_point * cos(angle_rad);//X����
	coordinate[1] = circleCenter_y + radius_landing_point * sin(angle_rad);//Y����
	return coordinate;
}


