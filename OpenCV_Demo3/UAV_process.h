#ifndef UAV_PROCESS_H
#define UAV_PROCESS_H
#include <highgui.h>
#include<iostream>
struct dji_drones //�����֮ǰ���а汾��drones���Ϊ��dji_drones�࣬��Ҫ��Ϊ�˷�ֹ��DJI_SDK�з�����ͻ
{
	double Xg, Yg, Zg;//���˻���������---���˻�����
	double Xc, Yc, Zc;//���˻���ͷ�ڲ��ľ�������
	double hieght, speed;//���˻�����������--�߶Ⱥ��ٶ�
	double destine_Xg, destine_Yg, destine_Zg;//���˻�Ŀ������
};
dji_drones get_UAV_info(void);


#endif


