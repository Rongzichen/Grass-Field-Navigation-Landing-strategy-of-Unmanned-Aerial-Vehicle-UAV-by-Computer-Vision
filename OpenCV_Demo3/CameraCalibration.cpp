#include "stdafx.h"
#include<CameraCalibration.hpp>
#include<pic_Pre_Process.h>
#include "highgui.h"
#include "iostream"
#include<UAV_process.h>
#include<landing_strategy.h>

using namespace std;
using namespace cv;

CvMat* camera_calibrate(IplImage* img)
{
	double data[9] = { 3122.00608557835,		-7.25889627321663,		    1912.27315909716,
		0,					3105.97908266029,			1554.24043103507,
		0,					0,						    1 };//Intrinsic Matrix Parameter of UAV's camera
	double data2[4] = { 0.104608586397581,-0.272655322644639,  0.0000904110153593259, -0.00415126121296222 };//Distortion Parameter of UAV's Camera
	CvMat intrinsicMatrix;//Intrinsic Matrix
	CvMat distortion_coeff;//Distortion Parameter
	CvMat* map_distortion = new CvMat[2];//��Ϊ�˺�������ͼ������ʼ��ֻʹ��һ�Σ���mapx��mapy��Ϊ�������ݸ�������Ҫ
	cvInitMatHeader(&intrinsicMatrix, 3, 3, CV_64FC1, data);//�����ݴ洢��CvMat֮��
	cvInitMatHeader(&distortion_coeff, 4, 1, CV_64FC1, data2);//���ݴ洢��CvMat֮��

	CvMat *Mapx = cvCreateMat(img->height, img->width, CV_32F);
	CvMat *Mapy = cvCreateMat(img->height, img->width, CV_32F);
	cvInitUndistortMap(&intrinsicMatrix, &distortion_coeff, Mapx, Mapy);
	map_distortion[0] = *cvCloneMat(Mapx);
	map_distortion[1] = *cvCloneMat(Mapy);
	cout << endl << endl;
	return map_distortion;//���ظ�������
}
