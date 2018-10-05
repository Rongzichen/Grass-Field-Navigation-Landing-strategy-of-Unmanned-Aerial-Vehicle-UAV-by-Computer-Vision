#include "stdafx.h"
/*���ڴ洢ͷ�ļ���������-Ԥ�����йغ����Ķ���*/
#include<pic_Pre_Process.h>
#include<CameraCalibration.hpp>
#include "highgui.h"
#include "iostream"
#include<UAV_process.h>
#include<landing_strategy.h>
#define CVX_RED CV_RGB(0xff, 0x00, 0x00)  
#define CVX_GREEN CV_RGB(0x00, 0xff, 0x00)  
#define CVX_BLUE CV_RGB(0x00, 0x00, 0xff) 
#define CV_RGB( r, g, b )  cvScalar( (b), (g), (r), 0 )  
using namespace std;

struct Pre_process //����ṹ��-��Ԥ�����ֺ�����ȡ����Ҫ��������Ϣ����֮��ĺ�����
{
	CvSeq* contour[10];
	double radius_contour;
	CvPoint2D32f* circle_center[10];
	IplImage* processed_pic;
};
struct circle
{
	double x[100], y[100], r[100];
};

void main_preprocess(IplImage* frame1,IplImage* frame2,CvCapture* capture)
{
	IplImage* img_thresh2 = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//������ֵ�������洢�Ŀռ�
	IplImage* img_real_show_contours = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//չʾ�Ҷ�ͼ�е�������ͼƬ
	IplImage* img_binary = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//���ж�ֵ��������ͼ��
	IplImage* img_flip = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//�洢��ַ����֡ͼ��
	IplImage* img_median_filter = cvCreateImage(cvGetSize(frame2), IPL_DEPTH_8U, 1);//�Ƚ�����ֵ�˲�
	IplImage* img_show_contours = cvCreateImage(cvGetSize(frame2), IPL_DEPTH_8U, 3);//���ڴ洢������Ϣ�Ĳ�ͼ
	CvMemStorage* storage = cvCreateMemStorage();//����֮�����ڴ洢������Ϣ�ı���
	CvSeq* first_contour = NULL;
	CvSeq* storage_seq[100]; //ָ�����飬���ڴ洢������Ϣ�ı�����֮��Ӧ�÷ֱ��ʼ����
	CvSeq* storage_seq_effecitve[100];//ָ�����飬���ڴ洢������ֵ����ȷ����Ч������
	CvMemStorage* storage_initiate = cvCreateMemStorage();//���ڳ�ʼ��storage[10]
	int counter = 0;//�������ڸó��������еط���ѭ���ṹ
	int counter2 = 0;//������counter�е�ѭ���ṹ�е�ѭ���ṹ
	CvPoint* centers[100];//���ڴ洢��������λ��
	double *distance_p2p[100];//���ڴ洢�����֮��ľ���
	double largest_distance;
	float radius;//���ڴ洢��Բ�������������״�İ뾶
	CvPoint2D32f* center_circle[10];//���ڴ洢Բ����ϵ�Բ��
	double Xg, Yg, Zg, height;//���ڴ洢���˻�������й���Ϣ
	int threshold_judge_effective;//�����ж������Ƿ���Ч---ͨ����߶Ƚ��бȶԡ�
	circle circle_world;//��ͼ��������ϵ�ԲͶ�������������Բ�Ĳ������洢�ڴ�
	double *cordinator = new double[2];//���ڴ洢������ת���������������������µ�X/Y��ֵ��������������ʱ�洢�ģ�����Ľṹ�����������ʽ�洢�ġ�
	double *cordinator2 = new double[2];//���ڴ洢�������ѡ������˻����վ�����½�ĵ�
	cvNamedWindow("Previous", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("Review", CV_WINDOW_AUTOSIZE);
	CvMat* mapx_distortion = NULL;//���ڴ洢��Camera_calibration�����д�����mapx����
	CvMat* mapy_distortion = NULL;//���ڴ洢��Camera����Calibration�����д�����Mapy����
	CvMat* map_distortion = new CvMat[2];
	int goto_rightnow = 0;//����͹�����У������͹��ֵΪ1��Ȼ������꽻��󣬿���ʹ��goto END

	map_distortion = camera_calibrate(frame2);//��ȡͼ�������ӳ�䣬ֻ����һ�δӶ����ټ�����������֮���cvRemap��
	mapx_distortion = cvCloneMat(&map_distortion[0]);
	mapy_distortion = cvCloneMat(&map_distortion[1]);

	
	for (int counter = 0; counter < 100; counter++)//��ʼ����ָ������--�洢����������Ϣ�Լ����ĵ���Ϣ
	{
		storage_seq[counter] = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage_initiate);
		storage_seq_effecitve[counter] = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage_initiate);
		centers[counter] = &cvPoint(0,0);
		center_circle[counter] = &cvPoint2D32f(0,0);
	}
	while (1)
	{
		dji_drones UAV = get_UAV_info();//���ڴ����˻���ȡ�й�������Ϣ
		frame2 = skip_frame(capture, frame2, 0);//������Ϊ��֮֡����0֡��
		if (!frame2)
		{
			cout << endl << "��Ƶ�������" << endl;
			break;
		}
		/**************���ڴ˴�����ͼ��Ľ�����**************/
		IplImage* img_non_distortion = cvCloneImage(frame2);
		cvRemap(frame2, img_non_distortion, mapx_distortion, mapy_distortion);
		/***************��The End��*************************/
		cvShowImage("Previous", frame2);//չʾԭͼ��Լ1.5��
		cvWaitKey(1500);
		cvCvtColor(frame2, img_binary, CV_RGB2GRAY);//ת��Ϊ�Ҷ�ͼ
		cvShowImage("Previous", img_binary);//չʾ��ֵ��ͼ��Լ1.5��
		cvCopy(img_binary, img_real_show_contours,NULL);//�Ѷ�ֵ��ͼ��������У�����չʾ����

		cvWaitKey(1500);
		cvThreshold(img_binary, img_thresh2, 50, 255, CV_THRESH_BINARY);//�Ժ�һ֡��ͼ����ж�ֵ��������Ϊ50�ڴ�Լ1�׸߲ݵ��ϽϺã����������͸����ˣ�С��������Ե������ԡ�
		cvShowImage("Previous", img_thresh2);//չʾ��ֵ��ͼ��Լ1.5��
		cvWaitKey(1500);

		img_flip = reverse(img_thresh2);
		find_connected_components(img_flip, 1, 50, NULL, NULL, NULL);//��������15��Ϊ50��
		cvShowImage("Previous", img_flip);//չʾ��ͨ��������ͼ��
		cvWaitKey(1500);//Լ1.5s
		/*�����������Ѿ����Һͷָ�����ˣ��������Ƕ�����������״������*/
		int num_contours = cvFindContours(img_flip, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST);
		int counter = 0;
		counter2 = 0;//�ü���ֵ��ʾ��Ч����������������ֵ������������ȷ�����ϰ����������������ֵ��
		for (CvSeq* c = first_contour; c != NULL; c = c->h_next)
		{
			storage_seq[counter] = c;//���������Ѿ�ȫ���洢�������������
			cvCvtColor(img_real_show_contours,img_show_contours,CV_GRAY2BGR);
			cvDrawContours
			(img_show_contours,//���ڻ���������ͼ��
				storage_seq[counter],			   //ָ��Ŀǰ�������ڵ�ַ�ռ� 
				CVX_RED,       //���������ɫ 
				CVX_GREEN,     //�ڲ�������ɫ  
				0,             //�ȼ�Ϊ0�����Ƶ���������
				2,             //����������ϸ
				8              //�߶�����Ϊ��8�ڽ�)������
				);
			threshold_judge_effective = (int)400 / UAV.hieght;//���ڶ�Ӧ��ͬ�߶ȵ����˻��������Ч����Ԫ����Ŀ��ͬ��������1�׸߶���200Ϊ��ֵ���ڲ�ͬ�߶���ֵ��200���Ը߶ȼ��ɡ�
			
			if (storage_seq[counter]->total < threshold_judge_effective)//���������Ԫ����Ŀ������ֵ������
			{
				;
			}
			else
			{
				storage_seq_effecitve[counter2] = storage_seq[counter];
				/*�����潫����������Բ�ε���ϡ���������п��Ա���Բ���Ҫ���������л��Ƽ���������*/
				cvMinEnclosingCircle(storage_seq_effecitve[counter2], center_circle[counter2], &radius);//���������Ӧ������Բ�����
				cout << "Բ�ĵ��������� X: " << center_circle[counter]->x << endl;
				cout << "Բ�ĵ��������� Y: " << center_circle[counter]->y << endl;
				cout << "���������µİ뾶: " << radius << endl;
				cvCircle(img_show_contours, cvPoint(center_circle[counter2]->x, center_circle[counter2]->y), (int)radius, CV_RGB(255, 255, 150), 2, 8, 0);//�Ѿ����ƺõ����������������ﶨ���ͼƬ��
				cvShowImage("Previous", img_show_contours);//չʾ����ͼ
				cvWaitKey(1500);

				//cordinator = coordinate_transfer((int)center_circle[counter2]->x, (int)center_circle[counter2]->y, 5, 0, 0, 2.5, 2.2, 0, 0, 1.571, 3.1415, 0.2831, -1.9707, 0.0000904110153593259, -0.00415126121296222, 0.00133, 0.00133, 1912.27315909716, 1554.24043103507);
				cordinator = coordinate_transfer(center_circle[counter2]->x, center_circle[counter2]->y, 5, 0.17, 0, 0,2.2,2.5, 0, 1.571, 3.1415, 0.2831, -1.9707, 0.0000904110153593259, -0.00415126121296222, 0.0016, 0.0016, 150,-550);
				circle_world.x[counter2] = cordinator[0];//�˴������������µ�x��ֵ��ֵ���ṹ���е�x
				circle_world.y[counter2] = cordinator[1];
				cout << "���������µ�Բ�� X: " << circle_world.x[counter2] << endl;
				cout << "���������µ�Բ�� Y: " << circle_world.y[counter2] << endl;

				//������������ͼ�������ڵļ��㣬��Ҫ��ת��Ϊ���������
				cordinator = selectpoint_circle(storage_seq_effecitve[counter2]->total, center_circle[counter2]->x, center_circle[counter2]->y, radius, 0);
				//�����ǽ�ͼ������ת��Ϊ��������
				cordinator2 = coordinate_transfer(cordinator[0], cordinator[1], 5, 0.17, 0, 1.85, 1.35, 2.5, 0, 1.571, 3.1415, 0.2831, -1.9707, 0.0000904110153593259, -0.00415126121296222, 0.0016, 0.0016, 150, -550);
				circle_world.x[98] = cordinator2[0];//�˴������������µ�x��ֵ��ֵ���ṹ���е�x
				circle_world.y[98] = cordinator2[1];
				
				//���ù��ɶ�����������������ϵ��Բ�İ뾶
				circle_world.r[counter2] = sqrt(abs(pow((circle_world.x[counter2] - circle_world.x[98]), 2) - pow((circle_world.y[counter2] - circle_world.y[98]), 2)));
				cout << "��������ϵ��Բ�İ뾶 R: " << circle_world.r[counter2]<<endl<<endl;
				//cout << endl << "���˻���������ϵ�� Y= " << circle_world.y[counter2]<<endl;
				counter2++;//������Ԫ����Ŀ������ֵ��ʱ�򣬲Ż��������ֵ���ۼӣ��Ҹ�ѭ��������counter2��Ϊ��֡����Ч����������Ŀ��
			}
															
			counter++;
			cvWaitKey(1000);//����ʱ1�롿
			
		
		}
		cordinator = find_intersection(circle_world.x[0], circle_world.x[1], circle_world.y[0], circle_world.y[1], circle_world.r[0], circle_world.r[0],counter,1,0.4);

		/*  ����Ԥ�������е��м�ֵ��Ϣ���˵��ṹ���У��Ӷ������ں�������ʱ��֮ת�Ƶ�����������   */
		//���Բ��֣���ʾͼ����������
		break;
	}
	cvWaitKey(-1);
	//cvReleaseCapture( &capture );
	cvDestroyWindow("Review");
	cvDestroyWindow("Previous");
}



/*ԭʼͼ������3ͨ�����������ɵ�ͨ������*/
void double_thresh(CvArr* src, CvArr* dst, double threshold, double max_value, int threshold_type)
{
	IplImage* gray_img = sum_rgb(src);//This is a one Channel image
	cvThreshold(gray_img, dst, threshold, 255, CV_THRESH_BINARY);//������ط���threshold֮ǰ���趨Ϊ��100��
	
}

/*��R��B��G��ͨ���ĻҶ�ֵ�������*/
IplImage* sum_rgb(CvArr* src)
{

	IplImage* r = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);//allocate individual image planes
	IplImage* g = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	IplImage* b = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);

	cvSplit(src, r, g, b, NULL);//Split image into the color planes *The most important part !!!*

	IplImage* temp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);//Temporary storage

	cvAddWeighted(r, 1. / 3., g, 1. / 3., 0.0, temp);//Add equally weightd rgb values
	cvAddWeighted(temp, 2. / 3., b, 1. / 3., 0.0, temp);

	return(temp);//return a single channel image pointer 
}

IplImage* reverse(IplImage* src)//ע��width��ָ���У�height��ָ����
{
	IplImage* img_rever = cvCreateImage(cvGetSize(src), src->depth, 1);
	IplImage* img_rever2 = cvCreateImage(cvGetSize(src), src->depth, 1);
	for (int i = 0; i<src->height; i++)
	{
		for (int j = 0; j<src->width; j++)
		{
			img_rever->imageData[i*src->widthStep + j] = 255 - src->imageData[i*src->widthStep + j];
		}
	}
	return(img_rever);
}

IplImage* frame_diff(IplImage* frame1, IplImage* frame2, int threshold, int max_value, int threshold_type)
{
	IplImage* frame_after_proc = cvCreateImage(cvGetSize(frame1), IPL_DEPTH_8U, 1);
	IplImage* frame_subtracted = cvCreateImage(cvGetSize(frame1), IPL_DEPTH_8U, 1);
	cvAbsDiff(frame1,frame2, frame_subtracted);
	cvThreshold(frame_subtracted,frame_after_proc,15,255,CV_THRESH_BINARY);//15��ʾ����15�ĻҶ�ֵ��Ӧ�ú��ԣ�255��ʾ�Ҷ�ֵ�����������ʾ����15�ĻҶ�ֵ�����Ϊ255��ʹ��Ŀ��Ϊ��ɫ��
	return(frame_after_proc);
}

IplImage* pic_flip(IplImage* src, int mode)
{
	IplImage* img_flip = cvCreateImage(cvSize(src->height, src->width), IPL_DEPTH_8U, 1);
	cvTranspose(src, img_flip);//֮����Ե����ó�ȥ��Ϊһ������ʹ��
	cvFlip(img_flip, NULL, mode);
	return(img_flip);
}

void find_connected_components(IplImage* mask, int poly1_hull0, double perimScale, int* num, CvRect* bbs, CvPoint* centers)
{
	static CvMemStorage* mem_storage = NULL;
	static CvSeq* contours = NULL;
	//CLEAN UP RAW MASK
	cvMorphologyEx(mask, mask, 0, 0, CV_MOP_OPEN, 1);
	cvMorphologyEx(mask, mask, 0, 0, CV_MOP_CLOSE, 1);
	if (mem_storage == NULL)
	{
		mem_storage = cvCreateMemStorage(0);
	}
	else
	{
		cvClearMemStorage(mem_storage);
	}
	CvContourScanner scanner = cvStartFindContours(mask, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	CvSeq* c;
	int numCont = 0;
	while ((c = cvFindNextContour(scanner)) != NULL)
	{
		double len = cvContourPerimeter(c);
		double q = (mask->height + mask->width) / perimScale;
		if (len < q)
		{
			cvSubstituteContour(scanner, NULL);
		}
		else
		{
			//Smooth its edges if its large enough
			CvSeq* c_new;
			if (poly1_hull0)
			{
				//Polygonal approximation
				c_new = cvApproxPoly(c, sizeof(CvContour), mem_storage, CV_POLY_APPROX_DP, 2);
			}
			else
			{
				//Convex hull of the segmentation
				c_new = cvConvexHull2(c, mem_storage, CV_CLOCKWISE, 1);
			}
			cvSubstituteContour(scanner, c_new);
			numCont++;
		}
	}
	contours = cvEndFindContours(&scanner);

	//Just some convenience variables
	const CvScalar CVX_WHITE = CV_RGB(0xff, 0xff, 0xff);
	const CvScalar CVX_BLACE = CV_RGB(0x00, 0x00, 0x00);
	//PAINT THE FOUND REGIONS BACK INTO THE IMAGE
	//
	cvZero(mask);
	IplImage *maskTemp;
	if (num != NULL)
	{
		int N = *num, numFilled = 0, i = 0;
		CvMoments moments;
		double M00, M01, M10;
		maskTemp = cvCloneImage(mask);
		for (i = 0, c = contours; c != NULL; c = c->h_next, i++)
		{
			if (i < N)
			{
				cvDrawContours(maskTemp, c, CVX_WHITE, CVX_WHITE, -1, CV_FILLED, 8);
				if (centers != NULL)
				{
					cvMoments(maskTemp, &moments, 1);
					M00 = cvGetSpatialMoment(&moments, 0, 0);
					M10 = cvGetSpatialMoment(&moments, 1, 0);
					M01 = cvGetSpatialMoment(&moments, 0, 1);
					centers[i].x = (int)(M10 / M00);
					centers[i].y = (int)(M01 / M00);
				}
				if (bbs != NULL)
				{
					bbs[i] = cvBoundingRect(c);
				}
				cvZero(maskTemp);
				numFilled++;
			}
			cvDrawContours(mask, c, CVX_WHITE, CVX_WHITE, -1, CV_FILLED, 8);
		}
		*num = numFilled;
		cvReleaseImage(&maskTemp);
	}
	else
	{
		for (c = contours; c != NULL; c = c->h_next)
		{
			cvDrawContours(mask, c, CVX_WHITE, CVX_BLACE, -1, CV_FILLED, 8);
		}
	}
}

IplImage* skip_frame(CvCapture* capture, IplImage* frame2,int num)
{
	IplImage* frame = cvCreateImage(cvGetSize(frame2), IPL_DEPTH_8U, 1);
	int count;
	for (count = 0; count < num+1; count++)
	{
		frame = cvQueryFrame(capture);
	}
	return(frame);
}

CvPoint* find_center_point(CvSeq* mask,CvPoint* centers,int counter)
{
	CvMoments moments;
	double M00, M01, M10;
	//cvZero(mask);
	CvSeq* maskTemp;
	maskTemp = cvCloneSeq(mask);
	cvMoments(maskTemp, &moments, 1);//�����ͼ��һ����ҪΪ��ֵͼ�񣬲�������ͨ��ͼ
	M00 = cvGetSpatialMoment(&moments, 0, 0);
	M10 = cvGetSpatialMoment(&moments, 1, 0);
	M01 = cvGetSpatialMoment(&moments, 0, 1);
	centers[counter].x = (int)(M10 / M00);//һ��ͼ�����кܶ��point��������i����ʲô��˼��
	centers[counter].y = (int)(M01 / M00);
	return &centers[counter];
}

double cal_distance(CvPoint* center,CvPoint* boundary)
{
	double distance;
	distance = powf((center->x - boundary->x), 2) + powf((center->y - boundary->y), 2);
	distance = sqrtf(distance);
	return distance;
}

double find_largest(double *distance[10])
{
	double largest = *distance[0];
	for (int i = 1; i < 10; i++)
	{
		double next = *distance[i];
		if ((next - largest) > 0.01)
		{
			largest = next;
		}
		else
			;
	}
	return largest;
}


/*
1.ֻҪ��Դ�ļ��õ��˵�ͷ�ļ�������Ҫinclude����
2.Ҳ��Ҫ����include stdafx.h
3.cvDrawContours()������ֱ����ʾ�޸ĺ��ͼ�񣬶��Ƕ�֮ǰ��ԭͼ��������޸ģ���ʱ��ʾԭͼ�񼴿ɡ�
4.frame_diff()����ʹ��������֡ͼ��֮����������ͨ����ֵ���㣬���Ի�ñ���Ϊ��ɫ��Ŀ��Ϊ��ɫ��֡��ͼ��������������OpenCV�Ŀ⺯����
5.ע����ںܶ�ͼ����ĺ�������ԭͼ��Ϳ�ͼ��Ŀռ�Ӧ��һģһ�����ڸ�����Ƶ��ͼ���������ʽ��ʱ����������˽ϴ����⣬img_thresh��ЩӦ���ǵ�ͨ���ģ�����֮ǰȴ��frame���ͨ��������һ���ˣ����Բ��ԡ�
6.���������쳣: д�����Ȩ�޳�ͻ������������ڸ������Ҫд��Ķ���û�г�ʼ���ã������ǳ�ʼ����ʱ��ͳ��������⣬û��ʹ�ú��ʵĺ����Ӷ�ʹ���޷�д�����ݽ�ȥ��
*/