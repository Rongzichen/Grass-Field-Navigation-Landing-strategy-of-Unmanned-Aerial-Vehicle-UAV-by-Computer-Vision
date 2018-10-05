#ifndef pic_Pre_Process_H
#define pic_Pre_Process_H
#include <highgui.h>
#include<iostream>
/*���ڴ��ͼ��Ԥ�������Ĳ���*/
void double_thresh(CvArr* src, CvArr* dst, double threshold, double max_value, int threshold_type);//��ͼ����RGB������ͺ���ж�ֵ��������
IplImage* sum_rgb(CvArr* src);//��RGB������͵ĺ���
void find_connected_components(IplImage* mask,int poly1_hull0,double perimScale,int* num,CvRect* bbs,CvPoint* centers);//����ͨ������ͨ�ķ�ʽȥ�������е�������
IplImage* frame_diff(IplImage* frame1, IplImage* frame2, int threshold, int max_value, int threshold_type);//Ѱ��ͼ�����ͨ����
IplImage* pic_flip(IplImage* src, int mode);//��ͼ�����90����ת���Ӷ����㿴ͼ�������������
void main_preprocess(IplImage* frame1, IplImage* frame2, CvCapture* capture);//Ԥ�����������
IplImage* skip_frame(CvCapture* capture, IplImage* frame2, int num);//�ú�����������������֡�������صľ����Թ�һ����Ŀ֮���֡��num��ʾ��֮֡��������֡������ֹ�˶�����ʶ�𲻳�֡�
IplImage* reverse(IplImage* src);//��ͼ��Ҷ��󷴵ĺ���
CvPoint* find_center_point(CvSeq* mask, CvPoint* centers, int counter);//���ڸ����Ѿ�֪����������Ϣ�ҳ������ľ��ο����ĵ�
double cal_distance(CvPoint* center, CvPoint* boundary);//���������ص�֮��ľ���
double find_largest(double *distance[10]);//����10��double������֮��������ֵ

#endif

/*
1. ͷ�ļ���ҲҪinclude�����õ��˵�ͷ�ļ�.....
2. ͷ�ļ�Ҫ�������ʹ�ã�����Ҫ�ڡ�����-VC++Ŀ¼-����Ŀ¼���������ͷ�ļ����ڵ��ļ���
3. ����ĵط�Ҫ�㿪�������Ͽ������������������
4. �Լ���ʱ����������ʽ������ȷ������㿪��������ȫ��
*/