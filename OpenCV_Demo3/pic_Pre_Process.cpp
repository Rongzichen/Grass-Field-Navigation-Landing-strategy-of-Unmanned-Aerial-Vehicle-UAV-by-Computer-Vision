#include "stdafx.h"
/*用于存储头文件声明函数-预处理有关函数的定义*/
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

struct Pre_process //定义结构体-从预处理部分函数获取所需要的所有信息用于之后的函数中
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
	IplImage* img_thresh2 = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//进行阈值化处理后存储的空间
	IplImage* img_real_show_contours = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//展示灰度图中的轮廓的图片
	IplImage* img_binary = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//进行二值化处理后的图像
	IplImage* img_flip = cvCreateImage(cvGetSize(frame2), frame2->depth, 1);//存储差分法后的帧图像
	IplImage* img_median_filter = cvCreateImage(cvGetSize(frame2), IPL_DEPTH_8U, 1);//先禁用中值滤波
	IplImage* img_show_contours = cvCreateImage(cvGetSize(frame2), IPL_DEPTH_8U, 3);//用于存储轮廓信息的彩图
	CvMemStorage* storage = cvCreateMemStorage();//创建之后用于存储轮廓信息的变量
	CvSeq* first_contour = NULL;
	CvSeq* storage_seq[100]; //指针数组，用于存储轮廓信息的变量，之后应该分别初始化。
	CvSeq* storage_seq_effecitve[100];//指针数组，用于存储经过阈值检测后确认有效的轮廓
	CvMemStorage* storage_initiate = cvCreateMemStorage();//用于初始化storage[10]
	int counter = 0;//可以用于该程序中所有地方的循环结构
	int counter2 = 0;//用于在counter中的循环结构中的循环结构
	CvPoint* centers[100];//用于存储轮廓中心位置
	double *distance_p2p[100];//用于存储点与点之间的距离
	double largest_distance;
	float radius;//用于存储以圆形来拟合轮廓形状的半径
	CvPoint2D32f* center_circle[10];//用于存储圆形拟合的圆心
	double Xg, Yg, Zg, height;//用于存储无人机自身的有关信息
	int threshold_judge_effective;//用于判断轮廓是否有效---通过与高度进行比对。
	circle circle_world;//将图像中所拟合的圆投射回世界坐标后的圆的参数将存储于此
	double *cordinator = new double[2];//用于存储从坐标转换函数回来的世界坐标下的X/Y数值，但是是用于临时存储的，上面的结构体变量才是正式存储的。
	double *cordinator2 = new double[2];//用于存储经过随机选择后，无人机最终决定着陆的点
	cvNamedWindow("Previous", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("Review", CV_WINDOW_AUTOSIZE);
	CvMat* mapx_distortion = NULL;//用于存储从Camera_calibration函数中传来的mapx数据
	CvMat* mapy_distortion = NULL;//用于存储从Camera——Calibration函数中传来的Mapy数据
	CvMat* map_distortion = new CvMat[2];
	int goto_rightnow = 0;//用于凸起检测中，检测完凸起赋值为1，然后等求完交点后，可以使用goto END

	map_distortion = camera_calibrate(frame2);//获取图像矫正的映射，只运行一次从而减少计算量，用于之后的cvRemap中
	mapx_distortion = cvCloneMat(&map_distortion[0]);
	mapy_distortion = cvCloneMat(&map_distortion[1]);

	
	for (int counter = 0; counter < 100; counter++)//初始化该指针数组--存储轮廓变量信息以及中心点信息
	{
		storage_seq[counter] = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage_initiate);
		storage_seq_effecitve[counter] = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage_initiate);
		centers[counter] = &cvPoint(0,0);
		center_circle[counter] = &cvPoint2D32f(0,0);
	}
	while (1)
	{
		dji_drones UAV = get_UAV_info();//用于从无人机获取有关数据信息
		frame2 = skip_frame(capture, frame2, 0);//【设置为两帧之间间隔0帧】
		if (!frame2)
		{
			cout << endl << "视频播放完毕" << endl;
			break;
		}
		/**************【在此处进行图像的矫正】**************/
		IplImage* img_non_distortion = cvCloneImage(frame2);
		cvRemap(frame2, img_non_distortion, mapx_distortion, mapy_distortion);
		/***************【The End】*************************/
		cvShowImage("Previous", frame2);//展示原图，约1.5秒
		cvWaitKey(1500);
		cvCvtColor(frame2, img_binary, CV_RGB2GRAY);//转换为灰度图
		cvShowImage("Previous", img_binary);//展示二值化图，约1.5秒
		cvCopy(img_binary, img_real_show_contours,NULL);//把二值化图像存于其中，用于展示轮廓

		cvWaitKey(1500);
		cvThreshold(img_binary, img_thresh2, 50, 255, CV_THRESH_BINARY);//对后一帧的图像进行二值化处理【设为50在大约1米高草地上较好，大了噪声就更多了，小了轮廓边缘会更不对】
		cvShowImage("Previous", img_thresh2);//展示阈值化图，约1.5秒
		cvWaitKey(1500);

		img_flip = reverse(img_thresh2);
		find_connected_components(img_flip, 1, 50, NULL, NULL, NULL);//【参数由15调为50】
		cvShowImage("Previous", img_flip);//展示连通话处理后的图像
		cvWaitKey(1500);//约1.5s
		/*【现在轮廓已经查找和分割完成了，接下来是对轮廓进行形状分析】*/
		int num_contours = cvFindContours(img_flip, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST);
		int counter = 0;
		counter2 = 0;//该计数值表示有效轮廓的数量（索引值），即被发现确定是障碍物的轮廓的索引数值。
		for (CvSeq* c = first_contour; c != NULL; c = c->h_next)
		{
			storage_seq[counter] = c;//现在轮廓已经全部存储在了这个数组中
			cvCvtColor(img_real_show_contours,img_show_contours,CV_GRAY2BGR);
			cvDrawContours
			(img_show_contours,//用于绘制轮廓的图像
				storage_seq[counter],			   //指向目前轮廓所在地址空间 
				CVX_RED,       //外层轮廓颜色 
				CVX_GREEN,     //内层轮廓颜色  
				0,             //等级为0，绘制单独的轮廓
				2,             //轮廓线条粗细
				8              //线段类型为（8邻接)连接线
				);
			threshold_judge_effective = (int)400 / UAV.hieght;//用于对应不同高度的无人机其检测的有效轮廓元素数目不同，所以在1米高度以200为阈值，在不同高度阈值以200除以高度即可。
			
			if (storage_seq[counter]->total < threshold_judge_effective)//如果轮廓中元素数目少于阈值，舍弃
			{
				;
			}
			else
			{
				storage_seq_effecitve[counter2] = storage_seq[counter];
				/*【下面将对轮廓进行圆形的拟合】在这里进行可以避免对不必要的轮廓进行绘制减少运算量*/
				cvMinEnclosingCircle(storage_seq_effecitve[counter2], center_circle[counter2], &radius);//计算出来对应轮廓的圆形拟合
				cout << "圆心的像素坐标 X: " << center_circle[counter]->x << endl;
				cout << "圆心的像素坐标 Y: " << center_circle[counter]->y << endl;
				cout << "像素坐标下的半径: " << radius << endl;
				cvCircle(img_show_contours, cvPoint(center_circle[counter2]->x, center_circle[counter2]->y), (int)radius, CV_RGB(255, 255, 150), 2, 8, 0);//已经绘制好的轮廓将出现在这里定义的图片中
				cvShowImage("Previous", img_show_contours);//展示轮廓图
				cvWaitKey(1500);

				//cordinator = coordinate_transfer((int)center_circle[counter2]->x, (int)center_circle[counter2]->y, 5, 0, 0, 2.5, 2.2, 0, 0, 1.571, 3.1415, 0.2831, -1.9707, 0.0000904110153593259, -0.00415126121296222, 0.00133, 0.00133, 1912.27315909716, 1554.24043103507);
				cordinator = coordinate_transfer(center_circle[counter2]->x, center_circle[counter2]->y, 5, 0.17, 0, 0,2.2,2.5, 0, 1.571, 3.1415, 0.2831, -1.9707, 0.0000904110153593259, -0.00415126121296222, 0.0016, 0.0016, 150,-550);
				circle_world.x[counter2] = cordinator[0];//此处将世界坐标下的x数值赋值给结构体中的x
				circle_world.y[counter2] = cordinator[1];
				cout << "世界坐标下的圆心 X: " << circle_world.x[counter2] << endl;
				cout << "世界坐标下的圆心 Y: " << circle_world.y[counter2] << endl;

				//下面计算的是在图像坐标内的计算，需要再转换为世界坐标的
				cordinator = selectpoint_circle(storage_seq_effecitve[counter2]->total, center_circle[counter2]->x, center_circle[counter2]->y, radius, 0);
				//下面是将图像坐标转换为世界坐标
				cordinator2 = coordinate_transfer(cordinator[0], cordinator[1], 5, 0.17, 0, 1.85, 1.35, 2.5, 0, 1.571, 3.1415, 0.2831, -1.9707, 0.0000904110153593259, -0.00415126121296222, 0.0016, 0.0016, 150, -550);
				circle_world.x[98] = cordinator2[0];//此处将世界坐标下的x数值赋值给结构体中的x
				circle_world.y[98] = cordinator2[1];
				
				//利用勾股定理计算出来世界坐标系下圆的半径
				circle_world.r[counter2] = sqrt(abs(pow((circle_world.x[counter2] - circle_world.x[98]), 2) - pow((circle_world.y[counter2] - circle_world.y[98]), 2)));
				cout << "世界坐标系下圆的半径 R: " << circle_world.r[counter2]<<endl<<endl;
				//cout << endl << "无人机世界坐标系下 Y= " << circle_world.y[counter2]<<endl;
				counter2++;//在轮廓元素数目大于阈值的时候，才会进行索引值的累加，且该循环结束后，counter2即为该帧中有效的轮廓总数目。
			}
															
			counter++;
			cvWaitKey(1000);//【延时1秒】
			
		
		}
		cordinator = find_intersection(circle_world.x[0], circle_world.x[1], circle_world.y[0], circle_world.y[1], circle_world.r[0], circle_world.r[0],counter,1,0.4);

		/*  将该预处理函数中的有价值信息搬运到结构体中，从而可以在函数结束时将之转移到其它函数中   */
		//测试部分：显示图像处理后的样张
		break;
	}
	cvWaitKey(-1);
	//cvReleaseCapture( &capture );
	cvDestroyWindow("Review");
	cvDestroyWindow("Previous");
}



/*原始图像本来是3通道，处理完后成单通道的了*/
void double_thresh(CvArr* src, CvArr* dst, double threshold, double max_value, int threshold_type)
{
	IplImage* gray_img = sum_rgb(src);//This is a one Channel image
	cvThreshold(gray_img, dst, threshold, 255, CV_THRESH_BINARY);//【这个地方的threshold之前被设定为了100】
	
}

/*对R、B、G三通道的灰度值进行求和*/
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

IplImage* reverse(IplImage* src)//注意width是指的列，height是指的行
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
	cvThreshold(frame_subtracted,frame_after_proc,15,255,CV_THRESH_BINARY);//15表示低于15的灰度值差应该忽略，255表示灰度值最大差，最后参数表示大于15的灰度值差的设为255（使得目标为白色）
	return(frame_after_proc);
}

IplImage* pic_flip(IplImage* src, int mode)
{
	IplImage* img_flip = cvCreateImage(cvSize(src->height, src->width), IPL_DEPTH_8U, 1);
	cvTranspose(src, img_flip);//之后可以单独拿出去作为一个函数使用
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
	cvMoments(maskTemp, &moments, 1);//这里的图像一定需要为二值图像，不能是三通道图
	M00 = cvGetSpatialMoment(&moments, 0, 0);
	M10 = cvGetSpatialMoment(&moments, 1, 0);
	M01 = cvGetSpatialMoment(&moments, 0, 1);
	centers[counter].x = (int)(M10 / M00);//一幅图像中有很多的point，看这里i代表什么意思？
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
1.只要此源文件用到了的头文件，都需要include进来
2.也不要忘了include stdafx.h
3.cvDrawContours()并不会直接显示修改后的图像，而是对之前的原图像进行了修改，此时显示原图像即可。
4.frame_diff()函数使得相邻两帧图像之间进行相减并通过阈值运算，可以获得背景为黑色，目标为白色的帧差图，里面用了两个OpenCV的库函数。
5.注意对于很多图像处理的函数，其原图像和空图像的空间应该一模一样，在更改视频和图像的数据形式的时候，这里出现了较大问题，img_thresh这些应该是单通道的，但是之前却和frame这个通道数保持一致了，所以不对。
6.“引发了异常: 写入访问权限冲突”这个错误在于该语句需要写入的对象没有初始化好，可能是初始化的时候就出现了问题，没有使用合适的函数从而使得无法写入数据进去。
*/