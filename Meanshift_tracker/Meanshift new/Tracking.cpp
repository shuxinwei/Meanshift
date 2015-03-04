// Tracking.cpp : 定义控制台应用程序的入口点。
//

/************************************************************************/
/*参考文献
基于彩色直方图和Kalman滤波的视频跟踪算法
Version 0.9
Written by Y. B. Mao
Visual Information Processing and Analysis Group (ViPAG), 
Nanjing University of Sci. & Tech.
www.open-image.org
Feb. 9, 2006
All rights reserved.

Kalman滤波算法的详细描述，参见：
[1] 徐士良. C常用算法程序集. 清华大学出版社. 1994.

Mean shift跟踪算法的详细描述，请参见：
[1] D. Comaniciu, V. Ramesh, P. Meer. Real-time tracking of non-rigid
objects using mean shift. Proc. Conf. Vision Pattern Rec., II: 142-149, 
Hilton Head, SC, June 2000.
[2] Dorin Comaniciu, Visvanathan Ramesh, Peter Meer. Kernel-based object
tracking. IEEE Trans. on Pattern Analysis and Machine Intelligence. 
Vol.25, No. 5, 2003, pp. 554-577.
[3] Huimin QIAN, Yaobin MAO, Jason GENG, Zhiquan WANG. Object tracking with 
self-updating tracking window. PAISI'2007.

[4]real-time Multiple Objects Tracking with Occlusion Handling in Dynamic Scenes 

本代码只可用于非商业用途。如使用本代码请在论文中标注上述三篇参考文献。*/
/************************************************************************/

#include "stdafx.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <math.h>
#include <iostream>
#include "MeanShift.h"

using namespace std;

#define B(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3]		//B
#define G(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3+1]	//G
#define R(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3+2]	//R
#define S(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)]	

#define Num 10  //帧差的间隔
#define ai 0.08 //学习率
#define SKIP_FRAME_COUNT 10
#define FRAME_RATE 29

bool pause=false;//是否暂停
bool track = false;//是否跟踪
bool selectRegion = false;
IplImage *curframe=NULL; 

unsigned char * img;//把iplimg改到char*  便于计算
int xin,yin;//跟踪时输入的中心点
int xout,yout;//跟踪时得到的输出中心点
int Wid,Hei;//图像的大小
int WidIn,HeiIn;//输入的半宽与半高
int WidOut,HeiOut;//输出的半宽与半高

#define MAX_OBJECTS 5
typedef struct params {  
	CvPoint loc1[MAX_OBJECTS];  
	CvPoint loc2[MAX_OBJECTS];  
	IplImage* objects[MAX_OBJECTS];  
	char* win_name;  
	IplImage* orig_img;  
	IplImage* cur_img;  
	int n;  
} params;  


int get_regions( IplImage*, CvRect** );  
void mouse( int, int, int, int, void* ); 
void IplToImg(IplImage* src, int w,int h);


void main()
{
	float totalDist = 0;
	int FrameNum=0;  //帧号

	//CvCapture *capture = cvCreateFileCapture("input/head.avi");	
	//CvCapture *capture = cvCreateFileCapture("input/garden.avi");
	CvCapture *capture = cvCreateFileCapture("input/west1near.avi");
	//CvCapture *capture = cvCreateFileCapture("input/west4many.avi");


	cvNamedWindow("video",1);

	uchar key = false;      //用来设置暂停
	float rho_v;//表示相似度
	int distx,disty;//用来记录输入与输出目标的距离
	int count = 0; //用来记录输入与输出目标跟踪相近的次数，当次数大于某个阈值时，说明目标跟踪丢失。

	CvRect* regions;	

	//curframe=cvQueryFrame(capture); //抓取一帧
	while(capture)
	{		
		curframe=cvQueryFrame(capture); //抓取一帧
		FrameNum ++;
		if(!curframe)
			break;
 		
		if(selectRegion)//第一帧时初始化过程
		{ 
			Wid = curframe->width;
			Hei = curframe->height; 
			img = new unsigned char [Wid * Hei * 3];

			get_regions(curframe, &regions);

			CvRect r = regions[0];			

			int centerX = r.x+r.width/2;//得到目标中心点
			int centerY = r.y+r.height/2;
			WidIn = r.width/2;//得到目标半宽与半高
			HeiIn = r.height/2;
			
			xin = centerX;
			yin = centerY;

			IplToImg(curframe,Wid,Hei);
			
			Initial_MeanShift_tracker(centerX,centerY,WidIn,HeiIn,img,Wid,Hei,1.0/FRAME_RATE);  //初始化跟踪变量
			
			track = true;
			selectRegion = false;
			totalDist = 0;

			continue;
		}
					
		IplToImg(curframe,Wid,Hei);		

		if(track)
		{
			/* 跟踪一帧 */
			rho_v = MeanShift_tracker( xin, yin, WidIn, HeiIn, img, Wid, Hei, xout, yout, WidOut, HeiOut );
			
			/* 画框: 新位置为蓝框 */
			cvRectangle(curframe,cvPoint(xout - WidOut, yout - HeiOut),cvPoint(xout+WidOut,yout+HeiOut),cvScalar(255,0,0),2,8,0);
			xin = xout; yin = yout;
			WidIn = WidOut; HeiIn = HeiOut;

			if ( rho_v < 0.8 )  /* 判断是否目标丢失 */
			{				
				if(count>20)
				{
					//MessageBox(TEXT("target loss, please relocate target"));
					xin = 0; yin = 0;
					xout =0; yout = 0;
					WidIn = 0; HeiIn = 0;
					WidOut = 0;HeiOut = 0;
					distx = 10; disty = 10;
					track = false;
				}
				else
				{
					distx=abs(xin-xout);
					disty = abs(yin-yout);
					if(distx<1&&disty<1)
					{
						count = count+1;
					}
					xin = xout; yin = yout;
					WidIn = WidOut; HeiIn = HeiOut;
				}				
			}
		}

		//pause = true;
		if(track)
		{
			cout<<FrameNum<<":\t"<<rho_v<<endl;
			totalDist += rho_v;
		}
		else
		{
			cvWaitKey(10);
		}
		
 		cvShowImage("video",curframe);

		key = cvWaitKey(10);
		if(key == 27)//ESC
		{
			break;
		}
 		else if(key == 'p')
		{
			pause = true;
		}
		else if(key == 't')
		{
			selectRegion = true;	
			pause = false;
		}

		while(pause)
		{
			key = cvWaitKey(10);
			if(key == 'p')
			{
				pause = false;	
			}
			else if(key == 't')
			{
				selectRegion = true;	
				pause = false;
			}
		}
		
	}

	//cout<<"average dist:"<<totalDist/(FrameNum-SKIP_FRAME_COUNT);

	cvReleaseImage(&curframe);
	cvDestroyAllWindows();

	Clear_MeanShift_tracker();

	getchar();
}

//int Wid,Hei;
void IplToImg(IplImage* src, int w,int h)
{
	int i,j;
	for ( j = 0; j < h; j++ ) // 转成正向图像
		for ( i = 0; i < w; i++ )
		{
			img[ ( j*w+i )*3 ] = R(src,i,j);
			img[ ( j*w+i )*3+1 ] = G(src,i,j);
			img[ ( j*w+i )*3+2 ] = B(src,i,j);
		}
}

int get_regions( IplImage* frame, CvRect** regions )  
{  
	char* win_name = "Select Region";  
	params p;  
	CvRect* r;  
	int i, x1, y1, x2, y2, w, h;  

	/* use mouse callback to allow user to define object regions */  
	p.win_name = win_name;  
	p.orig_img = (IplImage *)cvClone( frame );  
	p.cur_img = NULL;  
	p.n = 0;  
	cvNamedWindow( win_name, 1 );  
	cvShowImage( win_name, frame );  
	cvSetMouseCallback( win_name, &mouse, &p );  
	while(cvWaitKey(0) != '\r');  
	cvDestroyWindow( win_name );  
	cvReleaseImage( &(p.orig_img) );  
	if( p.cur_img )  
		cvReleaseImage( &(p.cur_img) );  

	/* extract regions defined by user; store as an array of rectangles */  
	if( p.n == 0 )  
	{  
		*regions = NULL;  
		return 0;  
	}  
	r = (CvRect *)malloc( p.n * sizeof( CvRect ) );  
	for( i = 0; i < p.n; i++ ) //for each rectangle round the object   
	{  
		x1 = MIN( p.loc1[i].x, p.loc2[i].x );  
		x2 = MAX( p.loc1[i].x, p.loc2[i].x );  
		y1 = MIN( p.loc1[i].y, p.loc2[i].y );  
		y2 = MAX( p.loc1[i].y, p.loc2[i].y );  
		w = x2 - x1;  
		h = y2 - y1;  

		/* ensure odd width and height */  
		w = ( w % 2 )? w : w+1;  
		h = ( h % 2 )? h : h+1;  
		r[i] = cvRect( x1, y1, w, h );    //define one of the rects   
	}  
	*regions = r;  
	return p.n;  
}

/* 
Mouse callback function that allows user to specify the initial object 
regions.  Parameters are as specified in OpenCV documentation. 
*/  
void mouse( int event, int x, int y, int flags, void* param )  
{  
	params* p = (params*)param;  
	CvPoint* loc;  
	int n;  
	IplImage* tmp;  
	static int pressed = FALSE;  

	int height=p->orig_img->height;  

	/* on left button press, remember first corner of rectangle around object */  
	if( event == CV_EVENT_LBUTTONDOWN )  
	{  
		n = p->n;  
		if( n == MAX_OBJECTS )  
			return;  
		loc = p->loc1;  
		loc[n].x = x;  
		loc[n].y = y;  
		pressed = TRUE;  
	}  

	/* on left button up, finalize the rectangle and draw it in black */  
	else if( event == CV_EVENT_LBUTTONUP )  
	{  
		n = p->n;  
		if( n == MAX_OBJECTS )  
			return;  
		loc = p->loc2;  
		loc[n].x = x;  
		loc[n].y = y;  
		cvReleaseImage( &(p->cur_img) );  
		p->cur_img = NULL;  
		cvRectangle( p->orig_img, p->loc1[n], loc[n], CV_RGB(255,0,0), 1, 8, 0 );  
		cvShowImage( p->win_name, p->orig_img );  
		pressed = FALSE;  
		p->n++;  
	}  

	/* on mouse move with left button down, draw rectangle as defined in white */  
	else if( event == CV_EVENT_MOUSEMOVE  &&  flags & CV_EVENT_FLAG_LBUTTON )  
	{  
		n = p->n;  
		if( n == MAX_OBJECTS )  
			return;  
		tmp = (IplImage *)cvClone( p->orig_img );  
		loc = p->loc1;  
		cvRectangle( tmp, loc[n], cvPoint(x, y), CV_RGB(255,255,255), 1, 8, 0 );  
		cvShowImage( p->win_name, tmp );  
		if( p->cur_img )  
			cvReleaseImage( &(p->cur_img) );  
		p->cur_img = tmp;  
	}  
} 
