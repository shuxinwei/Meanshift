//
//  main.cpp
//  Meanshift new
//
//  Created by Bran on 14-3-24.
//  Copyright (c) 2014年 Bran. All rights reserved.
//


#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <math.h>
#include <iostream>
#include "MeanShift.h"
#include <fstream>
using namespace std;

#define B(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3]		//B
#define G(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3+1]	//G
#define R(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3+2]	//R
#define S(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)]

#define Num 10
#define ai 0.08
#define SKIP_FRAME_COUNT 10
#define FRAME_RATE 29

bool pause=false;
bool track = false;
bool selectRegion = false;
IplImage *curframe=NULL;

unsigned char * img;
int xin,yin;
int xout,yout;
int Wid,Hei;
int WidIn,HeiIn;
int WidOut,HeiOut;

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
//鼠标函数
void mouse( int, int, int, int, void* );
void IplToImg(IplImage* src, int w,int h);


int main()
{
	float totalDist = 0;
	int FrameNum=0;
	//CvCapture *capture = cvCreateFileCapture("Users/apple/Desktop/Bran.avi");
    
    
	cvNamedWindow("video",1);
    
	uchar key = false;
	float rho_v;
	int distx,disty;
	int count = 0;
    //PP ret;
    
	CvRect* regions;
    string picture;
    char interval;
    string filename="/Users/Bran/Desktop/TestSet/Skating1/";
    ifstream fin(filename+"imagelist.txt");
    ifstream gin(filename+"groundtruth_rect.txt");
    MeanshiftTracker T;
    int x1,x2,w,h;
    //CAMShift func;
    const int LINE_LENGTH=100;
    char str[LINE_LENGTH-1];
	//curframe=cvQueryFrame(capture);
    if (!fin.is_open()||!gin.is_open())
        return -1;
    char buf[100];
    int Buf_i=0;
    IplImage* frame = 0;
    //func.getPosition(446,181,29,26);//ground truth
    /*for(;;){
        Buf_i++;
        sprintf(buf, "/Users/Bran/Desktop/TestSet/Coke/img/%04d.jpg", Buf_i);
        cout<<buf<<endl;
        frame = cvLoadImage(buf);
        cvShowImage( "CamShiftDemo", frame );
        if(!frame)
            break;
        PP ret = func.Tracking(buf);// path
        CvPoint x1 = ret.first;	// rectangle	point
        CvPoint x2 = ret.second;// rectangle	point
        cvRectangle( frame, x1,x2, CV_RGB(73,228,81), 3, CV_AA, 0 );
        cvShowImage( "CamShiftDemo", frame );
        cvWaitKey(1);
    }*/
    //getchar();
	while(fin.getline(str,LINE_LENGTH))
	{
		//curframe=cvQueryFrame(capture); 用视频作为输入
        
        //cout<<str<<endl;
        //getchar();
        gin>>x1>>interval>>x2>>interval>>w>>interval>>h;
        picture=filename+str;
        curframe=cvLoadImage(picture.c_str());
		FrameNum ++;
		if(!curframe)
			break;
 		/*
		if(selectRegion)//鼠标选中区域
		{
			Wid = curframe->width;
			Hei = curframe->height;
			img = new unsigned char [Wid * Hei * 3]; //选中图片大小 RGB三通道
            
            
			get_regions(curframe, &regions);
            
			CvRect r = regions[0];
            //func.getPosition(r.x,r.y,r.width,r.height);//ground truth
            func.getPosition(x1,x2,w,h);
            //ret = func.Tracking(picture.c_str());
			int centerX = r.x+r.width/2;//中心
			int centerY = r.y+r.height/2;
			WidIn = r.width/2;//输入框大小
			HeiIn = r.height/2;
			
			xin = centerX;
			yin = centerY;
            cvRectangle(curframe,cvPoint(r.x-r.width/2, r.y-r.height/2),cvPoint(r.x+r.width/2,r.y+r.height/2),cvScalar(255,0,0),2,8,0);
			IplToImg(curframe,Wid,Hei);
			
			//T.Initial_MeanShift_tracker(centerX,centerY,WidIn,HeiIn,img,Wid,Hei,1.0/FRAME_RATE);
			track = true;
			selectRegion = false;
			totalDist = 0;
            //cvShowImage("video",curframe);
            //while(1);
            continue;
		}
         */
        //getchar();

        if (!track) {
            Wid = curframe->width;
            Hei = curframe->height;
            img = new unsigned char [Wid * Hei * 3]; //选中图片大小 RGB三通道
            IplToImg(curframe,Wid,Hei);
            cout<<x1<<x2<<w<<h;
            WidIn = w/2;//输入框大小
            HeiIn = h/2;
            int centerX = x1+w/2;//中心
            int centerY = x2+h/2;
            xin = centerX;
            yin = centerY;

            T.Initial_MeanShift_tracker(centerX,centerY,WidIn,HeiIn,img,Wid,Hei,1.0/FRAME_RATE);
            cvRectangle(curframe,cvPoint(xin - WidIn, yin - HeiIn),cvPoint(xin + WidIn,yin + HeiIn),cvScalar(255,0,0),2,8,0);
            //func.getPosition(x1,x2,w,h);
            track=true;
            cvShowImage("video",curframe);
        }
        IplToImg(curframe,Wid,Hei);
		if(track)
		{
            cout<<picture.c_str()<<endl;
            //ret = func.Tracking(picture.c_str());
            //cout<<ret.first.x<<endl;
			rho_v = T.MeanShift_tracker( xin, yin, WidIn, HeiIn, img, Wid, Hei, xout, yout, WidOut, HeiOut );
			
			cvRectangle(curframe,cvPoint(xout - WidOut, yout - HeiOut),cvPoint(xout+WidOut,yout+HeiOut),cvScalar(255,0,0),2,8,0);
            cvRectangle(curframe, CvPoint(x1,x2), CvPoint(x1+w,x2+h), CV_RGB(73,228,81), 3, CV_AA, 0);
			xin = xout; yin = yout;
			WidIn = WidOut; HeiIn = HeiOut;
            
			/*if ( rho_v < 0.8 )
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
			}*/
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
    
	//T.Clear_MeanShift_tracker();
    
	//getchar();
}

//int Wid,Hei;
//三通道整合
void IplToImg(IplImage* src, int w,int h)
{
	int i,j;
	for ( j = 0; j < h; j++ )
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
    /*refresh the struct p*/
    
	/* extract regions defined by user; store as an array of rectangles */
	if( p.n == 0 )
	{
		*regions = NULL;
		return 0;
	}
	r = (CvRect *)malloc( p.n * sizeof( CvRect ) );
	for( i = 0; i < p.n; i++ ) //for each rectangle round the object
	{
		x1 = MIN( p.loc1[i].x, p.loc2[i].x ); //determine the query rectangle
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
	static int pressed = 0;
    
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
		pressed = 1;
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
		pressed = 0;
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
