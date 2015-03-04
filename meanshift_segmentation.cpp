//
//  main.cpp
//  meanshift_segmentation
//
//  Created by Bran on 12/22/14.
//  Copyright (c) 2014 Bran. All rights reserved.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <vector>

using namespace cv;
using namespace std;

string winName = "meanshift";
Mat img, res;
static void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
            }
        }
    }
}
class MeanShift
{
public:
    Mat src;    //源图片
    Mat src_Luv; //Luv格式
    vector<Mat> split_Luv;  //分离通道
    vector<Mat> merge_Luv;  //meanshift后的通道
    constexpr static const double h_s=10.0;
    constexpr static const double h_r=6.5;
    double C_kd;
    double sum_G; //公式中的g(x)求和
    double sum_Gx[5]; //0,1表示x,y 2，3，4表示LUV空间坐标
    double deltaf_d[5]; //f的导数
    int x,y;
    float L,U,V; //中心点
    double m_hg[5];
    int x_i,y_i;
    float L_i,U_i,V_i; //区域内的其他点
    
public:
    MeanShift(const Mat& src);
    ~MeanShift();
    double calculateF();//计算密度函数F
    void calculateG(int row,int col);//计算G
    void MS_Training(); //更新x的位置
        double **G;

};
MeanShift::MeanShift(const Mat& src){
    this->src=src;
    cvtColor(src,src_Luv,COLOR_BGR2Luv);
    //cvtColor(src,dst_Luv,COLOR_BGR2Luv);
    G=new double*[src.rows];
    split(src_Luv, split_Luv); //将Luv三个通道分离出来
    split(src_Luv,merge_Luv);
    for (int i=0;i<src.rows;i++) {
        G[i]=new double[src.cols];
        //for(int j=0;j<src.cols;j++)
        //    G[i][j]=0;
    }
    C_kd=0;
}

double MeanShift::calculateF(){
    int i;
    double ans=0;
    for(i=0;i<5;i++){
        deltaf_d[i]=0;
        deltaf_d[i]=sum_G*m_hg[i];
        ans+=pow(deltaf_d[i],2);
        //cout<<"delta "<<i<<":"<<deltaf_d[i]<<endl;
    }
    return ans;
}
void MeanShift::calculateG(int row,int col){
    int i,j,k;
    double norm_r,norm_s;
    sum_G=0;
    for(k=0;k<5;k++)
        sum_Gx[k]=0;
    for (i=row-h_s; i<row+h_s; i++) { //in radius Hs
        for(j=col-h_s;j<col+h_s;j++){
            x_i=i;
            y_i=j;
            L_i=(float)split_Luv[0].at<uchar>(i,j);
            U_i=(float)split_Luv[1].at<uchar>(i,j);
            V_i=(float)split_Luv[2].at<uchar>(i,j);
            
            L_i = L_i*100.0/255;
            U_i = U_i-128;
            V_i = V_i-128;
            
            double dL = L_i - L;
            double dU = U_i - U;
            double dV = V_i - V;
            if (dL*dL+dU*dU+dV*dV <= h_r) { //in radius Hr
                norm_s=(double)pow((x-x_i)/h_s,2)+pow((y-y_i)/h_s,2);
                //cout<<split_Luv[0].at<double>(i,j)<<endl;
                norm_r=(double)pow((L-L_i)/h_r,2)+pow((U_i-U_i)/h_r,2)+pow((V-V_i)/h_r,2);
                //cout<<norm_r<<" "<<norm_s<<endl;
                G[i][j]=(double)1/4*exp(-1.0/2*norm_r-1.0/2*norm_s);
                //cout<<G[i][j]<<endl;
                sum_G+=G[i][j];
                sum_Gx[0]+=G[i][j]*x_i;
                sum_Gx[1]+=G[i][j]*y_i;
                sum_Gx[2]+=G[i][j]*L_i;
                sum_Gx[3]+=G[i][j]*U_i;
                sum_Gx[4]+=G[i][j]*V_i;
            }
        }
    }
}
void MeanShift::MS_Training(){
    int i,j;
    int q;
    for(i=h_s;i<src.rows-h_s;i++)
        for(j=h_s;j<src.cols-h_s;j++){
            x=i;
            y=j;
            L=(float)split_Luv[0].at<uchar>(i,j);
            U=(float)split_Luv[1].at<uchar>(i,j);
            V=(float)split_Luv[2].at<uchar>(i,j);
            //cout<<"Before Train"<<U<<"V"<<V<<endl;
            L = L*100.0/255;
            U = U-128;
            V = V-128;
            for(q=0;q<10;q++){ //5次基本能保证收敛 可以省去F的计算
                calculateG(i, j);
                if (sum_G==0) break;
                //m_hg[k]=sum_Gx[k]/sum_G-x[k];
                x=sum_Gx[0]/sum_G;
                y=sum_Gx[1]/sum_G;
                L=sum_Gx[2]/sum_G;
                U=sum_Gx[3]/sum_G;
                V=sum_Gx[4]/sum_G;
                //double ans=calculateF();
            }
            cout<<"iterations:"<<q<<endl;
            cout<<"row"<<i<<"col"<<j<<endl;
            L = (uchar)L*255.0/100;
            U = U+128;
            V = V+128;
            merge_Luv[0].at<uchar>(i,j)=(uchar)L;
            merge_Luv[1].at<uchar>(i,j)=(uchar)U;
            merge_Luv[2].at<uchar>(i,j)=(uchar)V;
    
    }
    //m_hg=sum_Gx[i]/
}
MeanShift::~MeanShift(){
    for(int i=0;i<src.rows;i++)
        delete []G[i];
    delete []G;
    src.release();
    src_Luv.release();
}
int main(int argc, char** argv)
{
    Mat ans;
    img = imread("/Users/Bran/Desktop/demo3.jpg");
    if( img.empty() )
        return -1;
    MeanShift MS=MeanShift(img);
    MS.MS_Training();
    merge(MS.merge_Luv,ans);
    cvtColor(ans, ans, COLOR_Luv2LBGR);
    imshow("Filter Result", ans);
    floodFillPostprocess( ans, Scalar::all(5) );
    imshow(winName, ans);
    waitKey();
    return 0;
}