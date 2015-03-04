#include <math.h>
#include <stdio.h>
#include "kalman.h"
#include "MeanShift.h"


# define R_BIN      8  //each bin 8 bin for each color
# define G_BIN      8
# define B_BIN      8

# define R_SHIFT    5  // log2(256/8)
# define G_SHIFT    5
# define B_SHIFT    5


void MeanshiftTracker::CalcuColorHistogram( int x0, int y0, int Wx, int Hy,
						 unsigned char * image, int W, int H,
						 float * Kernel, float C_k,
						 float * ColorHist, int bins )
{
    int x_begin, y_begin;
	int y_end, x_end;
	int x, y, i, index, Wk;
	int r, g, b;

	Wk = 2 * Wx + 1;
	for ( i = 0; i < bins; i++ )
		ColorHist[i] = 0.0;
	if ( ( x0 < 0 ) || (x0 >= W) || ( y0 < 0 ) || ( y0 >= H ) 
		|| ( Wx <= 0 ) || ( Hy <= 0 ) ) return;

	x_begin = x0 - Wx;
	y_begin = y0 - Hy;
	if ( x_begin < 0 ) x_begin = 0;
	if ( y_begin < 0 ) y_begin = 0;
	x_end = x0 + Wx;
	y_end = y0 + Hy;
	if ( x_end >= W ) x_end = W-1;
	if ( y_end >= H ) y_end = H-1;

	for ( y = y_begin; y <= y_end; y++ )
	{
		for ( x = x_begin; x <= x_end; x++ )
		{
			r = image[(y*W+x)*3] >> R_SHIFT; //locate the R_bin
			g = image[(y*W+x)*3+1] >> G_SHIFT; 
			b = image[(y*W+x)*3+2] >> B_SHIFT;
			index = r * G_BIN * B_BIN + g * B_BIN + b;
			ColorHist[index] += Kernel[(y-y_begin)*Wk+(x-x_begin)];
		}
	}

	for ( i = 0; i < bins; i++ )
		ColorHist[i] = ColorHist[i]/C_k;

	return;
}

int MeanshiftTracker::CalcuEpanechnikovKernel( int Wx, int Hy, float * Kernel, float & C_k )
{
	int x, y, xP, yP, i, PixelNo;
	int a2;
	float r2, k;

	xP = Wx * 2 + 1;
	yP = Hy * 2 + 1;
	PixelNo = xP * yP;
	if ( PixelNo <= 0 ) return( 0 );
	for ( i = 0; i < PixelNo; i++ )
		Kernel[i] = 0.0;
	a2 = Wx*Wx+Hy*Hy;
	C_k = 0.0;
	for ( y = 0; y < yP; y++ )
	{
		for ( x = 0; x < xP; x++ )
		{
			r2 = (float)(((y-Hy)*(y-Hy)+(x-Wx)*(x-Wx))*1.0/a2);
			k = 1 - r2;
			C_k = C_k + k;
			Kernel[y*xP+x] = k;
		}
	}

	if ( C_k < 0.0000001 ) return( 0 );

	return( PixelNo );
}

void MeanshiftTracker::CalcuModelHist( int xt, int yt, int Wx, int Hy,
					unsigned char * image, int W, int H,
					float * ModelHist, int bins )
{
	float * Kernel, C_k;
	int xp, yp, PixelNo;

	xp = 2 * Wx + 1;
	yp = 2 * Hy + 1;
	PixelNo = xp * yp;
	Kernel = new float [PixelNo];

	PixelNo = CalcuEpanechnikovKernel( Wx, Hy, Kernel, C_k );

	CalcuColorHistogram( xt, yt, Wx, Hy, image, W, H, Kernel, C_k, ModelHist, bins );

	delete [] Kernel;

	return;
}


void MeanshiftTracker::Initial_MeanShift_tracker( int x0, int y0, int Wx, int Hy,
							   unsigned char * image, int W, int H,
							   float DeltaT )								
{
	int x, y;

	bins = R_BIN * G_BIN * B_BIN;
	Model_Hist = new float[bins];

	CalcuModelHist( x0, y0, Wx, Hy, image, W, H, Model_Hist, bins ); //Calculate target mode
	Qk = new float[4*4];
    
    //Initialize the Kalman Filter
	for ( y = 0; y < 4; y++ )
		for ( x = 0; x < 4; x++ )
			Qk[y*4+x] = 0.0;
	for ( y = 0; y < 4; y++ ) Qk[y*4+y] = 1.0;
	Rk = new float[2*2];
	for ( y = 0; y < 2; y++ )
		for ( x = 0; x < 2; x++ )
			Rk[y*2+x] = 0.0;
	for ( y = 0; y < 2; y++ ) Rk[y*2+y] = 1.0;
	Pk = new float[4*4];
	for ( y = 0; y < 4; y++ )
		for ( x = 0; x < 4; x++ )
			Pk[y*4+x] = 0.0;
	for ( y = 0; y < 4; y++ ) Pk[y*4+y] = 10.0;

	Am = new float[4*4];
	for ( y = 0; y < 4; y++ )
		for ( x = 0; x < 4; x++ )
			Am[y*4+x] = 0.0;
	for ( y = 0; y < 4; y++ ) Am[y*4+y] = 1.0;
	Am[0*4+2] = DeltaT;  	
	Am[1*4+3] = DeltaT;

	Hm = new float[2*4];
	for ( y = 0; y < 2; y++ )
		for ( x = 0; x < 4; x++ )
			Hm[y*4+x] = 0.0;
	Hm[0*4+0] = 1.0; Hm[1*4+1] = 1.0;

	yk = new float[2];
	yk[0] = (float)x0;
	yk[1] = (float)y0;

	xk = new float[4];
	xk[0] = (float)x0;
	xk[1] = (float)y0;
	xk[2] = 0.0;
	xk[3] = 0.0;

	deltat = DeltaT;

	return;
}

//Calculate Bhattacaryya相似度
float MeanshiftTracker::CalcuBhattacharyya( float * p, float * q, int bins )
{
	int i;
	float rho;

	rho = 0.0;
	for ( i = 0; i < bins; i++ )
		rho = (float)(rho + sqrt( p[i]*q[i] ));

	return( rho );
}

const int MAX_ITERATE_TIMES=20;

int MeanshiftTracker::Mean_shift_iteration( int xi, int yi, int Wx, int Hy,
						 unsigned char * image, int W, int H,
						 float * ModelHist, int bins,
						 int & xo, int & yo, float & rho )
{
	int x, y, i, p_idx;
	float * Kernel, * ColorHist, C_k;
	float * w_i, * index_weight, sum_wi;
	int xp, yp, PixelNo;
	int x_begin, x_end, y_begin, y_end;
	int r, g, b, indx;
	float xo_f, yo_f, rho1, err;
	int flag;

	xp = 2 * Wx + 1;
	yp = 2 * Hy + 1;
	PixelNo = xp * yp;
	Kernel = new float [PixelNo]; //Kernel result at each pixel of Box
	w_i = new float [PixelNo];    //Weight Function
	index_weight = new float [bins]; //index
	ColorHist = new float[ bins ]; //color Mode

	//Precalculate Kernel Function
	PixelNo = CalcuEpanechnikovKernel( Wx, Hy, Kernel, C_k );
	//
	flag = 1;
	for ( i = 0; i < MAX_ITERATE_TIMES; i++ )
	{
		if ( flag == 1 )
		{
			CalcuColorHistogram( xi, yi, Wx, Hy, image, W, H, Kernel, C_k, ColorHist, bins );
			//Bhattacharyya similar function
			rho = CalcuBhattacharyya( ColorHist, ModelHist, bins );
		}
		else
		{
			rho = rho1;
		}
		for ( x = 0; x < PixelNo; x++ )
			w_i[x] = 0.0;
		for ( x = 0; x < bins; x++ )
		{
			index_weight[x] = (float) (ColorHist[x] > 0.0000001 ? sqrt(ModelHist[x]/ColorHist[x]) : 0.0 );
            //Calculate similarity of each bin
		}
        x_begin = xi - Wx;
		y_begin = yi - Hy;
		x_begin = x_begin < 0 ? 0 : x_begin;
		y_begin = y_begin < 0 ? 0 : y_begin;
		x_end = xi + Wx;
		y_end = yi + Hy;
		x_end = x_end >= W ? x_end = W-1 : x_end;
		y_end = y_end >= H ? y_end = H-1 : y_end;
		sum_wi = 0.0;
		xo_f = 0.0; yo_f = 0.0;
		for ( y = y_begin; y <= y_end; y++ )
		{	
			for ( x = x_begin; x <= x_end; x++ )
			{
				r = image[(y*W+x)*3] >> R_SHIFT;
				g = image[(y*W+x)*3+1] >> G_SHIFT; 
				b = image[(y*W+x)*3+2] >> B_SHIFT;
				indx = r * G_BIN * B_BIN + g * B_BIN + b;
				p_idx = (y-y_begin)*xp+(x-x_begin);
				w_i[ p_idx ] = index_weight[indx];
				sum_wi += index_weight[indx];
				xo_f += x * w_i[ p_idx ];
				yo_f += y * w_i[ p_idx ];	
			}
		}

		xo = (int)(xo_f / sum_wi + 0.5 );
		yo = (int)(yo_f / sum_wi + 0.5 );

		CalcuColorHistogram( xo, yo, Wx, Hy, image, W, H, Kernel, C_k, ColorHist, bins );

		rho1 = CalcuBhattacharyya( ColorHist, ModelHist, bins );

		if ( rho1 < rho )
		{
			xo = (xo+xi)/2; 
			yo = (yo+yi)/2;
			flag = 1;
		}
		else 
			flag = 0;

		err = (float)(fabs(xo-xi)+fabs(yo-yi));
		if ( err <= 1.0 ) //end situation
		{
			rho = rho1;
			break;
		}
		else
		{
			xi = xo; yi = yo;
		}		
	}

	delete [] Kernel;
	delete [] w_i;
	delete [] index_weight;
	delete [] ColorHist;

	return( i+1 );
}


# define Delta_h     0.1
# define GAMMA       0.5

float MeanshiftTracker::MeanShift_tracker( int xin, int yin, int Win, int Hin,
						unsigned char * image, int W, int H,
						int & xout, int & yout, int & Wout, int & Hout )
{
	int rv;
	int xin1, yin1, Win1, Hin1;
	float rho1, rho3, rho;
	int W1, W3, H1, H3, xo1, yo1, xo3, yo3;


	xin1 = (int)( xin + xk[2] * deltat + 0.5 );
	yin1 = (int)( yin + xk[3] * deltat + 0.5 );
    //xin1=xin;
    //yin1=yin;
	if ( xin1 < 0 ) xin1 = 0;
	if ( xin1 >= W ) xin1 = W-1;
	if ( yin1 < 0 ) yin1 = 0;
	if ( yin1 >= H ) yin1 = H-1;


	Win1 = Win;
	Hin1 = Hin;
	if ( Win1 <= 0 ) Win1 = 1;
	if ( Hin1 <= 0 ) Hin1 = 1;
	if ( Win1 >= W/3 ) Win1 = W/3;
	if ( Hin1 >= H/3 ) Hin1 = H/3;


	W1 = (int)( Win1*(1.0 - Delta_h) + 0.5 );
	H1 = (int)( Hin1*(1.0 - Delta_h) + 0.5 );
	rv = Mean_shift_iteration( xin1, yin1, W1, H1, image, W, H, Model_Hist, 
		bins, xo1, yo1, rho1 );	//First location

	Wout = Win1;
	Hout = Hin1; 
	rv = Mean_shift_iteration( xin1, yin1, Wout, Hout, image, W, H, Model_Hist, 
		bins, xout, yout, rho ); //Second location

	W3 = (int)( Win1*(1.0 + Delta_h) + 0.5 ); //Third location
	H3 = (int)( Hin1*(1.0 + Delta_h) + 0.5 );
	rv = Mean_shift_iteration( xin1, yin1, W3, H3, image, W, H, Model_Hist, 
		bins, xo3, yo3, rho3 );

	if ( rho < rho1 )
	{
		rho = rho1;
		xout = xo1; yout = yo1;
		Wout = W1; Hout = H1;
	}
	if ( rho < rho3 )
	{
		rho = rho3;
		xout = xo3; yout = yo3;
		Wout = W3; Hout = H3;
	}

	Wout = (int)( GAMMA * Wout + (1 - GAMMA) * Win + 0.5 ); 
	Hout = (int)( GAMMA * Hout + (1 - GAMMA) * Hin + 0.5 );

    //Predict next center
	yk[0] = (float)xout; yk[1] = (float)yout;
	rv = Kalman( 4, 2, Am, Hm, Qk, Rk, yk, xk, Pk );

	return( rho );
}

void MeanshiftTracker::Clear_MeanShift_tracker()
{
	if ( Model_Hist!=NULL )
		delete [] Model_Hist;
	delete [] Qk;
	delete [] Rk;
	delete [] Pk;
	delete [] Am;
	delete [] Hm;
	delete [] yk;
	delete [] xk;
}