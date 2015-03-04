
//#include "stdafx.h"
#include <math.h>
#include <stdlib.h>

/*
用全选主元Gauss-Jordan法求n阶实矩阵A的逆矩阵A^{-1}
输入参数：
double * a：     原矩阵，为一个方阵
int n：          矩阵维数
输出参数：
double * a：     求得的逆矩阵
返回值：
如果返回标记为0，表示矩阵奇异；否则返回非0值
*/
int brinv( double * a, int n )
{ 
	int * is, * js, i, j, k, l, u, v;
	double d,p;

	is = (int *)malloc( n*sizeof(int) );
	js = (int *)malloc( n*sizeof(int) );
	for ( k = 0; k < n; k++ )
	{ 
		d = 0.0;
		for ( i = k; i < n; i++ )
			for ( j = k; j < n; j++ )
			{ 
				l = i*n+j;
				p = fabs(a[l]);
				if ( p > d ) 
				{ 
					d = p; is[k] = i; js[k] = j;
				}
			}
		if ( d+1.0 == 1.0 ) /* 矩阵为奇异阵 */
		{ 
			free( is ); 
			free( js ); 
			// printf("err**not inv\n");
			return( 0 );
		}
		if ( is[k] != k )
			for ( j = 0; j < n; j++ )
			{ 
				u = k*n+j;
				v = is[k]*n+j;
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
		if ( js[k] != k )
			for ( i = 0; i < n; i++ )
			{ 
				u = i*n+k;
				v = i*n+js[k];
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
			l = k*n+k;
		a[l] = 1.0/a[l];
		for ( j = 0; j < n; j++ )
			if ( j != k )
			{ 
				u = k*n+j;
				a[u] = a[u]*a[l];
			}
		for ( i = 0; i < n; i++ )
			if ( i != k )
				for ( j = 0; j < n; j++ )
					if ( j != k )
					{ 
						u = i*n+j;
						a[u] = a[u] - a[i*n+k]*a[k*n+j];
					}
		for ( i = 0; i < n; i++ )
			if ( i != k )
			{ 
				u = i*n+k;
				a[u] = -a[u]*a[l];
			}
	}
	for ( k = n-1; k >= 0; k-- )
	{ 
		if ( js[k] != k )
			for ( j = 0; j <= n-1; j++ )
			{ 
				u = k*n+j;
				v = js[k]*n+j;
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
		if ( is[k] != k )
			for ( i = 0; i < n; i++ )
			{ 
				u = i*n+k;
				v = i*n+is[k];
				p = a[u]; a[u] = a[v]; a[v] = p;
			}
	}
	free( is );
	free( js );

	return(1);
}


/*
一步Kalman滤波程序
对n维线性动态系统与m维线性观测系统
X_k = A_k,k-1*X_k-1 + W_k-1
Y_k = H_k*X_k + V_k
k = 1,2,...
X_k为n维状态向量，Y_k为m维观测向量。
A_k,k-1(nxn维)为状态转移阵，H_k(nxm维)为观测矩阵
W_k为n维状态噪声向量，一般假设为高斯白噪声，且均值为0，协方差为Q_k
V_k为m维观测噪声向量，一般假设为高斯白噪声，且均值为0，协方差为R_k

Kalman滤波问题就是在已知k个观测向量Y_0,Y_1,...,Y_k和初始状态向量估计X_0
及其估计误差协方差阵P_0，以及Q_k,R_k等情况下，递推估计各个x_k及其噪声
估计协方差阵P_k的问题。具体计算公式如下：
P_k|k-1 = A_k,k-1 * P_k-1 * A_k,k-1^T + Q_k-1
K_k = P_k|k-1 * H_k^T * [H_k * P_k|k-1 * H_k^T + R_k]^{-1}
X_k = A_k,k-1 * X_k-1 + K_k * [Y_k - H_k * A_k,k-1 * X_k-1]
P_k = (I-K_k*H_k)*P_k|k-1
其中：
K_k(nxm维)为Kalman增益矩阵
Q_k(nxn维)为W_k协方差阵
R_k(mxm维)为V_k协方差阵
P_k(nxn维)为估计误差协方差阵

一步Kalman滤波函数参数：
n：     整型变量，动态系统的维数（状态量个数）
m：     整型变量，观测系统的维数（观测量个数）
A：     双精度2维数组，nxn，系统状态转移矩阵
H：     双精度2维数组，mxn，观测矩阵
Q：     双精度2维数组，nxn，模型噪声W_k的协方差矩阵
R：     双精度2维数组，mxm，观测噪声V_k的协方差矩阵
y_k：   双精度2维数组，mx1，观测向量序列
x_k：   双精度2维数组，nx1，状态向量估计量序列。输入x_k-1，返回x_k
P_k：   双精度2维数组，nxn，存放误差协方差阵P_k-1。返回时存放更新的估计误差协
方差阵P_k
输出：  x_k, P_k
函数返回值：
若返回0，说明求增益矩阵过程中求逆失败；若返回非0，表示正常返回

Kalman算法的优化：
如果观测误差协方差Q_k-1和测量误差协方差R_k近似为常数，则
观测误差协方差 P_k|k-1 = A_k,k-1 * P_k-1 * A_k,k-1^T + Q_k-1 近似为常数；
这样，K_k也近似为常数，P的更新P_k = ( I - K_k*H_k ) * P_k|k-1 也近似不变
上面的三个量P_k|k-1, K_k, P_k都可以离线算出！
这个程序没有这么优化，因此更通用一些。
*/

int Kalman( int n, int m, float * A, float * H, float * Q, float * R,
		   float * y_k, float * x_k, float * P_k )
{ 
	float * Ax, * PH, * P, * P_kk_1, temp, * K_k; 
	float * yHAx, * KH, * I;
	double * HPHR;
	int x, y, i;
	int invRval;

	P = new float [n*n];
	P_kk_1 = new float [n*n];
	/* 状态误差协方差的预测 P_k|k-1 */
	for ( y = 0; y < n; y++ )  /* A_k,k-1*P_k-1 */
		for ( x = 0; x < n; x++ )
		{
			temp = 0;
			for ( i = 0; i < n; i++ )
				temp += A[y*n+i]*P_k[i*n+x];
			P[y*n+x] = temp;
		}
	for ( y = 0; y < n; y++ )  /* (A_k,k-1*P_k-1)*A_k,k-1^T+Q_k-1 */
		for ( x = 0; x < n; x++ )
		{
			temp = 0;
			for ( i = 0; i < n; i++ )
				temp += P[y*n+i]*A[x*n+i];
			P_kk_1[y*n+x] = temp + Q[y*n+x];
		}
		/* 求增益矩阵 K_k */
	PH = new float[n*m];
	for ( y = 0; y < n; y++ ) /* P_k|k-1*H_k^T */
		for ( x = 0; x < m; x++ )
		{
			temp = 0;
			for ( i = 0; i < n; i++ )
				temp += P_kk_1[y*n+i]*H[x*m+i];
			PH[y*m+x] = temp;
		}
	HPHR = new double[m*m]; /* 求H_k*P_k|k-1*H_k^T+R_k */
	for ( y = 0; y < m; y++ )
		for ( x = 0; x < m; x++ )
		{
			temp = 0;
			for ( i = 0; i < n; i++ )
				temp += H[y*n+i]*PH[i*m+x];
			HPHR[y*m+x] = temp + R[y*m+x];
		}
	invRval = brinv( HPHR, m ); /* 求逆 */
	if ( invRval == 0 )
	{
		delete [] P;
		delete [] P_kk_1;
		delete [] PH;
		delete [] HPHR;
		return( 0 );
	}
	K_k = new float[n*m]; /* 求K_k */
	for ( y = 0; y < n; y++ )
		for ( x = 0; x < m; x++ )
		{
			temp = 0;
			for ( i = 0; i < m; i++ )
				temp += PH[y*m+i] * (float)HPHR[i*m+x];
			K_k[y*m+x] = temp;
		}
		/* 求状态的估计 x_k */
		Ax = new float[n];
	for ( y = 0; y < n; y++ ) /* A_k,k-1 * x_k-1 */
	{
		temp = 0;
		for ( i = 0; i < n; i++ )
			temp += A[y*n+i]*x_k[i];
		Ax[y] = temp;
	}
	yHAx = new float[m];
	for ( y = 0; y < m; y++ ) /* y_k - H_k*A_k,k-1*x_k-1 */
	{
		temp = 0;
		for ( i = 0; i < n; i++ )
			temp += H[y*n+i] * Ax[i];
		yHAx[y] = y_k[y] - temp;
	}
	for ( y = 0; y < n; y++ )   /* 求x_k */
	{
		temp = 0;
		for ( i = 0; i < m; i++ )
			temp += K_k[y*m+i]*yHAx[i];
		x_k[y] = Ax[y] + temp;
	}
	/* 更新误差的协方差 P_k */
	KH = new float [n*n];
	for ( y = 0; y < n; y++ )
		for ( x = 0; x < n; x++ )
		{
			temp = 0;
			for ( i = 0; i < m; i++ )
				temp += K_k[y*m+i]*H[i*n+x];
			KH[y*n+x] = temp;
		}
	I = new float [n*n];
	for ( y = 0; y < n; y++ )
		for ( x = 0; x < n; x++ )
			I[y*n+x] = (float)(x==y ? 1 : 0);
	for ( y = 0; y < n; y++ )   /* I - K_k*H_k */
		for ( x = 0; x < n; x++ )
			I[y*n+x] = I[y*n+x] - KH[y*n+x];
	for ( y = 0; y < n; y++ )  /* P_k = ( I - K_k*H_k ) * P_k|k-1 */
		for ( x = 0; x < n; x++ )
		{
			temp = 0;
			for ( i = 0; i < n; i++ )
				temp += I[y*n+i]*P_kk_1[i*n+x];
			P_k[y*n+x] = temp;
		}

	delete [] P;
	delete [] P_kk_1;
	delete [] PH;
	delete [] HPHR;
	delete [] K_k;
	delete [] Ax;
	delete [] yHAx;
	delete [] KH;
	delete [] I;

	return( 1 );
}