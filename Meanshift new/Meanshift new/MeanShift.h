//meanshift.h

class MeanshiftTracker{
private:
    float * Model_Hist;
    int bins;
    float * Qk;   // ״̬??Э????(ģ??????)
    float * Rk;   // ?۲???Э????(?۲?????)
    float * Pk;   // ״̬????????Э????
    float * Am;   // ״̬ת????
    float * Hm;   // ?۲?????
    float * yk;   // ?۲???
    float * xk;   // ״̬??
    float deltat; // ??Ƶ????ʱ????????֡?ʵĵ???
public:
    void Initial_MeanShift_tracker( int x0, int y0, int Wx, int Hy,
                                   unsigned char * image, int W, int H,
                                   float DeltaT );
    
    float MeanShift_tracker( int xin, int yin, int Win, int Hin,
                            unsigned char * image, int W, int H,
                            int & xout, int & yout, int & Wout, int & Hout );
    
    void Clear_MeanShift_tracker();
    void CalcuColorHistogram( int x0, int y0, int Wx, int Hy,
                             unsigned char * image, int W, int H,
                             float * Kernel, float C_k,
                             float * ColorHist, int bins );
    int CalcuEpanechnikovKernel( int Wx, int Hy, float * Kernel, float & C_k );
    void CalcuModelHist( int xt, int yt, int Wx, int Hy,
                        unsigned char * image, int W, int H,
                        float * ModelHist, int bins );
    int Mean_shift_iteration( int xi, int yi, int Wx, int Hy,
                             unsigned char * image, int W, int H,
                             float * ModelHist, int bins,
                             int & xo, int & yo, float & rho );
    float CalcuBhattacharyya( float * p, float * q, int bins );
};

