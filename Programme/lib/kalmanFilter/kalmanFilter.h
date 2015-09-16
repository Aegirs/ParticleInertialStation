#include "L3G4200D.h"
#include "MMA_7455.h"
#include <math.h>

class KalmanFilter{
  public:
    KalmanFilter(int sensitivity, double fe);
    double* readAccelero();
    double* readGyro();

    double* getApprox(double speedAnguGyro,double angleAccelero);
    void setAte(double te);
    void destructAllocKalman();
    void initKalman(double fe);
  protected:
    double* Y; // vecteur de mesure [2]
    double** H;// matrice d'observation [2][3]
    double* X; // vecteur d'etat [3]
    double* B; // vecteur de bruit[2]

    double** A;// matrice de transition [3][3]
    double** R;// matrice de covariance du bruit (gyroscope et accelero) [2][2]
    double** Q;// matrice de max de covariance du bruit [3][3]
    double** P;// matrice prediction erreur [3][3]

    L3G4200D *gyro;
    MMA_7455 *accelero;

    // kalman op

    void getXPlus();
    void getPPlus();
    void predictionKalman();

    double** getK();
    void getP(double** K);
    void getX(double** K);
    void updateKalman();

    // matrix struct
    void freeMatrix(double** A,int N);
    double* imgMatrix(double** A,double* X,int N,int M);

    double** addMatrix(double** A,double** B,int N,int M);

    double* addVect(double* A,double* B,int N);
    double* subVect(double* A,double* B,int N);

    double** productMatrix(double** A,double** B,int N,int M,int dimComm);
    double** transposeMatrix(double** A,int N,int M);

    double** substractionIdMatrix(double** M, int N);
    //pivot de gauss
    void commuteLineMatrix(double** M,int C,int i,int j);
    void substractionLineMatrix(double** M,int C,int i,double coeff1,int j,double coeff2);
    void multLineMatrix(double** M,int C,int i,double coeff);
    double** inverseMatrix(double** M,int N);
};
