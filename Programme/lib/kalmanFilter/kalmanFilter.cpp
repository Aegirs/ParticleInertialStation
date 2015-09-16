#include "kalmanFilter.h"

KalmanFilter::KalmanFilter(int sensitivity, double fe) {
  // gyroscope
  gyro = new L3G4200D(2000,200);
  //accelero
  accelero = new MMA_7455(sensitivity);

  // init KalmanFilter
  initKalman(fe);
}

void KalmanFilter::setAte(double te) {
  A[1][0] = te;
}

double* KalmanFilter::readAccelero() {

  double* data = (double*)calloc(3,sizeof(double));
  data[0] = accelero->readAxisInG('x');  data[1] = accelero->readAxisInG('y');
  data[2] = accelero->readAxisInG('z');

  if ( isnan(data[0])  || isnan(data[1]) || isnan(data[2]) ) {
    return NULL;
  }

  return data;
}

double* KalmanFilter::readGyro() {
  gyro->read();

  double* data = (double*)calloc(3,sizeof(double));
  data[0] = (int8_t)gyro->g.x;  data[1] = (int8_t)gyro->g.y;  data[2] = (int8_t)gyro->g.z;

  if ( isnan(data[0])  || isnan(data[1]) || isnan(data[2]) ) {
    return NULL;
  }

  return data;
}

double* KalmanFilter::getApprox(double speedAnguGyro,double angleAccelero) {
  Y[0] = speedAnguGyro;
  Y[1] = angleAccelero;

  predictionKalman();
  updateKalman();

  return X;
}

void KalmanFilter::initKalman(double fe) {
  double te = 1.0/fe; // temps entre deux mesures

  Y = (double*)calloc(2,sizeof(double));
  X = (double*)calloc(3,sizeof(double));
  B = (double*)calloc(2,sizeof(double));

  H = (double**)calloc(2,sizeof(double*));
  A = (double**)calloc(3,sizeof(double*));
  R = (double**)calloc(2,sizeof(double*));
  Q = (double**)calloc(3,sizeof(double*));
  P = (double**)calloc(3,sizeof(double*));

  for( int i = 0 ; i < 3 ; i++ ) {
    A[i] = (double*)calloc(3,sizeof(double));
    Q[i] = (double*)calloc(3,sizeof(double));
    P[i] = (double*)calloc(3,sizeof(double));
    if ( i < 2 ) {
      R[i] = (double*)calloc(2,sizeof(double));
      H[i] = (double*)calloc(3,sizeof(double));
    }
  }

  H[0][0] = 1; H[0][2] = 1; H[1][1] = 1;
  A[0][0] = 1; A[1][0] = te; A[1][1] = 1; A[2][2] = 1;
  R[0][0] = 1; R[1][1] = 1;
  Q[0][0] = 2; Q[1][1] = 0; Q[2][2] = 1;
}

void KalmanFilter::destructAllocKalman() {
  freeMatrix(H,2);
  freeMatrix(A,3);
  freeMatrix(R,2);
  freeMatrix(Q,3);
  freeMatrix(P,3);

  free(Y);
  free(X);
  free(B);

  Y = NULL;
  X = NULL;
  B = NULL;
}

void KalmanFilter::getXPlus() {
  double* img = imgMatrix(A,X,3,3);
  free(X);
  X = img;
}

void KalmanFilter::getPPlus() {
  double** prod1 = productMatrix(A,P,3,3,3);
  double** tA = transposeMatrix(A,3,3);
  double** prod2 = productMatrix(prod1,tA,3,3,3);

  freeMatrix(P,3);
  P = addMatrix(prod2,Q,3,3);

  freeMatrix(prod1,3);
  freeMatrix(prod2,3);
  freeMatrix(tA,3);
}

void KalmanFilter::predictionKalman() {
  getXPlus();
  getPPlus();
}

double** KalmanFilter::getK() {
  double** prod1 = productMatrix(H,P,2,3,3);
  double** tH = transposeMatrix(H,2,3);
  double** prod2 = productMatrix(prod1,tH,2,2,3);

  freeMatrix(prod1,2);

  double** tmp = addMatrix(prod2,R,2,2);
  freeMatrix(prod2,2);

  double** inverse = inverseMatrix(tmp,2);
  freeMatrix(tmp,2);

  prod1 = productMatrix(P,tH,3,2,3);
  freeMatrix(tH,3);

  double** K = NULL;

  if( inverse != NULL ) {
    K = productMatrix(prod1,inverse,3,2,2);
    freeMatrix(inverse,2);
  }
  freeMatrix(prod1,3);

  return K;
}

void KalmanFilter::getP(double** K) {
  double** tmp = productMatrix(K,H,3,3,2);
  double** subI = substractionIdMatrix(tmp,3);

  freeMatrix(tmp,3);

  tmp = productMatrix(subI,P,3,3,3);

  freeMatrix(P,3);
  freeMatrix(subI,3);

  P = tmp;
}

void KalmanFilter::getX(double** K) {
  double* img = imgMatrix(H,X,2,3);
  double* subV = subVect(Y,img,2);

  free(img);
  img = imgMatrix(K,subV,3,2);

  free(subV);
  subV = NULL;

  double* tmp = addVect(X,img,3);

  free(img);
  img = NULL;
  free(X);
  X = tmp;

  tmp = NULL;
}

void KalmanFilter::updateKalman() {
  double** K = getK();

  getP(K);
  getX(K);

  freeMatrix(K,3);
}

void KalmanFilter::freeMatrix(double** A,int N) {

  for(int i = 0 ; i < N ; i++ ) {
    free( A[i] );
  }

  free(A);
  A = NULL;
}

double* KalmanFilter::imgMatrix(double** A,double* X,int N,int M) {
  double* res = (double*)calloc(N,sizeof(double));
  for(int i = 0 ; i < N ; i++ ) {
    for(int j = 0; j < M; j++) {
      res[i] += A[i][j]*X[j];
    }
  }

  return res;
}

double** KalmanFilter::addMatrix(double** A,double** B,int N,int M) {
  double** add = (double**)calloc(N,sizeof(double*));

  for(int i = 0 ; i < N ; i++ ) {
    add[i] = (double*)calloc(M,sizeof(double));
    for(int j = 0; j < M ; j++) {
      add[i][j] = A[i][j] + B[i][j];
    }
  }

  return add;
}

double* KalmanFilter::addVect(double* A,double* B,int N) {
  double* add = (double*)calloc(N,sizeof(double));

  for(int i = 0 ; i < N ; i++ ) {
    add[i] = A[i] + B[i];
  }

  return add;
}

double* KalmanFilter::subVect(double* A,double* B,int N) {
  double* sub = (double*)calloc(N,sizeof(double));

  for(int i = 0 ; i < N ; i++ ) {
    sub[i] = A[i] - B[i];
  }

  return sub;
}

double** KalmanFilter::productMatrix(double** A,double** B,int N,int M,int dimComm) {
  double** prod = (double**)calloc(N,sizeof(double*));

  for(int i = 0 ; i < N ; i++ ) {
    prod[i] = (double*)calloc(M,sizeof(double));
    for(int j = 0; j < M; j++) {
      for(int k = 0; k < dimComm; k++ ) {
        prod[i][j] += A[i][k] * B[k][j];
      }
    }
  }

  return prod;
}

double** KalmanFilter::transposeMatrix(double** A,int N,int M) {
  double** trans = (double**)calloc(M,sizeof(double*));

  for(int i = 0 ; i < M ; i++ ) {
    trans[i] = (double*)calloc(N,sizeof(double));
    for(int j = 0; j < N; j++) {
      trans[i][j] = A[j][i];
    }
  }

  return trans;
}

double** KalmanFilter::substractionIdMatrix(double** M, int N) {
  double** res = (double**)calloc(N,sizeof(double*));
  for( int i = 0 ; i < N ; i++ ) {
    res[i] = (double*)calloc(N,sizeof(double));
    for( int j = 0 ; j < N ; j++ ) {
      res[i][j] = 1. * (i == j) - M[i][j];
    }
  }

  return res;
}

// inverse Matrix
void KalmanFilter::commuteLineMatrix(double** M,int C,int i,int j) {
  double tmp = 0.;
  for(int k = 0 ; k < C ; k++ ) {
    tmp = M[i][k];
    M[i][k] = M[j][k];
    M[j][k] = tmp;
  }
}

// Li = coeff1*Li - coeff2*Lj
void KalmanFilter::substractionLineMatrix(double** M,int C,int i,double coeff1,int j,double coeff2) {
  for(int k = 0 ; k < C ; k++ ) {
    M[i][k] = coeff1*M[i][k] - coeff2*M[j][k];
  }
}

void KalmanFilter::multLineMatrix(double** M,int C,int i,double coeff) {
  for(int k = 0 ; k < C ; k++ ) {
    M[i][k] *= coeff;
  }
}

int isEqualTo(double val,double test,double epsilon) {
  return ( fabs(val-test) <= epsilon );
}

double** KalmanFilter::inverseMatrix(double** M,int N) {
  double** res = (double**)calloc(N,sizeof(double*));
  double norm = 0.;
  int j = 0;

  for(int i = 0 ; i < N ; i++ ) {
    res[i] = (double*)calloc(N,sizeof(double));
    res[i][i] = 1.;
  }

  for( int i = 0; i < N ; i++ ) {
    // positionne la ligne avec l'element sur la diag concerné non nulle.
    for(j = i ; j < N ; j++ ) {
      if ( !isEqualTo(M[j][i],0.,1e-6) ) break;
    }

    if ( j == N ) {
      return NULL;
    }

    else {
      if ( i != j ) {
        commuteLineMatrix(M,N,i,j);
        commuteLineMatrix(res,N,i,j);
      }
      norm = 1./M[i][i];
      multLineMatrix(M,N,i,norm);
      multLineMatrix(res,N,i,norm);

      for(j = (i+1) ; j < N ; j++ ) {
        norm = M[j][i];
        substractionLineMatrix(M,N,j,1.,i,norm);
        substractionLineMatrix(res,N,j,1.,i,norm);
      }

    }
  }

  for(int i = N-1 ; i > -1 ; i-- ) {
    for(j = (i-1) ; j > -1 ; j-- ) {
      norm = M[j][i];
      substractionLineMatrix(M,N,j,1.,i,norm);
      substractionLineMatrix(res,N,j,1.,i,norm);
    }
  }

  return res;
}
