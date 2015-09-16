#include "kalmanFilter.h"

// ref voiture (X',Y',Z')
// ref de la maison(fixe) (x,y,z)
// angleX( angle entre X' et le plan (x,y) ): gyro[0] accelero[0];
// angleY( angle entre Y' et le plan (x,y) ): gyro[1] accelero[1];
// angleZ( angle entre Z' et le plan (x,y) ): gyro[2] accelero[2];
char* name = "Inertial Station";
KalmanFilter* kalmanFilter1;
KalmanFilter* kalmanFilter2;

unsigned long temps = millis();
double angle[3] = {0.0};
int mesure = 0;

void setup() {
  Serial.begin(115200);
  kalmanFilter1 = new KalmanFilter(2,50.0);
  kalmanFilter2 = new KalmanFilter(2,50.0);

  Spark.function("read", read);
  Spark.variable("name",name,STRING);
}

double pi = 3.1415926535;
double* dataAccelero = NULL;
double* dataGyro = NULL;
double* approx1 = NULL;
double* approx2 = NULL;
double dt = 0;

double teta1 = 0;
double teta2 = 0;

int zero = 0;

//attention un filtre par angle !!!
void loop() {

  if ( mesure ) {
    dataAccelero = kalmanFilter1->readAccelero();
    dataGyro = kalmanFilter1->readGyro();

    // Test
    //printData("Accelero",dataAccelero,3);
    //printData("Gyro",dataGyro,3);

    if ( dataAccelero == NULL ) {
      mesure = 0;
    }
    else if ( dataGyro == NULL ) {
      mesure = 0;
    }
    else {
      dt = (millis() - temps)/1000.0;
      temps = millis();

      kalmanFilter1->setAte(dt);
      kalmanFilter2->setAte(dt);

      teta1 = min(dataAccelero[0],1);
      teta1 = max(teta1,-1);
      teta1 = -180*asin(-1*teta1)/pi;

      teta2 = min(dataAccelero[1],1);
      teta2 = max(teta2,-1);
      teta2 = -180*asin(-1*teta2)/pi;

      approx1 = kalmanFilter1->getApprox( dataGyro[0] , teta1 );
      angle[0] += approx1[0]*dt;

      approx2 = kalmanFilter2->getApprox( dataGyro[1] , teta2 );
      angle[1] += approx2[0]*dt;

      if( fabs(dataAccelero[2] - 1.0) < 2e-2 ) {
        zero++;
        if( zero == 100 ) {
          kalmanFilter1->destructAllocKalman();
          kalmanFilter2->destructAllocKalman();

          kalmanFilter1->initKalman(1.0/dt);
          kalmanFilter2->initKalman(1.0/dt);

          angle[0] = 0;
          angle[1] = 0;
        }
      }
      else {
        zero = 0;
      }

      //approx = kalmanFilter2->getApprox( dataGyro[2] , -180*asin(-1*dataAccelero[2])/pi );
      //angle[2] += approx[0]*dt;

       angle[2] = 0;

       free(dataAccelero);
       dataAccelero = NULL;

       free(dataGyro);
       dataGyro = NULL;
    }

     Serial.print(angle[0]);
     Serial.print(" ");
     Serial.print(angle[1]);
     Serial.print(" ");
     Serial.println(angle[2]);
  }

}

int read(String info) {
  mesure = (mesure + 1)%2;
  temps = millis();

  return 1;
}

void printData(String name,double* data,int N) {
  int i = 0;

  Serial.print(name);
  Serial.print(": ");
  for( i = 0 ; i < N ; i++ ) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println("");

}
