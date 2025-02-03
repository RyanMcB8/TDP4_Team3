#include "ICM20948.h"

ICM20948 IMU(Wire, 0x69); // an ICM20948 object with the ICM-20948 sensor on I2C bus 0 with address 0x69
int status;

bool dataAvailable = false;
int dataTime = 0;
int lastDataTime = 0;
int runCount = 0;
float MagX;
float MagY;
float MagZ;
float MagXAvg = 0;
float MagYAvg = 0;
float MagZAvg = 0;
#define sampleSize 10000 //Number of data points used to find the average

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  Serial.print("status = ");
  Serial.println(status);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
   }
  IMU.configAccel(ICM20948::ACCEL_RANGE_16G, ICM20948::ACCEL_DLPF_BANDWIDTH_50HZ);
  IMU.configGyro(ICM20948::GYRO_RANGE_2000DPS, ICM20948::GYRO_DLPF_BANDWIDTH_51HZ);
  IMU.setGyroSrd(113); // Output data rate is 1125/(1 + srd) Hz
  IMU.setAccelSrd(113);
  IMU.enableDataReadyInterrupt();
  
}


void loop() {
  
  
  dataTime = micros();
  dataAvailable = true;
  if (dataAvailable) {
    dataAvailable = false;
    int timeDiff = dataTime - lastDataTime;
    lastDataTime = dataTime;
    IMU.readSensor();
    if (runCount == 0){
      Serial.print("Calculating average readings... \n");
      
      for (int i = 0; i < sampleSize; i++){
        MagX = (IMU.getMagX_uT());
        MagY = (IMU.getMagY_uT());
        MagZ = (IMU.getMagZ_uT());
        MagXAvg = (MagX + MagXAvg) / 2;
        MagYAvg = (MagY + MagYAvg) / 2;
        MagZAvg = (MagZ + MagZAvg) / 2;      
      }
      Serial.print("\n=====================================\n");
      Serial.print(MagXAvg);Serial.print(",");
      Serial.print(MagYAvg);Serial.print(",");
      Serial.print(MagZAvg);
      Serial.print("\n=====================================\n");
      runCount = 1;
    }
    //display the data
    //Serial.print(dataTime);
    //Serial.print("\t");
    Serial.print("delta t:");Serial.print(timeDiff);
    //Serial.print("\t");
    //Serial.print("Variable_1");
    //Raw data
    Serial.print("  A_x:");Serial.print(IMU.getAccelX_mss(),6);Serial.print(",");
    Serial.print("  A_y:");Serial.print(IMU.getAccelY_mss(),6);Serial.print(",");
    Serial.print("  A_z:");Serial.print(IMU.getAccelZ_mss(),6);Serial.print(",");
    Serial.print("  G_x:");Serial.print(IMU.getGyroX_rads(),6);Serial.print(",");
    Serial.print("  G_y:");Serial.print(IMU.getGyroY_rads(),6);Serial.print(",");
    Serial.print("  G_z:");Serial.print(IMU.getGyroZ_rads(),6);Serial.print(",");
    Serial.print("  M_x:");Serial.print(IMU.getMagX_uT(),6);Serial.print(",");
    Serial.print("  M_y:");Serial.print(IMU.getMagY_uT(),6);Serial.print(",");
    Serial.print("  M_z:");Serial.print(IMU.getMagZ_uT(),6);Serial.print(",");

      //Bias removed
//    Serial.print("A_x:");Serial.print(IMU.getAccelX_mss(),6);Serial.print(",");
//    Serial.print("  A_y:");Serial.print(IMU.getAccelY_mss(),6);Serial.print(",");
//    Serial.print("  A_z:");Serial.print(IMU.getAccelZ_mss(),6);Serial.print(",");
//    Serial.print("  G_x:");Serial.print(IMU.getGyroX_rads(),6);Serial.print(",");
//    Serial.print("  G_y:");Serial.print(IMU.getGyroY_rads(),6);Serial.print(",");
//    Serial.print("  G_z:");Serial.print(IMU.getGyroZ_rads(),6);Serial.print(",");
//    
//    Serial.print("  relM_x:");Serial.print(IMU.getMagX_uT() - MagXAvg);Serial.print(",");
//    Serial.print("  relM_y:");Serial.print(IMU.getMagY_uT() - MagYAvg);Serial.print(",");
//    Serial.print("  relM_z:");Serial.print(IMU.getMagZ_uT() - MagZAvg);Serial.print(",");
//    //Serial.println(IMU.getTemperature_C(),6);
    Serial.print("\n");

    // plot the data
//    Serial.print(dataTime);
//    Serial.print("\n");
//    Serial.print(timeDiff);
//    Serial.print("\n");
//    Serial.print(IMU.getAccelX_mss(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getAccelY_mss(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getAccelZ_mss(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getGyroX_rads(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getGyroY_rads(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getGyroZ_rads(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getMagX_uT(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getMagY_uT(),6);
//    Serial.print("\n");
//    Serial.print(IMU.getMagZ_uT(),6);
//    Serial.print("\n");
//    Serial.println(IMU.getTemperature_C(),6);


  }

}
