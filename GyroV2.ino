//for gyro
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"

#define BAUDRATE 9600


int gyroScale = 131;
bool firstIteration = true;
const int offsetIterations = 6;

double currentTime, lastTime, timeStep;
double radsToDeg = (180/3.141592);
double offsetX = 0;

double startX = 0;

double errorAcceptance = 2;
typedef struct
{
 int16_t x;
 int16_t y;
 int16_t z; 

 int16_t *ptr_x = &x;
 int16_t *ptr_y = &y;
 int16_t *ptr_z = &z;
}Acceleration;

typedef struct
{
  double aX;
  double aY;
  double aZ; 
}AccelAngle;

typedef struct 
{
  int16_t x;
  int16_t y;
  int16_t z;

 int16_t *ptr_x = &x;
 int16_t *ptr_y = &y;
 int16_t *ptr_z = &z;
}Gyro;

typedef struct 
{
  int sX;
  int sY;
  int sZ;
}ScaledGyro;

typedef struct
{
 double aX;
 double aY;
 double aZ; 
}GyroAngle;

typedef struct 
{
  double rotX;
  double rotY;
  double rotZ; 
}Rotation;


Acceleration accel;
AccelAngle  aAccel;
Gyro gyro;
ScaledGyro sGyro;
GyroAngle aGyro;
Rotation rotation;

MPU6050 IMU;

void setup()
{
  Wire.begin();
  Serial.begin(BAUDRATE);
  IMU.initialize();
  currentTime = millis();
  delay(500);
  for(int i = 0; i < offsetIterations; i++)
  {
    updateTime();
    readIMUValues();
    scaleGyro();
    computeAccelAngle();
    integrateGyro();
    complementaryFilter();

    startX += rotation.rotX;
  }
    offsetX = startX/offsetIterations; 
    Serial.println(offsetX);
  
}

void loop()
{
  updateTime();
  readIMUValues();
  scaleGyro();
  computeAccelAngle();
  integrateGyro();
  complementaryFilter();
  logAngle();
  testDrive();
}


void updateTime()
{
  lastTime = currentTime;
  currentTime = millis();
  timeStep = (currentTime - lastTime) / 1000;
}

void readIMUValues()
{
  IMU.getMotion6(accel.ptr_x, 
                 accel.ptr_y,
                 accel.ptr_z,
                 gyro.ptr_x,
                 gyro.ptr_y,
                 gyro.ptr_z);
}

void scaleGyro()
{
  sGyro.sX = gyro.x/gyroScale;
  sGyro.sY = gyro.x/gyroScale;
  sGyro.sZ = gyro.z/gyroScale;
}

void computeAccelAngle()
{
  double sqrAccelY = pow(accel.y, 2);
  double sqrAccelZ = pow(accel.z, 2);
  double sqrAccelX = pow(accel.x, 2);

  double radicalY = sqrt(sqrAccelY + sqrAccelZ);
  double radicalZ = sqrt(sqrAccelX + sqrAccelY);
  double radicalX = sqrt(sqrAccelX + sqrAccelZ); 

  double aAccelY = -radsToDeg*(atan((accel.x)/(radicalY)));
  double aAccelZ = radsToDeg*atan((radicalZ/accel.z));
  double aAccelX = radsToDeg*atan((accel.y/radicalX)); 

  aAccel.aX = aAccelX;
  aAccel.aY = aAccelY;
  aAccel.aZ = aAccelZ;
}

void integrateGyro()
{
  if(firstIteration)
  {
    aGyro.aX = aAccel.aX;
    aGyro.aY = aAccel.aY;
    aGyro.aZ = aAccel.aZ;
  }else
  {
    aGyro.aX = aGyro.aX + (timeStep * sGyro.sX);
    aGyro.aY = aGyro.aY + (timeStep * sGyro.sY);
    aGyro.aZ = aGyro.aZ + (timeStep * sGyro.sZ);
  }
  
}

void complementaryFilter()
{
 rotation.rotX = (0.96 * aAccel.aX) + (0.04 * aGyro.aX) - offsetX;
 rotation.rotY = (0.96 * aAccel.aY) + (0.04 * aGyro.aY);
 rotation.rotZ = (0.96 * aAccel.aZ) + (0.04 * aGyro.aZ);
}

void logAngle()
{
 Serial.println(rotation.rotX);
 //Serial.println(rotation.rotY);
 //Serial.println(rotation.rotZ); 
}

void testDrive()
{
 if(rotation.rotX > 2.3)
 {
  Serial.println("Forward"); 
 } else if(rotation.rotX < -2.3)
 {
     Serial.println("Backward"); 
 }else
 {
  Serial.println("stopping");
  Serial.print(rotation.rotX); 
 }
  
}
