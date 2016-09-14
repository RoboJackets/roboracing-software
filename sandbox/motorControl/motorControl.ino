#include <MPU9250.h>
#include <quaternionFilters.h>

#include <LiquidCrystal.h>

// Testing the traxxas VXL 3s ESC by Seth Bergman. This works on mine from my Traxxas Rustler VXL.
// Please use with caution. I started with the Servo sketch by  Michal Rinott.

#include <Servo.h>  // create servo object to control a servo 
#include "Message.h"

//Prototypes
void motor(int val);
void steer(int val);
int update();
void printSpeedNHeading();
void limitDesiredValues(int& requestedSpeed, int& requestedHeading);
int updateHeading(int val);
int updateSpeed();

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

MPU9250 imu;

int LED = 13;

//Variables for motors & state variables
Servo VXL3sMotor;
int VXL3sMotorPin=9;
Servo SteeringMotor;
int SteeringMotorPin=10;

Message m;

int currentSpeed = 0;
int desiredSpeed=0;
int currentHeading = 0;
int desiredHeading = 0;
int j = 1;
int i = 0;

//control limits
const int maxFVel = 30; // maximum velocity 
const int maxRVel = -15;
int accelRate = 15; // change in velocity per second

//Determines how much time should run between increments
int computeDelay()
{
  return round(1000/accelRate);
}

//Returns 1 if num is positive, 0 if num is 0, -1 if num is negative
int computeSign(int num)
{
  return ((num>0)-(num<0));
}

void setupIMU()
{
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c != 0x71) {
    Serial.print("Incorrect who am I value for MPU9250 IMU. Expected 0x71, but got ");
    Serial.println(c, HEX);
  }

  // Calibrate gyro and accelerometers, load biases in bias registers
  imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

  // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  imu.initMPU9250();

  // Read the WHO_AM_I register of the magnetometer, this is a good test of
  // communication
  byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

  if (d != 0x48) {
    Serial.print("Incorrect who am I value for AK8963 Magnetometer. Expected 0x48, but got ");
    Serial.println(d, HEX);
  }

  // Get magnetometer calibration from AK8963 ROM
  imu.initAK8963(imu.magCalibration);
}

void updateIMU()
{
  imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values
  imu.getAres();

  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  imu.ax = (float)imu.accelCount[0]*imu.aRes; // - accelBias[0];
  imu.ay = (float)imu.accelCount[1]*imu.aRes; // - accelBias[1];
  imu.az = (float)imu.accelCount[2]*imu.aRes; // - accelBias[2];

  imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values
  imu.getGres();

  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  imu.gx = (float)imu.gyroCount[0]*imu.gRes;
  imu.gy = (float)imu.gyroCount[1]*imu.gRes;
  imu.gz = (float)imu.gyroCount[2]*imu.gRes;

  imu.readMagData(imu.magCount);  // Read the x/y/z adc values
  imu.getMres();
  // User environmental x-axis correction in milliGauss, should be
  // automatically calculated
  imu.magbias[0] = +470.;
  // User environmental x-axis correction in milliGauss TODO axis??
  imu.magbias[1] = +120.;
  // User environmental x-axis correction in milliGauss
  imu.magbias[2] = +125.;

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections
  // Get actual magnetometer value, this depends on scale being set
  imu.mx = (float)imu.magCount[0]*imu.mRes*imu.magCalibration[0] -
             imu.magbias[0];
  imu.my = (float)imu.magCount[1]*imu.mRes*imu.magCalibration[1] -
             imu.magbias[1];
  imu.mz = (float)imu.magCount[2]*imu.mRes*imu.magCalibration[2] -
             imu.magbias[2];

  // read temperature data
  imu.tempCount = imu.readTempData();
  imu.temperature = (float)imu.tempCount / 65.5360f;

  // Must be called before updating quaternions!
  imu.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx*DEG_TO_RAD,
                         imu.gy*DEG_TO_RAD, imu.gz*DEG_TO_RAD, imu.my,
                         imu.mx, imu.mz, imu.deltat);
}

void sendIMUData()
{
  Serial.print("$"); Serial.print(1000*imu.ax, 6);
  Serial.print(","); Serial.print(1000*imu.ay, 6);
  Serial.print(","); Serial.print(1000*imu.az, 6);

  Serial.print(","); Serial.print( imu.gx, 6);
  Serial.print(","); Serial.print( imu.gy, 6);
  Serial.print(","); Serial.print( imu.gz, 6);

  auto Q = getQ();
  Serial.print(","); Serial.print(Q[0], 6);
  Serial.print(","); Serial.print(Q[1], 6);
  Serial.print(","); Serial.print(Q[2], 6);
  Serial.print(","); Serial.print(Q[3], 6);

  Serial.print(","); Serial.print( imu.mx, 6);
  Serial.print(","); Serial.print( imu.my, 6);
  Serial.print(","); Serial.print( imu.mz, 6);

  Serial.print(","); Serial.print( imu.temperature, 6);
  
  Serial.println();
}

void setup() 
{ 
  Wire.begin();
  Serial.begin(9600);
  setupIMU();
  VXL3sMotor.attach(VXL3sMotorPin);
  SteeringMotor.attach(SteeringMotorPin);
  motor(0);
  delay(150);
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Steer    Speed");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
} 

unsigned long time = 0;

void loop() {
  
  update();
  updateIMU();
  //limit publishing rate of imu data
  if(millis() - time >= 50) {
    sendIMUData();
    time = millis();
    digitalWrite(LED, !digitalRead(LED));
  }
  lcd.setCursor(0, 1);
  lcd.print("               ");
  lcd.setCursor(0, 1);
  lcd.print(desiredHeading);
  lcd.setCursor(9,1);
  lcd.print(desiredSpeed);
}

void printSpeedNHeading()
{
  Serial.print("Desired Speed: ");
  Serial.print(desiredSpeed);
  Serial.print("Desired Heading: ");
  Serial.print(desiredHeading);
  Serial.println();
}


void motor(int val){
  val += 90;
  VXL3sMotor.write(val);
}

void steer(int val)
{
  val += 90;
  SteeringMotor.write(val);
}

int update()
{
  unsigned long startTime = millis();
  //Serial.println(m.getMessage2(desiredSpeed, desiredHeading)); //Check for new command
  int ret = m.getMessage2(desiredSpeed, desiredHeading);
  limitDesiredValues(desiredSpeed, desiredHeading); //limit values if they are beyond limits

  lcd.setCursor(0,0);
  lcd.print(ret);
  /*if(ret == -1) {
    lcd.print("NO MESSAGE");
  } else {
    lcd.print("MESSAGE");
  }*/

  
  updateHeading(desiredHeading);
  updateSpeed();
  
  //Compute run time of loop so far, dealy for amount required to enforced desired acceleration rate
  unsigned long stopTime = millis();
  long elapsed = stopTime-startTime;
  int remainingDelay = max(0, computeDelay()-elapsed);
  delay(remainingDelay);
}

//Ensure desired velocity does not exceed limits
void limitDesiredValues(int& requestedSpeed, int& requestedHeading)
{
  if (requestedSpeed > 0 && requestedSpeed > maxFVel)
  {requestedSpeed=maxFVel;}
  else if (requestedSpeed < 0 && abs(requestedSpeed) > abs(maxRVel))
  {requestedSpeed=maxRVel;}
}

int updateHeading(int val)
{
  currentHeading = desiredHeading; //set heading to desired heading
  steer(currentHeading); //update rotation to reflect changed value
}

//Make an incremental change to speed in the desired direction
int updateSpeed()
{
  currentSpeed+=computeSign(desiredSpeed-currentSpeed);
  motor(currentSpeed);
}

/*
int changeSpeed(int currentSpeed, int desiredSpeed)
{
  if (desiredSpeed > 0 && desiredSpeed > maxFVel)
  {desiredSpeed=maxFVel;}
  else if (desiredSpeed < 0 && abs(desiredSpeed) > abs(maxRVel))
  {desiredSpeed=maxRVel;}
  
  if(desiredSpeed == 0, abs(currentSpeed) > 25)
  {
    brake(0);
  }
  else if (desiredSpeed == 0, abs(currentSpeed) >15)
  {brake(1);
  }
  else if (desiredSpeed == 0, abs(currentSpeed) >5)
  {brake(2);
  }
  else if (currentSpeed < desiredSpeed)
  {  while(currentSpeed != desiredSpeed) 
    {currentSpeed++;
     computeDelay();
     motor(currentSpeed);    
    }
  }
  else if (currentSpeed > desiredSpeed)
  {  while(currentSpeed != desiredSpeed)
    {currentSpeed--;
     computeDelay();
     motor(currentSpeed);
    }
  }
  return currentSpeed;  
}
*/

int changeSpeed2(int& currentSpeed, int& desiredSpeed)
{
  if (desiredSpeed > 0 && desiredSpeed > maxFVel)
  {desiredSpeed=maxFVel;}
  else if (desiredSpeed < 0 && abs(desiredSpeed) > abs(maxRVel))
  {desiredSpeed=maxRVel;}
  
  if(desiredSpeed == 0, abs(currentSpeed) > 25)
  {
    brake(0);
  }
  else if (desiredSpeed == 0, abs(currentSpeed) >15)
  {brake(1);
  }
  else if (desiredSpeed == 0, abs(currentSpeed) >5)
  {brake(2);
  }
  else if (currentSpeed < desiredSpeed)
  {  
     currentSpeed++;
     computeDelay();
     motor(currentSpeed);    
  }
  else if (currentSpeed > desiredSpeed)
  {  
    currentSpeed--;
    computeDelay();
    motor(currentSpeed);
  }
  return currentSpeed;  
}



void brake(int level){

  switch(level){
  case 0:
    Serial.println("Hard Stop");
    motor(-2*currentSpeed);
    delay(abs(currentSpeed));
    delay(1000);
    motor(0);
    break; 

  case 1:
    motor(-currentSpeed);
    Serial.println("Soft Stop");
    delay(abs(currentSpeed)* 20);
    delay(1000);
    motor(0);
    break; 

  case 2:
    motor(0);
    Serial.println("Coast Stop");
    delay(abs(currentSpeed) * 70);
    delay(1000);
    motor(0);
    break; 
  }  
}

