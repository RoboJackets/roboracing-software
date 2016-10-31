#include <MPU9250.h>
#include <quaternionFilters.h>

#include <LiquidCrystal.h>

#include <Servo.h>

void motor(int val);
void steer(int val);
void update();
void updateHeading();
void updateSpeed();
bool getMessage();
void encoderTick();
int limitDesiredSpeed(int requestedSpeed);
int limitDesiredHeading(int requestedHeading);

//LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

MPU9250 imu;

static int LED = 13;

//Variables for motors & state variables
Servo VXL3sMotor;
static int VXL3sMotorPin=5;
Servo SteeringMotor;
static int SteeringMotorPin=6;

// set up vars for PD speed control
const static int   encoderPin1 = 2;
const static int   encoderPin2 = 3;
const static int   pid_p = 0.1;
const static int   pid_i = 0.1;
const static int   pid_d = 0.1;
const static int   ticks_per_rotation = 1000;
const static float meters_per_rotation = 0.1;
// max interrupt rate is (car m/s) / (m/rot) * (ticks/rot)

static volatile int currentTicks = 0; //volatile data for manipulation in interrupt routines
static          int lastTicks = 0;

#define HISTORY_SIZE 100
static int           currentMotorPwm = 0;
static float         desiredSpeed = 0;
static float         errorSum = 0;
static float         errorHistory[HISTORY_SIZE] = {0};
static unsigned long lastSpeedUpdateMicros = 0;
static int           historyIndex = 0;

static int currentHeading = 0;
static int desiredHeading = 0;

//control limits
static const int maxSpeed = 10; // maximum target speed in m/s
static const int minSpeed = -5;
static const int minSteer = -25; // leftmost target servo location (zero center)
static const int maxSteer = 25;

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
  Serial.begin(115200);
  setupIMU();
  VXL3sMotor.attach(VXL3sMotorPin);
  SteeringMotor.attach(SteeringMotorPin);
  motor(0);
  steer(0);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), encoderTick, CHANGE);
//  lcd.begin(16, 2);
  // Print a message to the LCD.
//  lcd.print("Steer    Speed");
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

//  lcd.setCursor(0, 1);
//  lcd.print("               ");
//  lcd.setCursor(0, 1);
//  lcd.print(desiredHeading);
//  lcd.setCursor(9,1);
//  lcd.print(desiredSpeed);
}

void motor(int val){
  VXL3sMotor.write(val+90);
}

void steer(int val)
{
  SteeringMotor.write(val+90);
}

void update()
{
  if(getMessage()) {
    // limit the input values
    desiredSpeed = limitDesiredSpeed(desiredSpeed);
    desiredHeading = limitDesiredHeading(desiredHeading);
  }
  updateHeading();
  updateSpeed();
}

//Ensure desired ESC input does not exceed limits
int limitDesiredSpeed(int requestedSpeed)
{
  return min(maxSpeed, max(requestedSpeed, minSpeed));
}
int limitDesiredHeading(int requestedSteer)
{
  return min(maxSteer, max(requestedSteer, minSteer));
}

void updateHeading()
{
  if(currentHeading != desiredHeading) {
    currentHeading = desiredHeading; //set heading to desired heading
    steer(currentHeading); //update rotation to reflect changed value
  }
}

// Set the speed based on a PID algorithm
void updateSpeed()
{
  // update the speed
  float deltaMeters = (float)(currentTicks - lastTicks) / ticks_per_rotation * meters_per_rotation;
  float deltaSeconds = (float)(micros() - lastSpeedUpdateMicros) / 1000000;
  float currentError = desiredSpeed - (deltaMeters / deltaSeconds);
  

  // find derivative of error
  float derivError = (float)(currentError - errorHistory[historyIndex]) 
                            / (micros() - lastSpeedUpdateMicros);

  // update integral error
  // TODO test assumption of even-enough spacing of the measurements thru time
  // TODO test assumption that floating point inaccuracies won't add up too badly
  historyIndex = (historyIndex+1) % HISTORY_SIZE;
  errorSum -= errorHistory[historyIndex];
  errorSum += currentError;
  float integralError = errorSum / HISTORY_SIZE;
  
  // combine PID terms
  float targetMotorPwm = pid_p * currentError + pid_i * integralError + pid_d * derivError;
  
  // store previous state info
  lastSpeedUpdateMicros = micros();
  errorHistory[historyIndex] = currentError;
  lastTicks = currentTicks;
  

  // update acutal PWM value. Slows down change rate to ESC to avoid errors
  motor(currentMotorPwm);
}

bool getMessage()
{
  bool gotMessage = false;
  while(Serial.available())
  {
    if(Serial.read() == '$')
    {
      desiredSpeed = Serial.parseFloat();
      desiredHeading = Serial.parseFloat();
      gotMessage = true;
    }
  }
  return gotMessage;
}

// run when encoder value changes
void encoderTick()
{
  if (digitalRead(encoderPin1) == digitalRead(encoderPin2)) {
    currentTicks++;
  } else {
    currentTicks--;
  }
}
