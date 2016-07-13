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

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

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

void setup() 
{ 
  Serial.begin(9600);
  VXL3sMotor.attach(VXL3sMotorPin);
  SteeringMotor.attach(SteeringMotorPin);
  motor(0);
  delay(150);
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Steer    Speed");
} 

void loop() {
  
  update();
  //printSpeedNHeading();
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
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

