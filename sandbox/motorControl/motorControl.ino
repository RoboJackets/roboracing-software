#include <LiquidCrystal.h>

#include <Servo.h>

void motor(int val);
void steer(int val);
void update();
void limitDesiredValues(int& requestedSpeed, int& requestedHeading);
void updateHeading();
void updateSpeed();
bool getMessage();

//LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

static int LED = 13;

//Variables for motors & state variables
Servo EscMotor;
static int escPin = 10;
Servo SteeringMotor;
static int SteeringMotorPin=11;

static int currentSpeed = 0;
static int desiredSpeed=0;
static int currentHeading = 0;
static int desiredHeading = 0;

//control limits
static const int maxSpeed = 30; // maximum velocity 
static const int minSpeed = -15;
static const int minSteer = -25;
static const int maxSteer = 25;

void setup() 
{
  Serial.begin(115200);
  EscMotor.attach(escPin);
  SteeringMotor.attach(SteeringMotorPin);
  motor(0);
  steer(0);
//  lcd.begin(16, 2);
  // Print a message to the LCD.
//  lcd.print("Steer    Speed");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
} 

unsigned long time = 0;

void loop() {
  
  update();
//  lcd.setCursor(0, 1);
//  lcd.print("               ");
//  lcd.setCursor(0, 1);
//  lcd.print(desiredHeading);
//  lcd.setCursor(9,1);
//  lcd.print(desiredSpeed);
}

void motor(int val){
  EscMotor.write(val+90);
}

void steer(int val)
{
  SteeringMotor.write(val+90);
}

void update()
{
  if(getMessage()) {
    limitDesiredValues(desiredSpeed, desiredHeading);
  }
  updateHeading();
  updateSpeed();
}

//Ensure desired velocity does not exceed limits
void limitDesiredValues(int& requestedSpeed, int& requestedSteer)
{
  // speed
//  requestedSpeed = min(maxSpeed, max(requestedSpeed, minSpeed));

  // steer
//  requestedSteer = min(maxSteer, max(requestedSteer, minSteer));
      
}

void updateHeading()
{
  if(currentHeading != desiredHeading) {
    currentHeading = desiredHeading; //set heading to desired heading
    steer(currentHeading); //update rotation to reflect changed value
  }
}

//Make an incremental change to speed in the desired direction
void updateSpeed()
{
  if(currentSpeed != desiredSpeed) {
    currentSpeed = desiredSpeed;
    motor(currentSpeed);
    digitalWrite(LED, HIGH);
    delay(10);
    digitalWrite(LED, LOW);
  }
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

