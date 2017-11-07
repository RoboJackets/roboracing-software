//#define DEBUG 0 //num is the sensor to debug
#define NUM_SENSORS 6
#define BAUD_RATE 9600

//Firmware for sensor array with HC-SR04 ultrasonic sensors


long ping(int trigPin, int echoPin) {
  //outputs distance (in cm currently; #TODO change that?)

  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(1); //May be unnecessary #TODO
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  distance = (duration / 2) / 29.1;
  return distance;

}

//#TODO: write a version of ping that does not use delay. Use states. i=0;j=NUM_SENSORS/2 First state check sensor i and j, increment for next state, etc
//#TODO: https://www.bananarobotics.com/shop/HC-SR04-Ultrasonic-Distance-Sensor for info on sensor

int trigPins[] = {13}; //trigger pins in order of left to right
int echoPins[] = {12}; //echo pins in order of left to right
long distances[NUM_SENSORS];

int state = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    distances[i] = ping(trigPins[i], echoPins[i])
  }


#ifdef DEBUG
  if (distances[DEBUG] >= 200 || distances[DEBUG] <= 0){
    Serial.println("Out of range");
  }
  else {
    Serial.print(distances[DEBUG]);
    Serial.println(" cm");
  }
#endif


  //output distances to serial
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(distances[i]);
    if (i != NUM_SENSORS - 1) {
      Serial.print(",");
    }
  }
  Serial.println();

  delay(200); //may want to lessen this #TODO
}
