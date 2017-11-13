#define NUM_SENSORS 8
#define BAUD_RATE 9600
#define SPEED_OF_SOUND 294.12 //microseconds per meter!

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

  distance = (duration / 2) / SPEED_OF_SOUND; //Distance = (Time x SpeedOfSound) / 2. The "2" is in the formula because the sound has to travel back and forth
  return distance;

}

//#TODO: write a version of ping that does not use delay. Increment through two at a time, need to write your own "pulseIn"
//#TODO: https://www.bananarobotics.com/shop/HC-SR04-Ultrasonic-Distance-Sensor for info on sensor

int trigPins[] = {13}; //trigger pins in order of left to right
int echoPins[] = {12}; //echo pins in order of left to right
long distances[NUM_SENSORS];

void setup() {
  Serial.begin(BAUD_RATE);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

}

void loop() {

  for (int i = 0; i < NUM_SENSORS; i++) {
    distances[i] = ping(trigPins[i], echoPins[i]);
  }


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
