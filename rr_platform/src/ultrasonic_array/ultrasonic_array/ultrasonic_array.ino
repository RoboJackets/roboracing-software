#define NUM_SENSORS 2
#define BAUD_RATE 9600
//multiplexer pins
#define S0 2
#define S1 3
#define S2 4
#define S3 5

#define SPEED_OF_SOUND 2941.2 //microseconds per meter!
//Firmware for sensor array with HC-SR04 ultrasonic sensors
//#TODO: write a version of ping that does not use delay. Increment through two at a time, need to write your own "pulseIn"
//#TODO: https://www.bananarobotics.com/shop/HC-SR04-Ultrasonic-Distance-Sensor for info on sensor

int trigPins[] = {2, 8}; //trigger pins in order of left to right
int echoPins[] = {3, 9}; //echo pins in order of left to right
double distances[NUM_SENSORS];


void switchMux(int channel) { //#TODO: implement mux stuff
  //Channels 0 - 15 on mux
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));

  }


double ping(int trigPin, int echoPin) {
  //outputs distance in meters

  long duration;
  double distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); //May be unnecessary #TODO
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  distance = ((double)duration / 2.0) / SPEED_OF_SOUND; //Distance = (Time x SpeedOfSound) / 2. The "2" is in the formula because the sound has to travel back and forth
  return distance;

}

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
    delay(5); //neccessary because echo from other sensor can interfere if lowre
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
