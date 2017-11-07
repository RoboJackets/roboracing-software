#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>


//#define DEBUG 0 //num is the sensor to debug
#define NUM_SENSORS 6

//Firmware for sensor array with HC-SR04 ultrasonic sensors


long ping(int trigPin, int echoPin) {}

//#TODO: write a version of ping that does not use delay. Use states. i=0;j=NUM_SENSORS/2 First state check sensor i and j, increment for next state, etc
//#TODO: https://www.bananarobotics.com/shop/HC-SR04-Ultrasonic-Distance-Sensor for info on sensor

int trigPins[] = {13}; //trigger pins in order of left to right
int echoPins[] = {12}; //echo pins in order of left to right
long distances[NUM_SENSORS];


int state = 0;

ros::NodeHandle  nh;
sensor_msgs::PointCloud sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);

tf::TransformListener listener;

char[] chassis_link = "/chassis";
char[] sensor_link = "/ultrasonic_" //#TODO: update this for all of the sensors?


void setup() {
  nh.initNode();
  nh.advertise(pub_sonar);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

}

void loop() {[SOURCE]
  for (int i = 0; i < NUM_SENSORS; i++) {
    distances[i] = ping(trigPins[i], echoPins[i])
  }

  tf::usTransform0;
  tf::usTransform1;
  tf::usTransform2;
  tf::usTransform3;
  tf::usTransform4;
  tf::usTransform5;

  if(tf_listener.waitForTransform(chassis_link, sensor_link, ros::Time(0), ros::Duration(3.0))) {
    // source -> target
    // target, source, when, result
    tf_listener.lookupTransform("/base_link", "/ball", ros::Time(0), transform);
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


  //transform points from sensor to chassis



  //Sonar Message
  sonar_msg.header.stamp = nh.now();
  pub_sonar.publish(&sonar_msg);
  nh.spinOnce();

  delay(200); //may want to lessen this #TODO
}



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
