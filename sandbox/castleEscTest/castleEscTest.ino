#include <Servo.h>

/**
 * This program test the functionality of the
 * Castle Sidewinder 3 ESC.
 */
Servo castleESC;
const int escPin = 5;

/*int pwm(float speed) {
    return (int) 
}*/

void setup() {
    Serial.begin(9600);

    pinMode(escPin, OUTPUT); //not sure if needed
    castleESC.attach(escPin);
}


const float vMin = 90;
const float vMax = 120;

void loop() {
    castleESC.write(vMin);
    delay(1000);

    castleESC.write(vMax);
    delay(1000);
}
