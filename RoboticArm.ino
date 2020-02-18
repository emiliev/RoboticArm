#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

bool initLoop = false;
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600// this is the 'maximum' pulse length count (out of 4096)
#define FREQUENCY 50

/// Home position
//float thitaAngles[] = { 0, 90, -90, 180, -90};
float thitaAngles[] = { 0, 90, 90, 180, 90}; // <- modified angles just for testing the home position
float numberServos = 5;
/*
 * 1st is correct 
 * 2nd looks correct
 * 3rd one should be taken by abs in order to be correct
 * 4th might be correct :/
 * 5th should be takeb by abs, also it has physical limitation!!!!!!
 */
/* 
 * Elbow moves between 240-SERVOMAX (600 pulse width) 
 */

int pulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.print("Analog value: ");
  Serial.println(angle);
  Serial.print("Pulse Width value: ");
  Serial.println(analog_value);
  return pulse_wide;
}

void setup() {
  Serial.begin(9600);
  
//  Serial.print("Hello world\n");
  // Set tolerance to 0.5mm. If reachable, the end effector will approach
  // the target with this tolerance
  pwm.begin();
  pwm.setPWMFreq(60);
//  pwm.setPWM(0, 0, 150);
  // Temporary set delay in order to be sure that the pwm is configured!
  delay(500);
  //homePosition();
}
/*
void loop() {
    if (initLoop) {
        for (int index = 0; index < numberServos; ++index) {
            float currentAngle = thitaAngles[index];
            float receivedAngles[] = {90, 120, 90, 180, 90};
            float newAngle = receivedAngles[index];
            
            /// calculating the pulse width
            int fromPulse = pulseWidth(currentAngle);
            int toPulse = pulseWidth(newAngle);
            
            /// rotating backwards 
            if (currentAngle > newAngle) {
                 loopBackward(fromPulse, toPulse, index);
            } else if (currentAngle < newAngle ) {
                /// rotating forwards
                loopForward(fromPulse, toPulse, index);
            } else {
                /// do nothing, the angles are equal  
            }
            Serial.println("");
            Serial.println(newAngle);
            /// updating new joint angles 
            thitaAngles[index] = newAngle;
       }
  }

  delay(1000);
  for (uint16_t pulselen = 150; pulselen < 600; pulselen++) {
        pwm.setPWM(0, 0, pulselen );  
        delay(5);
  }
  
  for (uint16_t pulselen = 600; pulselen > 150; pulselen--) {
        pwm.setPWM(0, 0, pulselen );  
        delay(5);
  }
  initLoop = false;  
  delay(1000);
}
*/

void loopForward(int currentValue, int targetValue, int jointIndex) {
    Serial.print("Looping forward");
    Serial.print(currentValue);
    Serial.print("\t");
    Serial.print(targetValue);
    for (uint16_t pulselen = currentValue; pulselen < targetValue; pulselen++) {
        pwm.setPWM(jointIndex, 0, pulselen);
        delay(50);
        Serial.println(pulselen);
    }
}

void loopBackward(int currentValue, int targetValue, int jointIndex) {
    Serial.print("Looping forward");
    Serial.print(currentValue);
    Serial.print("\t");
    Serial.print(targetValue);
    for (uint16_t pulselen = currentValue; pulselen > targetValue; pulselen--) {
        pwm.setPWM(jointIndex, 0, pulselen);
        delay(10);
    }
}


void homePosition() {
  for(int index = 0; index < numberServos; ++index) {
      int angle = thitaAngles[index];
      float width = pulseWidth(angle);
      loopForward(300, width, index);
  }
}

float serialInput[6];
int currentIndex = 0;
bool hasNewData = false;

void loop() {

    while(Serial.available() > 0) {
        float inputData = Serial.parseFloat();
        Serial.println(inputData , DEC);
        serialInput[currentIndex] = inputData;
        currentIndex += 1;
        hasNewData = true;
    }

    if(hasNewData) {
        for(int index = 0; index < currentIndex; ++index) {
            Serial.print(" Received value = ");
            Serial.println(serialInput[index], DEC);
        }
        currentIndex = 0;
        hasNewData = false;

        // homePosition();
   //     loopForward(150, 600, 0);
//        loopBackward(600, 150, 0);
    }
}
