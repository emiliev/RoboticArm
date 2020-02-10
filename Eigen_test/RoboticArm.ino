/********************************************************
 * FABRIK2D 4DOF example
 * Creating the FABRIK object and moving the end effector in a linear motion in x, y, z coordinates.
 * You can use whichever unit you want for coordinates, lengths and tolerances as long as you are consistent.
 * Default unit is millimeters.
 ********************************************************/

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

bool initLoop = false;
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600// this is the 'maximum' pulse length count (out of 4096)
#define FREQUENCY 50

/// Home position
float thitaAngles[] = { 0, 90, -90, 180, -90};
float numberServos = 5;

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

  // Set tolerance to 0.5mm. If reachable, the end effector will approach
  // the target with this tolerance
  pwm.begin();
  pwm.setPWMFreq(60);

  // Temporary set delay in order to be sure that the pwm is configured!
  delay(500);
//  homePosition();
}

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
//            } else if (currentAngle < newAngle ) {
                /// rotating forwards
//                loopForward(fromPulse, toPulse, index);
            } else {
                /// do nothing, the angles are equal  
            }
            Serial.println("");
            Serial.println(newAngle);
            /// updating new joint angles 
            thitaAngles[index] = newAngle;
       }

       lF(150, 550, 0);
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

void lF(int currentValue, int targetValue, int jointIndex) {
    Serial.print("Looping forward");
    Serial.print(currentValue);
    Serial.print("\t");
    Serial.print(targetValue);
    for(uint16_t pulselen = currentValue; pulselen < targetValue; pulselen++) {
        pwm.setPWM(jointIndex, 0, targetValue);
    }
}

void loopForward(int currentValue, int targetValue, int jointIndex) {
    Serial.print("Looping forward");
    Serial.print(currentValue);
    Serial.print("\t");
    Serial.print(targetValue);
    for (uint16_t pulselen = currentValue; pulselen < targetValue; pulselen++) {
        pwm.setPWM(jointIndex, 0, pulselen);
        delay(5);
    }
}

void loopBackward(int currentValue, int targetValue, int jointIndex) {
    Serial.print("Looping forward");
    Serial.print(currentValue);
    Serial.print("\t");
    Serial.print(targetValue);
    for (uint16_t pulselen = currentValue; pulselen > targetValue; pulselen--) {
        pwm.setPWM(jointIndex, 0, pulselen);
        delay(5);
    }
}

void homePosition() {
  for(int index = 0; index < numberServos; ++index) {
      int angle = thitaAngles[index];
      float width = pulseWidth(angle);
      pwm.setPWM(index, 0, width );
  }

//  pwm.setPWM(0, 0, SERVOMIN);  
//  pwm.setPWM(1, 0, 375);
//  pwm.setPWM(2, 0, 375);
//  pwm.setPWM(3, 0, SERVOMIN);
//  pwm.setPWM(4, 0, 275);
}
