#include <Encoder.h>

/*  Brent Misare, EENG350

    Description: 

    Equation:
    pulse width in ms = (speedLeft/255)*(2 ms)

*/

//=======================================================================================================

#define LA     2      // Sets pin for left wheel encoder signal A (Interupt compatable)
#define LB     6      // Sets pin for left wheel encoder signal B
#define RA     3      // Sets pin for right wheel encoder signal A (Interupt compatable)
#define RB     5      // Sets pin for right wheel encoder signal B

#define ENABLE 4      // Sets pin for motors 1 and 2 enable (Low = Deactivated)
#define M1SIGN 7      // Sets pin for motor 1 voltage sign  (effectively direction)
#define M2SIGN 8      // Sets pin for motor 2 voltage sign  (effectively direction)
#define M1VOLT 9      // Sets pin for motor 1 voltage       (effectively speed)
#define M2VOLT 10     // Sets pin for motor 2 voltage       (effectively speed)
#define FAULT  12     // Sets pin fault feedback


//Defines Variables/constants
int speedLeft    = 0;
int tempTime     = 0;
float angLeft    = 0;
float angLeftOld = 0;     // stores the positions of the wheels
float angVel     = 0;

// Defines encoders
Encoder wheelLeft(LA, LB);
Encoder wheelRight(RA, RB);

//=======================================================================================================

void setup() {
  // Initializes various pins
  // Encoder pins are set automatically with the "Encoder" library
  pinMode(ENABLE, OUTPUT);
  pinMode(M1SIGN, OUTPUT);
  pinMode(M2SIGN, OUTPUT);
  pinMode(M1VOLT, OUTPUT);
  pinMode(M2VOLT, OUTPUT);

  // Sets initial conditions of pins
  digitalWrite(ENABLE, HIGH); 
  analogWrite(M1VOLT, 0);    // writes a PWM signal to the voltage pin
  digitalWrite(M1SIGN, HIGH);  // writes to the voltage sign pin

  // Sets up serial communication
  Serial.begin(250000);
  Serial.println("Motor/Encoder Test:\n");
}

//=======================================================================================================

void loop() {
  tempTime = millis();
  
  if((millis() >= 1000) && (millis() <= 4000)){
    speedLeft = 255;    // writes a PWM signal to the voltage pin
  }else{
    speedLeft = 0;
  }
  
  analogWrite(M1VOLT, speedLeft);
  
   // stores the angles of the wheels in radians
  angLeft  = (float)wheelLeft.read()  * (2*PI/3200);

  angVel = (angLeft - angLeftOld)/0.01;

  angLeftOld = angLeft;
  
  // Prints the angular position of the encoders in radians
  Serial.print((float)millis()/1000);
  Serial.print("\t");
  Serial.print((double)speedLeft * 0.04705882352);
  Serial.print("\t");
  Serial.print(angVel);
  //Serial.print("\t");
  //Serial.print(angLeft);
  Serial.print("\n");
  
  // time delay in ms
  if((millis() - tempTime) > 10){
    Serial.print("ERROR!");
  }
  delay(10 - (millis() - tempTime));
}
