/*  
    Brent Misare, Brett Schearer, EENG350

    Description: This arduino code implelemts the controller for the motor based
    on data from Matlab simulations.

    Circuit: 
    Attach Motor Driver to the Arduino
    Motor Red    <->  Motor Driver M1A
    Motor Black  <->  Motor Driver M1B
    Motor Green  <->  Ground
    Motor Blue   <->  5 VDC
    Motor Yellow <->  DI 2
    Motor White  <->  DI 6
    Battery Circuit + <-> Motor Driver VIN
    Battery Circuit - <-> Motor Driver GND
    
    Useful Equations:
    pulse width in ms = (speedLeft/255)*(2 ms) 

*/

//=======================================================================================================
//Libraries

#include <Wire.h>
#include <Encoder.h>

//=======================================================================================================
//Defines variables/constants

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
#define COMS   13     // Sets pin for i2c communication

#define SLAVE_ADDRESS 0x04  // Sets the address for the arduino when communicating with the pi

int voltLeft       = 0;   // Stores the PWM "voltage" going to the motor (0 - 255)

int   startTime    = 0;   // Stores the start time of the loop
int   endTime      = 0;   // Stores the end time of the loop
int   timeDelay    = 10;  // Sample/cycle period in ms
float timeDelaySec = (float)timeDelay / 1000;  // time delay in seconds

int   encCountLeft = 0;    // Stores the encoder count
float thetaLeft    = 0;    // Stores angle value for the left wheel
float thetaLeftOld = 0;    // Stores previous angle value for the left wheel  
float thetaLeftVel = 0;    // Stores the angular velovity of the left wheel
float thetaLeftDes = 0;    // Stores the desired angle for the left wheel
int   thetaLeftDir = HIGH; // Stores the current direction of the motor

float Kp = 300; // Describes values/constants for controller
float Ki = 60;
float Kd = 0;

int quad = 0; // Stores the intended quadrant recieved from the pi (0-3)

//=======================================================================================================
// Defines encoder objects

Encoder wheelLeft(LA, LB);
Encoder wheelRight(RA, RB);

//=======================================================================================================
// Setup

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
  analogWrite(M1VOLT, 0);      // writes a PWM signal to the voltage pin
  digitalWrite(M1SIGN, HIGH);  // writes to the voltage sign pin

  // Sets up serial communication
  Serial.begin(250000);
  Serial.println("Motor/Encoder Test:\n");

  // initialize i2c as slave
  pinMode(COMS, OUTPUT);
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

}

//=======================================================================================================

void loop() {
  // Stores the current run time to use in the delay at the endo of the loop
  startTime = millis();

  // Sets theta desired based on signal from i2c
  if((quad >= 0) && (quad <= 3)){
    thetaLeftDes = quad * (PI/2);    
  }

  // Calculates and stores the angle of the wheel in radians
  thetaLeftOld = thetaLeft;
  encCountLeft = wheelLeft.read();
  thetaLeft    = (-1)*(float)encCountLeft  * (2*PI/3200);
  thetaLeftVel = (thetaLeft - thetaLeftOld) / (timeDelaySec);

  // PID controller
  voltLeft = (thetaLeftDes - thetaLeft) * (Kp + Ki * ((thetaLeft - thetaLeftOld) * timeDelaySec) + (Kd * thetaLeftVel));
  if(voltLeft < 0){
    thetaLeftDir = HIGH;
    voltLeft = (-1)*voltLeft;
  }else{
    thetaLeftDir = LOW;
  }
  if(voltLeft > 255){
    voltLeft = 255;
  }

  // Sends calculated voltage to motor 
  digitalWrite(M1SIGN, thetaLeftDir);
  analogWrite(M1VOLT, voltLeft);

  // Prints out various position/velocity/PWM values
  Serial.print((float)millis()/1000);
  Serial.print("\t");
  Serial.print(thetaLeftDes);
  Serial.print("\t");
  Serial.print(thetaLeft);
  Serial.print("\t");
  Serial.print(voltLeft);
  Serial.print("\t");
  Serial.print(quad);
  Serial.print("\n");
  
  // time delay
  endTime = millis();
  if( timeDelay - (endTime - startTime) <= 0){
    Serial.print("ERROR!");
  }else{
    delay(timeDelay - (endTime - startTime));
  }
}


//=======================================================================================================
// Transmit/Recieve functions

// callback for received data
void receiveData(int byteCount){
  int tempQuad = 0;

  //Serial.print("Data Recieved: "); 
  while(Wire.available()) {    
    tempQuad = Wire.read();
  }
  
  if(tempQuad != 6){
    quad = tempQuad;
  }
}


// callback for sending data
void sendData(){
  int count = abs(encCountLeft);
  int data[4];

  //stores encoder value as an array of 4 bytes
  data[3] = count / 1000;
  count = count % 1000;
  data[2] = count / 100;
  count = count % 100;
  data[1] = count / 10;
  count = count % 10;
  data[0] = count;

  // Sends encoder value to pi one byte at a time
  for(int i = 0; i < 4; i++){
    Wire.write(data[i]);
    Serial.print(data[i]);
  }  
  Serial.print("\n");
}
