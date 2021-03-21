/*  
    Brent Misare, Perry Rodenbeck, Brett Schearer, Nick Zimkas
    EENG350

    Description: This arduino code implelemts the controller for the two motors based
    on data from Matlab simulations. It allows the robot to move based on desired
    distance and angle values.

    Circuit: 
    Attach Motor Driver to the Arduino
    
    Left Wheel:
    Motor Red    <->  Motor Driver M1A
    Motor Black  <->  Motor Driver M1B
    Motor Green  <->  Ground
    Motor Blue   <->  5 VDC
    Motor Yellow <->  DI 2
    Motor White  <->  DI 6
    
    Right Wheel:
    Motor Red    <->  Motor Driver M2A
    Motor Black  <->  Motor Driver M2B
    Motor Green  <->  Ground
    Motor Blue   <->  5 VDC
    Motor Yellow <->  DI 3
    Motor White  <->  DI 5
    
    Battery Circuit + <-> Motor Driver VIN
    Battery Circuit - <-> Motor Driver GND
    

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
#define MLSIGN 7      // Sets pin for motor 1 voltage sign  (effectively direction)
#define MRSIGN 8      // Sets pin for motor 2 voltage sign  (effectively direction)
#define MLVOLT 9      // Sets pin for motor 1 voltage       (effectively speed)
#define MRVOLT 10     // Sets pin for motor 2 voltage       (effectively speed)
#define FAULT  12     // Sets pin fault feedback
#define COMS   13     // Sets pin for i2c communication

#define SLAVE_ADDRESS 0x04  // Sets the address for the arduino when communicating with the pi

float wheelRad = 0.24600;  // Radius of the wheels in feet (0.242782 measured) (Increasing this decreases travel distance)
float botDiam  = 0.88000; // Distance between the wheels in feet (0.85958 measured) (Increasing this increases travel distance)

int   voltLeft  = 0;   // Stores the PWM "voltage" going to the motor (0 - 255)
int   voltRight = 0;   // Stores the PWM "voltage" going to the motor (0 - 255)
float voltBar   = 0;   // Stores Vbar, the sum of the two voltages
float voltDelta = 0;   // Stores delta V, voltRight - voltLeft

unsigned long   startTime    = 0;   // Stores the start time of the loop
unsigned long   endTime      = 0;   // Stores the end time of the loop
int             timeDelay    = 10;  // Sample/cycle period in ms

int   encCountLeft      = 0;    // Stores the encoder counts
int   encCountRight     = 0;    
int   encCountLeftPrev  = 0;    // Stores the previous encoder counts
int   encCountRightPrev = 0;   

unsigned long timeLeft      = 0; // stores the times between encoder counts
unsigned long timeLeftPrev  = 0;
unsigned long timeRight     = 0; // stores the times between encoder counts
unsigned long timeRightPrev = 0;

float thetaLeft     = 0;   // Stores the angular position of the left wheel
float thetaRight    = 0;   // Stores the angular position of the right wheel
float thetaLeftdot  = 0;   // Stores the angular velovity of the left wheel
float thetaRightdot = 0;   // Stores the angular velovity of the right wheel

// Position variables (Rho)
float rho           = 0;     
float rhoDes        = 0;
float rhoErr       = 0;
float rhoErrPrev   = 0;

// Angular Position Variables (Phi)
float phi           = 0;     
float phiDes        = 0;
float phiErr       = 0;
float phiErrPrev   = 0;

// Velocity variables (Rho dot)
float rhoDot       = 0;  
float rhoDotMax    = 1;   
float rhoDotPrev   = 0;
float rhoDotDes    = 0;
float rhoDotErr    = 0;
float rhoDotInt    = 0;
float rhoDotIntMax = 1.02;  // Maximum value for the integral sum

// Angular velocity variables (Phi dot)
float phiDot       = 0;
float phiDotMax    = PI/2.0;
float phiDotPrev   = 0;
float phiDotDes    = 0;
float phiDotErr    = 0;
float phiDotInt    = 0;
float phiDotIntMax = 1.457; // Maximum value for the integral sum

// Controller Constants
float Kp_phi    = 2.5;  
float Kd_phi    = 0.5;  
float Kp_rho    = 6.3;
float Kd_rho    = 1.7;
float Kp_phiDot = 0;
float Ki_phiDot = 350;
float Kp_rhoDot = 0;
float Ki_rhoDot = 500;

// Stores the intended quadrant recieved from the pi (0-3)
int quad = 0; 

//=======================================================================================================
// Sets up Encoders

void encISRLeft(){
  // Reads values
  int valA = digitalRead(2);
  int valB = digitalRead(6);
  // Compares the values for A and B and increments accordingly
  if((valA == valB)){
    encCountLeft--;
  }else{
    encCountLeft++;
  } 
  // Calculates and updates wheel velocity
  timeLeft = micros();
  thetaLeft = encCountLeft * (PI/400.0);
  thetaLeftdot = (encCountLeft - encCountLeftPrev) * (PI/400.0) * (1000000.0) / (timeLeft - timeLeftPrev);
  // Updates "previous" values
  encCountLeftPrev = encCountLeft;
  timeLeftPrev = timeLeft;
}

void encISRRight(){
  // Reads values
  int valA = digitalRead(3);
  int valB = digitalRead(5);
  if((valA == valB)){
    encCountRight++;
  }else{
    encCountRight--;
  }
  // Calculates and updates wheel velocity
  timeRight = micros();
  thetaRight = encCountRight * (PI/400.0);
  thetaRightdot = (encCountRight - encCountRightPrev) * (PI/400.0) * (1000000.0) / (timeRight - timeRightPrev);
  // Updates "previous" values
  encCountRightPrev = encCountRight;
  timeRightPrev = timeRight;
}

//=======================================================================================================
// Wheel functions

// moves the left wheel based on the PWM voltage value
void moveLeft(int voltage){
  voltage = normalize(voltage, 255);
  if(voltage < 0){
    digitalWrite(MLSIGN, LOW);
    voltage = abs(voltage);
  }else{
    digitalWrite(MLSIGN, HIGH);  
  }
  analogWrite(MLVOLT, voltage);
}

// moves the right wheel based on the PWM voltage value
void moveRight(int voltage){
  voltage = normalize(voltage, 255);
  if(voltage < 0){
    digitalWrite(MRSIGN, HIGH);
    voltage = abs(voltage);
  }else{
    digitalWrite(MRSIGN, LOW);
  }
  analogWrite(MRVOLT, voltage);
}

// Stops both wheels
void stopWheels(){
  digitalWrite(MLSIGN, HIGH);
  digitalWrite(MRSIGN, LOW);
  analogWrite(MLVOLT, 0);
  analogWrite(MRVOLT, 0);
}

// restrains values between the value "cap" and negative "cap"
float normalize(float voltage, float cap){
  if(voltage > cap){
    voltage = cap;
  }else if(voltage < (-1*cap)){
    voltage = (-1*cap);
  }
  return voltage;
}

// Sets the desired values to the current values, stopping the robot in place
void forget(){
  phiDes = phi;
  rhoDes = rho;
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



//=======================================================================================================
// Setup

void setup() {
  // Initializes various pins
  // Encoder pins are set automatically with the "Encoder" library
  pinMode(ENABLE, OUTPUT);
  pinMode(MLSIGN, OUTPUT);
  pinMode(MRSIGN, OUTPUT);
  pinMode(MLVOLT, OUTPUT);
  pinMode(MRVOLT, OUTPUT);
  pinMode(LA,INPUT);
  pinMode(LB,INPUT);
  pinMode(RA,INPUT);
  pinMode(RB,INPUT);

  // Sets initial conditions of pins
  digitalWrite(ENABLE, HIGH); 
  analogWrite(MLVOLT, 0);      // writes a PWM signal to the voltage pin
  analogWrite(MRVOLT, 0);      // writes a PWM signal to the voltage pin
  digitalWrite(MLSIGN, HIGH);  // writes to the voltage sign pin
  digitalWrite(MRSIGN, HIGH);  // writes to the voltage sign pin

  // Sets up serial communication
  Serial.begin(250000);

  // Attaches pins and ISR to interupt (for encoders)
  attachInterrupt(digitalPinToInterrupt(2), encISRLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(3), encISRRight, RISING);
  
  // initialize i2c as slave
  pinMode(COMS, OUTPUT);
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

}



//=======================================================================================================
// Main Loop

void loop() {
  // Stores the current run time to use in the delay at the endo of the loop
  startTime = millis();

  // Sets desired position and angle values
  if(startTime > 8000){
    rhoDes = 1.0;
  }else if(startTime > 7500){
    forget();
  }else if(startTime > 3000){
    phiDes = -PI;
    rhoDes = 0.0;
  }
  
  // Angular Controller Outer Loop
  phi = wheelRad * (1.0 / botDiam) * (thetaRight - thetaLeft);
  phiErr = phiDes - phi;
  phiDotDes = (phiErr * Kp_phi) + (Kd_phi * (phiErr - phiErrPrev) * (1000.0 / timeDelay));
  phiDotDes = normalize(phiDotDes, phiDotMax);
  
  // Position Controller Outer Loop
  rho = wheelRad * (0.5) * (thetaRight + thetaLeft);
  rhoErr = rhoDes - rho;
  rhoDotDes = (rhoErr * Kp_rho) + (Kd_rho * (rhoErr - rhoErrPrev) * (1000.0 / timeDelay));
  rhoDotDes = normalize(rhoDotDes, rhoDotMax);
  
  
  // Angular Velocity Controller
  phiDot = wheelRad * (thetaRightdot - thetaLeftdot) * (1.0 / botDiam);
  phiDotErr = phiDotDes - phiDot;
  phiDotInt += (phiDotErr) * (timeDelay / 1000.0);
  if(phiDotInt > phiDotIntMax){
    phiDotInt = phiDotIntMax;
  }
  voltDelta = (phiDotErr * Kp_phiDot) + (Ki_phiDot * phiDotInt);

  // Velocity Controller
  rhoDot = (0.5) * wheelRad * (thetaLeftdot + thetaRightdot);
  rhoDotErr = rhoDotDes - rhoDot;
  rhoDotInt += (rhoDotErr) * (timeDelay / 1000.0);
  if(rhoDotInt > rhoDotIntMax){
    rhoDotInt = rhoDotIntMax;
  }
  voltBar = (rhoDotErr * Kp_rhoDot) + (Ki_rhoDot * rhoDotInt);

  // Calculated left/right voltage values
  voltLeft  = (0.5)*(voltBar - voltDelta);
  voltRight = (0.5)*(voltBar + voltDelta);

  // Updates previous values
  phiErrPrev = phiErr;
  rhoErrPrev = rhoErr;
  phiDotPrev  = phiDot;
  rhoDotPrev  = rhoDot;

  // Sends voltages to the motor
  voltLeft  = (int)(normalize(voltLeft, 255));
  voltRight = (int)(normalize(voltRight, 255));
  moveLeft(voltLeft);
  moveRight(voltRight);
  
  // Prints out various position/velocity/PWM values
  Serial.print(startTime / 1000.0);
  Serial.print("\t phiDes: ");
  Serial.print(phiDes);
  Serial.print("\t phi: ");
  Serial.print(phi);
  Serial.print("\t rhoDes: ");
  Serial.print(rhoDes);
  Serial.print("\t rho: ");
  Serial.print(rho);
  Serial.print("\t EncRight: ");
  Serial.print(encCountRight);
  Serial.print("\t EncLeft: ");
  Serial.print(encCountLeft);
  Serial.print("\t Vl: ");
  Serial.print(voltLeft);
  Serial.print("\t Vr: ");
  Serial.print(voltRight);
  Serial.print("\n");

  // Timing  
  endTime = millis();
  if( timeDelay - (endTime - startTime) <= 0){
    Serial.print("ERROR!");
  }else{
    delay(timeDelay - (endTime - startTime));
  }
}
