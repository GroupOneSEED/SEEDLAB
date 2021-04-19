/*  
    Brent Misare, Perry Rodenbeck, Brett Schearer, Nick Zimkas
    EENG350

    Description: This arduino code implelemts a finite state machine
    to have the robot seek a marker, turn towards it, drive up to it,
    and drive a circle around it.

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
//=======================================================================================================
#include <Wire.h>
#include <Encoder.h>

//=======================================================================================================
//Defines variables/constants
//=======================================================================================================
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

const float wheelRad = 0.24600;  // Radius of the wheels in feet (0.242782 measured) (Increasing this decreases travel distance)
const float botDiam  = 0.88000;  // Distance between the wheels in feet (0.85958 measured) (Increasing this increases travel distance)

int   voltLeft  = 0;   // Stores the PWM "voltage" going to the motor (0 - 255)
int   voltRight = 0;   // Stores the PWM "voltage" going to the motor (0 - 255)
float voltBar   = 0;   // Stores Vbar, the sum of the two voltages
float voltDelta = 0;   // Stores delta V, voltRight - voltLeft

unsigned long   startTime    = 0;   // Stores the start time of the loop
unsigned long   endTime      = 0;   // Stores the end time of the loop
int             timeDelay    = 10;  // Sample/cycle period in ms

// Stores encoder counts for both wheels
int   encCountLeft      = 0;
int   encCountRight     = 0;    
int   encCountLeftPrev  = 0;
int   encCountRightPrev = 0;   

// Stores times between encoder counts for both wheels
unsigned long timeLeft      = 0;
unsigned long timeLeftPrev  = 0;
unsigned long timeRight     = 0;
unsigned long timeRightPrev = 0;

// Angle (theta) values for the wheels
float thetaLeft     = 0;
float thetaRight    = 0;
float thetaLeftdot  = 0;
float thetaRightdot = 0;

// Position variables (Rho)
float rho          = 0;     
float rhoDes       = 0;
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

// Sets whether the controler sets the desired position or velocity
bool contVel  = false;

// Variables used to navigate the finite state machine
enum FSM {Boot, Seek, Ping1, Stop1, Ping2, Turn, Stop2, Forward, Stop3, Turn90, Stop4, Circle, Done};
FSM state      = Seek;
FSM nextState  = Seek;
bool firstLoop = true;
bool insideThresh = false;
int  FSMTimer  = 250;

// Used to store values read from the PI
float tempDummy  = 0;
float tempDist   = 0;   // in inches
float tempAngle  = 0;   // in deg
float tempTime   = 0;
int count = 0;

// If true, the robot recieves simulated PI values after 5 seconds of scanning (for testing)
bool TESTMODE = false;

//=======================================================================================================
// Sets up Encoders
//=======================================================================================================
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
// Wheel and Movement functions
//=======================================================================================================
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

// Sets various encoder/position/angle values to zero to stop the robot and reset it's position
void stopRobot(){
  encCountRight = 0;
  encCountLeft  = 0;
  encCountRightPrev = 0;
  encCountLeftPrev  = 0;
  phi = 0;
  rho = 0;
  phiDes = 0;
  rhoDes = 0;
  phiDotDes = 0;
  rhoDotDes = 0;
  phiDotInt = 0;
  rhoDotInt = 0;  
}

void moveRobot(float distance, float angle, int milliseconds){
  contVel = false;
  rhoDes = distance;
  phiDes = angle;
  FSMTimer = milliseconds;
}

void circleRobot(float radius, float milliseconds){
  contVel = true;
  rhoDotDes = PI * radius * (2000.0 / milliseconds);
  phiDotDes = PI * (2000.0 / milliseconds);
  FSMTimer = milliseconds + 200;
}

//=======================================================================================================
// Calculation functions
//=======================================================================================================
// restrains values between the value "cap" and negative "cap"
float normalize(float voltage, float cap){
  if(voltage > cap){
    voltage = cap;
  }else if(voltage < (-1*cap)){
    voltage = (-1*cap);
  }
  return voltage;
}

// Converst degrees to raians
float degToRad(float deg){
  return (deg * (PI / 180.0));
}

//=======================================================================================================
// Transmit/Recieve functions
//=======================================================================================================
// reads the three values from the PI
void readFromPI(){
  if(Wire.available()){
    count++;
    tempDummy = 0;
    tempAngle = 0;
    tempDist  = 0;
    while(Wire.available()){
      tempDummy = Wire.read();
      tempAngle = Wire.read();
      tempDist  = Wire.read();
    }
    // converts read values to distance/angle
    tempAngle = (tempAngle/2.0) - 32.0;
    tempDist  = (tempDist/2.0) - 14.0;
    if(tempDist < 0){
      tempDist = 0;
    }
    // converts to proper units (rad and feet)
    tempAngle = degToRad(tempAngle);
    tempDist = (tempDist / 12.0);
  }
}

// pings the PI, return true if there are values on the wire
bool pingPI(){
  if(Wire.available()){
    while(Wire.available()){ //clears the wire
      tempDummy = Wire.read();
    }
    return true;
  }else{
    return false;
  }
}

void receiveData(int byteCount){
}

void sendData(){
}


//=======================================================================================================
// Setup
//=======================================================================================================
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
//=======================================================================================================
void loop() {
  // Stores the current run time to use in the delay at the endo of the loop
  startTime = millis();

// - - - - - - - - - - - - - - - - - - - Finite State Machine - - - - - - - - - - - - - - - - - - - - - -   
  // Finite state machine (Boot, Seek, Ping1, Stop1, Ping2, Turn, Stop2, Forward, Stop3, Turn90, Stop4, Circle, Done)
  switch(state){
    case Boot:
      if(pingPI() == true){
        nextState = Seek;
      }else{
        nextState = Boot;
      }
      break;
    case Seek:
      contVel = true;
      rhoDotDes = 0;
      phiDotDes = (PI/12.0);
      if(FSMTimer <= 0){
        nextState = Ping1;
        if((startTime > 5000) && TESTMODE){
          nextState = Stop1;
          tempDist = 2.333;
          tempAngle = degToRad(45);
        }
      }
      break;
    case Ping1:  
      if(pingPI() == true){
        nextState = Stop1;
        stopRobot();
        FSMTimer = 1500;
      }else{
        nextState = Seek;
        FSMTimer = 250;
      }
      break;
    case Stop1:
      if(FSMTimer <= 0){
        pingPI();
        nextState = Ping2;
      }      
      break;
    case Ping2:
      if(firstLoop){
        FSMTimer = 1000;
        firstLoop = false;
      }else if(FSMTimer <= 0){
        readFromPI();
        firstLoop = true;
        nextState = Turn;
      }    
      break;
    case Turn:
      if(firstLoop){
        moveRobot(0, tempAngle, 2000);
        firstLoop = false;
      }else if(FSMTimer <= 0){
        stopRobot();
        nextState = Stop2;
        firstLoop = true;
      }      
      break;
    case Stop2:
      if(firstLoop){
        stopRobot();
        FSMTimer = 250;
        firstLoop = false;
      }else if(FSMTimer <= 0){
        nextState = Forward;
        firstLoop = true;
      }      
      break;
    case Forward:
      if(firstLoop){
        moveRobot(tempDist, 0, 10000);
        firstLoop = false;
      }else if( (rhoErr <= 0.5) && (!insideThresh) ){
        insideThresh = true;
        FSMTimer = 1000;
      }else if(FSMTimer <= 0){
        nextState = Stop3;
        firstLoop = true;
        insideThresh = false;
      }      
      break;
    case Stop3:
      if(firstLoop){
        stopRobot();
        FSMTimer = 500;
        firstLoop = false;
      }else if(FSMTimer <= 0){
        nextState = Turn90;
        firstLoop = true;
      }
      break;
    case Turn90:
      if(firstLoop){
        moveRobot(0, (-0.55*PI), 1500);
        firstLoop = false;
      }else if(FSMTimer <= 0){
        nextState = Stop4;
        firstLoop = true;
      } 
      break;
    case Stop4:
      if(firstLoop){
        stopRobot();
        FSMTimer = 500;
        firstLoop = false;
      }else if(FSMTimer <= 0){
        nextState = Circle;
        firstLoop = true;
      }
      break;
    case Circle:
      if(firstLoop){
        circleRobot(1.25, 3750);
        firstLoop = false;
      }else if(FSMTimer <= 0){
        nextState = Done;
        firstLoop = true;      
      } 
      break;
    case Done:
      stopRobot();
      break;
    default:
      break;
  }
  state = nextState;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// - - - - - - - - - - - - - - - - - - - - - - Controlers - - - - - - - - - - - -  - - - - - - - - - - -    
  if(!contVel){
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
  }

    
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
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  
  // Prints out various position/velocity/PWM values
  Serial.print(startTime / 1000.0);
  Serial.print("\t temp First: ");
  Serial.print(tempDummy);
  Serial.print("\t temp Distance: ");
  Serial.print(tempDist);
  Serial.print("\t temp Angle: ");
  Serial.print(tempAngle);
  Serial.print("\t Read Count: ");
  Serial.print(count);
  Serial.print("\t State: ");
  Serial.print(state);
  Serial.print("\t Timer: ");
  Serial.print(FSMTimer);
  Serial.print("\n");

  // Timing  
  endTime = millis();
  if( timeDelay - (endTime - startTime) <= 0){
    Serial.print("ERROR!");
  }else{
    delay(timeDelay - (endTime - startTime));
  }
  
  // Movement Timer
  if(FSMTimer > 0){
    FSMTimer -= 10;
  }
}
