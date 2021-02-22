#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
byte data[32];
int i = 0;
byte temp = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output
// initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

// define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount){

  i = 0;
  Serial.print("Data Recieved: ");
  
  while(Wire.available()) {
    data[i] = Wire.read();
    Serial.print(data[i]);
    Serial.print(' ');
    i++;
    
  }

  if(temp == 0){
    number = i;
    temp = 1;
  }
  
  Serial.print(' ');
}

// callback for sending data
void sendData(){

 
  for(int j=1; j < number - j; j++){
    temp = data[j];
    data[j] = data[number - j];
    data[number - j] = temp;
  }

  for(int j=1; j < 32; j++){
    Wire.write(data[j]);
  }
    Wire.write(0);
    temp = 0;
  
}
