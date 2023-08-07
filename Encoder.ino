#include <Keypad.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

// encoder pins
#define ENCODER_A 2
#define ENCODER_B 3
// motor pins
#define MOTOR_ENABLE 11
#define BRAKE 8
#define DIR 13

// direction sensing flags
volatile byte AfirstUp = 0; 
volatile byte BfirstUp = 0; 
volatile byte AfirstDown = 0; 
volatile byte BfirstDown = 0; 
// CW when it increases, CCW when it decreases
volatile int encoderPos = 0; //this variable stores our current value of encoder position. 
int oldEncPos = 0; //this variable stores our last value of encoder position for printing
volatile byte pinA_read = 0; // this variable is shared among interrupts
volatile byte pinB_read = 0; // this variable is shared among interrupts

//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {7, 6, 5, 4}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {A5, A4, A3, A2}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

void setMotorCW(){
  digitalWrite(DIR, LOW);
}

void setMotorCCW(){
  digitalWrite(DIR, HIGH);
}

void driveMotor(int motor_speed){
  motor_speed = min(motor_speed, 255);
  digitalWrite(BRAKE, LOW);
  analogWrite(MOTOR_ENABLE, motor_speed);
}

void stopMotor(){
  analogWrite(MOTOR_ENABLE, 0); 
  digitalWrite(BRAKE, HIGH);   //Disengage the Brake for Channel A

}

void doEncoderA(){
  pinA_read = digitalRead(ENCODER_A);
  pinB_read = digitalRead(ENCODER_B);
  // for rising edge
  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  if((pinA_read==1) && (pinB_read==1) && BfirstUp) { 
    encoderPos ++; //increment the encoder's position count
    AfirstUp = 0; //reset flags for the next turn
    BfirstUp = 0; //reset flags for the next turn
  }
  // Direction sensing
  else if ((pinA_read==1) && (pinB_read==0)) AfirstUp = 1; 

  // for falling edge
  //check that we have both pins at LOW 
  if((pinA_read==0) && (pinB_read==0) && BfirstDown) { 
    encoderPos ++; //increment the encoder's position count
    AfirstDown = 0; //reset flags for the next turn
    BfirstDown = 0; //reset flags for the next turn
  }
  // Direction sensing
  else if ((pinA_read==0) && (pinB_read==1)) AfirstDown = 1; 
}

void doEncoderB(){
  pinA_read = digitalRead(ENCODER_A);
  pinB_read = digitalRead(ENCODER_B);
  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
  if ((pinA_read==1) && (pinB_read==1) && AfirstUp) { 
    encoderPos --; //decrement the encoder's position count
    AfirstUp = 0; //reset flags for the next turn
    BfirstUp = 0; //reset flags for the next turn
  }
  // Direction sensing
  else if ((pinA_read==0) && (pinB_read==1)) BfirstUp = 1; 

  // for falling edge
  //check that we have both pins at LOW
  if((pinA_read==0) && (pinB_read==0) && AfirstDown) { 
    encoderPos --; //decrement the encoder's position count
    AfirstDown = 0; //reset flags for the next turn
    BfirstDown = 0; //reset flags for the next turn
  }
  // Direction sensing
  else if ((pinA_read==1) && (pinB_read==0)) BfirstDown = 1; 
}

void setup(){
  Serial.begin(9600);
  // pin mode for encoder A
  pinMode(ENCODER_A, INPUT);
  // pin mode for encoder B 
  pinMode(ENCODER_B, INPUT);
  // set up interrupt
  attachInterrupt(0, doEncoderA, CHANGE); 
  attachInterrupt(1, doEncoderB, CHANGE); 
  //Motor pins
  pinMode(MOTOR_ENABLE,OUTPUT); // speed with PWM output
  pinMode(DIR,OUTPUT);   // direction pin
  pinMode(BRAKE,OUTPUT);   // direction pin

  digitalWrite(DIR, LOW); //Establishes forward direction of Channel A
  digitalWrite(BRAKE, LOW);   //Disengage the Brake for Channel A
  Serial.print("Encoder position: ");Serial.println(encoderPos);

}
  
void loop(){
  char customKey = customKeypad.getKey();
  
  if (customKey=='A'){
    // set CW
    Serial.println("CW");
    setMotorCW();
    driveMotor(55);
  }
  if (customKey=='B'){
    // set CW
    Serial.println("CCW");
    setMotorCCW();
    driveMotor(55);
  }
  if (customKey=='C'){
    // stop the motor
    stopMotor();
    Serial.println("Stop");
  }
  // print out encoder position
  if(encoderPos!=oldEncPos){
    Serial.print("Encoder position: ");Serial.println(encoderPos);
    oldEncPos = encoderPos;
  }
}


 
