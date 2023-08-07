#include <Keypad.h>
#include <math.h>
#define CLOCK_FREQUENCY 16000000
#define SERIAL_BAUDRATE 9600
// encoder pins
#define ENCODER_A 2
#define ENCODER_B 3
// motor pins
#define MOTOR_ENABLE 11
#define DIR 13
#define BRAKE 8
#define MAX_PWM 255
#define MIN_PWM 52 // this one depends on the dead zone of the motor input voltage
// 1080 is ideally, but there may exists some offset from your observation and sensor noise 
#define PPR 1040

// interpolated P-control
#define KP 0.12 // P control parameter
#define TARGET_DIST 100 // pulses
#define DIS2GO 5
int dist_moved = 0;
int controlLoopRate = 95;

// dial padlock resolution
const float dial_ticks = 60;
// encoder reading flags
volatile byte AfirstUp = 0; 
volatile byte BfirstUp = 0; 
volatile byte AfirstDown = 0; 
volatile byte BfirstDown = 0; 
// CW when it increases, CCW when it decreases
volatile int encoderPos = 0; //this variable stores our current value of encoder position. 
volatile int lastEncoderPos = 0; //this variable stores our last value of encoder position. 
volatile byte pinA_read = 0; // this variable is shared among interrupts
volatile byte pinB_read = 0; // this variable is shared among interrupts

// machine setup 
char zeroPosInputs[2]; // input by the user 
int initialPosition; // converted by the user
int distanceToGo; // rotation pulses of the motor

// P control
int positionError=0;
int PWM_value;

//define the cymbols on the buttons of the keypads
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char hexaKeys[ROWS][COLS] = {
{'1','2','3','A'},
{'4','5','6','B'},
{'7','8','9','C'},
{'*','0','#','D'}
};
byte rowPins[ROWS] = {7, 6, 5, 4}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {A5, A4, A3, A2}; // connect to the column pinouts of the keypad
//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
char customKey; // char of keyPad input from the key

void setMotorCW(){
// direction set pin from the shield
digitalWrite(DIR, LOW);
}

void setMotorCCW(){
// direction set pin from the shield
digitalWrite(DIR, HIGH);
}

void driveMotor(int motor_speed){
digitalWrite(BRAKE, LOW); // release the brake before setting the speed
motor_speed = min(motor_speed, MAX_PWM); // upper bound
motor_speed = max(motor_speed, MIN_PWM); // lower bound
analogWrite(MOTOR_ENABLE, motor_speed);
}

void stopMotor(){
digitalWrite(BRAKE, HIGH); //Engage the Brake for Channel A
analogWrite(MOTOR_ENABLE, 0); 
}
// after each robotation of the motor, reset the control parameter
void resetMotorControl(int pulses){
noInterrupts();
encoderPos = 0;
lastEncoderPos = 0;
AfirstUp = 0; //reset flags for the next turn
BfirstUp = 0; //reset flags for the next turn
AfirstDown = 0; //reset flags for the next turn
BfirstDown = 0; //reset flags for the next turn
// clear position error
positionError = 0;
PWM_value = 0;
dist_moved = 0;
distanceToGo = pulses;
TCNT1 = 0; 
interrupts();
}
// ISR for encoder reading
void doEncoderA(){
pinA_read = digitalRead(ENCODER_A);
pinB_read = digitalRead(ENCODER_B);
// for rising edge
//check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
if((pinA_read==1) && (pinB_read==1) && BfirstUp) { 
encoderPos --; //decrement the encoder's position count
AfirstUp = 0; //reset flags for the next turn
BfirstUp = 0; //reset flags for the next turn
}
// Direction sensing
else if ((pinA_read==1) && (pinB_read==0)) AfirstUp = 1;

// for falling edge
//check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
if((pinA_read==0) && (pinB_read==0) && BfirstDown) { 
encoderPos --; //decrement the encoder's position count
AfirstDown = 0; //reset flags for the next turn
BfirstDown = 0; //reset flags for the next turn
}
// Direction sensing
else if ((pinA_read==0) && (pinB_read==1)) AfirstDown = 1; 
}

void doEncoderB(){
// reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
pinA_read = digitalRead(ENCODER_A);
pinB_read = digitalRead(ENCODER_B);
if ((pinA_read==1) && (pinB_read==1) && AfirstUp) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
encoderPos ++; //increment the encoder's position count
AfirstUp = 0; //reset flags for the next turn
BfirstUp = 0; //reset flags for the next turn
}
// Direction sensing
else if ((pinA_read==0) && (pinB_read==1)) BfirstUp = 1;

// for falling edge
//check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
if((pinA_read==0) && (pinB_read==0) && AfirstDown) { 
encoderPos ++; //decrement the encoder's position count
AfirstDown = 0; //reset flags for the next turn
BfirstDown = 0; //reset flags for the next turn
}
// Direction sensing
else if ((pinA_read==1) && (pinB_read==0)) BfirstDown = 1; 
}

// timer setup: frequency needs to be larger than 1 HZ
void setupTimer(int freq){
// set up Timer
noInterrupts(); // stop interrupts

TCCR1A = 0; // set entire TCCR1A register to 0
TCCR1B = 0; // same for TCCR1B
TCNT1 = 0; // initialize counter value to 0

// freq: 1~3
if(freq <= 3){ 
// set compare match register for freq(<=3) Hz increments
OCR1A = CLOCK_FREQUENCY/(256*freq)-1; 
// Set CS12, CS11 and CS10 bits for 256 prescaler
TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
}
// freq: 4~30
else if(freq <= 30){ 
// set compare match register for freq(<=30) Hz increments
OCR1A = CLOCK_FREQUENCY/(64*freq)-1; 
// Set CS12, CS11 and CS10 bits for 64 prescaler
TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
}

// freq: 31~244
else if(freq <= 244){ 
// set compare match register for freq(<=30) Hz increments
OCR1A = CLOCK_FREQUENCY/(8*freq)-1; 
// Set CS12, CS11 and CS10 bits for 8 prescaler
TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
}

// freq: >=245
else{ 
// set compare match register for freq(<=30) Hz increments
OCR1A = CLOCK_FREQUENCY/(1*freq)-1; 
// Set CS12, CS11 and CS10 bits for 1 prescaler
TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
}

// turn on CTC mode
TCCR1B |= (1 << WGM12);
// enable timer compare interrupt
TIMSK1 |= (1 << OCIE1A);

interrupts(); // allow interrupts
}

// convert delta dial distance to pulses
int dialDistanceToPulses(int deltaDials){
int delta_pulses;
delta_pulses = deltaDials/dial_ticks*PPR;

return delta_pulses;
}

// ISR of timer
ISR(TIMER1_COMPA_vect) {
// calculate the position error
if (abs(distanceToGo - encoderPos) < TARGET_DIST){
positionError = distanceToGo - encoderPos;
}
// positionError = set_distance - encoderPos;
else{
dist_moved = encoderPos - lastEncoderPos;
if(distanceToGo > 0){
positionError += DIS2GO - dist_moved;
}
else{
positionError += -DIS2GO - dist_moved; 
}
}
// positionError = distanceToGo - encoderPos;
PWM_value = (int)(KP*(float)positionError);
// update the last_encoder reading
lastEncoderPos = encoderPos;
}

void setup(){
Serial.begin(SERIAL_BAUDRATE);
// pin mode for encoder A
pinMode(ENCODER_A, INPUT);
// pin mode for encoder B 
pinMode(ENCODER_B, INPUT);
// set up interrupt
attachInterrupt(0, doEncoderA, CHANGE); 
attachInterrupt(1, doEncoderB, CHANGE); 
//Motor pins
pinMode(MOTOR_ENABLE, OUTPUT); // speed with PWM output
pinMode(DIR, OUTPUT); // direction pin
pinMode(BRAKE, OUTPUT); // brake pin
digitalWrite(DIR, LOW); //Establishes forward direction of Channel A
digitalWrite(BRAKE, LOW); //Disengage the Brake for Channel A
// setup timer: given frequency
setupTimer(controlLoopRate);
}


// parameter is the distance we want the motor to drive to
void drivePulses(int distInPulses){
// get the sign of running distInPulses
int dir_sign = (distInPulses > 0 ? 1 : -1);
resetMotorControl(distInPulses);
while(1)
{
if (PWM_value < 0){
setMotorCCW();
}
if (PWM_value > 0){
setMotorCW();
}
// setMotorCCW();
driveMotor(abs(PWM_value));
Serial.print("PWM_value: ");Serial.println(PWM_value);
Serial.print("positionError: ");Serial.println(positionError);
// stop the motor when we are at the desired position
// if(encoderPos - distInPulses <= 1){
if((encoderPos - distInPulses)*dir_sign >= -1){
stopMotor();
Serial.println("Arrived at desired position!");
break;
}
}
}

void loop(){
// machine setup
Serial.print("Please input the first digit: ");
while(!customKey){
customKey = customKeypad.getKey();
}
zeroPosInputs[0] = customKey;
Serial.println(customKey);
customKey = customKeypad.getKey();
Serial.print("Please input the second digit: ");

while(!customKey){
customKey = customKeypad.getKey();
}
// get zero position
zeroPosInputs[1] = customKey;
Serial.println(customKey);
customKey = customKeypad.getKey();
Serial.print("Zero position of dial: ");
initialPosition = atoi(zeroPosInputs);
Serial.println(initialPosition);

// drive the motor
// calculate the distance to run
// running distance from initial position to zero position 
int pulse_dist = dialDistanceToPulses(dial_ticks - initialPosition);
// P-control 
drivePulses(pulse_dist);
delay(1000); // it is essential for controller reseting
drivePulses(-PPR);
}
