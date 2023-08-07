#include <Keypad.h>
#include <math.h>
#define CLOCK_FREQUENCY 16000000
#define SERIAL_BAUDRATE 57600
// encoder pins
#define ENCODER_A 2
#define ENCODER_B 3
// motor pins
#define MOTOR_ENABLE 11
#define DIR 13
#define BRAKE 8
#define MAX_PWM 255
#define MIN_PWM 55 // this one depends on the dead zone of the motor input voltage
#define PPR 813  // 1080 is ideally, but there may exists some offset from your observation and sensor noise 

// interpolated P-control
#define KP 0.12  // P control parameter
#define TARGET_DIST 100 // pulses
#define DIS2GO 5
int dist_moved = 0;
int controlLoopRate = 95;
int distanceToGo;    
int positionError=0;
int PWM_value;

const float dial_ticks = 60; // dial padlock resolution

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

/* KeyPad parameters */
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
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

/* FTS parameters */
byte OMD, lastOMD; // used to save the operation mode
volatile unsigned int MS, lastMS;
bool ERR1; // a flag for error level 1 
bool FFRA; // a flag for the first running
char machineMessage[25], menuTitle[25], action1[25], action2[25], action3[25], action4[25];
int dial, last_dial; // dial position

/* Functions of Motor Control */
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
  motor_speed = max(motor_speed, MIN_PWM);  // lower bound
  analogWrite(MOTOR_ENABLE, motor_speed);
}

void stopMotor(){
  digitalWrite(BRAKE, HIGH);   //Engage the Brake for Channel A
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
  interrupts();
}
// convert delta dial distance to pulses
int dialDistanceToPulses(int deltaDials){
  int delta_pulses;
  delta_pulses = deltaDials/dial_ticks*PPR;
  
  return delta_pulses;
}

// ISR for encoder reading: Channel A
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
// ISR for encoder reading: Channel B
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

// Timer Setup: frequency needs to be larger than 1 HZ
void setupTimer(int freq){
  // set up Timer
  noInterrupts(); // stop interrupts
  
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  
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

// parameter is the distance in pulses we want the motor to drive to
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
    if((encoderPos - distInPulses)*dir_sign >= -1){
      stopMotor();
      break;
    }
  }
}
// parameter is the distance in ticks we want the motor to drive to
void driveTicks(float ticks){
  int pulses = dialDistanceToPulses(ticks);
  drivePulses(pulses);
}

/* move CCW to a dial position */
// input: dial position combo it will go to;
// output: the motor will rotate to that number CCW;
// calculate the incremental ticks it needs to go and use function 'driveTicks(ticks)' to implement
void moveCcwToDialNO(int combo)  {
driveTicks(combo);
}

/* move CW to a dial position */
// input: dial position combo it will go to;
// output: the motor will rotate to that number CW;
// calculate the incremental ticks it needs to go and use function 'driveTicks(ticks)' to implement
void moveCwToDialNO(int combo)  {
driveTicks(combo);
}

/* Functions of Free Time System  */ 
// FTS initialization function
void INZfunction() {
  // initailize parameters
  OMD = 0;
  lastOMD = 0;
  lastMS = 0;
  ERR1 = false;
  FFRA = true;
}

// Digonosis function
int DIGfunction() {
  // put the diagnostics program here
  if (ERR1) {
    return 0; // for error
  } else {
    return 1; // successful diagnostics
  }
}

// set idle menu
void setIdleMenu(){
  String str1, str2, str3, str4, str5;
  str1 = "OPERATION MENU";
  str2 = "1. Manual Operation";
  str3 = "2. Automatic Operation";
  str4 = "3. Machine Data Set-Up";
  str5 = "5. Exit";
  str1.toCharArray(menuTitle,25);
  str2.toCharArray(action1, 25);
  str3.toCharArray(action2, 25);
  str4.toCharArray(action3, 25);
  str5.toCharArray(action4, 25);
}

// set Manual Menu
void setManualMenu(){
  String str1, str2, str3, str4, str5;
  str1 = "MANUAL OPERATION MENU";
  str2 = "1. Move One Tick CW";
  str3 = "2. Move One Tick CCW";
  str4 = " ";
  str5 = "5. Exit";
  str1.toCharArray(menuTitle,25);
  str2.toCharArray(action1, 25);
  str3.toCharArray(action2, 25);
  str4.toCharArray(action3, 25);
  str5.toCharArray(action4, 25);
}
// set Automatic menu
void setAutomaticMenu(){
  String str1, str2, str3, str4, str5;
  // fill str1~5 to have the Automatic manaul print out correctly
  str1 = "AUTOMATIC OPERATION MENU ";
  str2 = "";
  str3 = "";
  str4 = " ";
  str5 = "5. Exit ";
  str1.toCharArray(menuTitle,25);
  str2.toCharArray(action1, 25);
  str3.toCharArray(action2, 25);
  str4.toCharArray(action3, 25);
  str5.toCharArray(action4, 25);
}
// set Machine setup menu
void setSetupMenu(){
  String str1, str2, str3, str4, str5;
  str1 = "MACHINE SETUP MENU";
  str2 = " ";
  str3 = " ";
  str4 = " ";
  str5 = " ";
  str1.toCharArray(menuTitle,25);
  str2.toCharArray(action1, 25);
  str3.toCharArray(action2, 25);
  str4.toCharArray(action3, 25);
  str5.toCharArray(action4, 25);
}

// get input digits from the KeyPad
int getTwoCharDigits(char *arr){
  char key = customKeypad.getKey();
  int dialPosition;
  while(!key){
    key = customKeypad.getKey();
  }
  arr[0] = key;
  // Serial.println(customKey);
  key = customKeypad.getKey();
  
  while(!key){
    key = customKeypad.getKey();
  }
  // get zero position
  arr[1] = key;
  key = customKeypad.getKey();
  dialPosition = atoi(arr);

  return dialPosition;
}

// Machine setup function
void MSDfunction(){
  char numbers[2];
  char key;
  int dialPos, inputFlag;
  inputFlag = 1;
  Serial.println("Input a value for the initial dial position");
  dialPos = getTwoCharDigits(numbers);
  while(inputFlag) {
    if (dialPos >= 0 && dialPos <= dial_ticks) {
      //Set Dial position if it is an appropriate option
      dial = dialPos;
      Serial.print("Dial Position Set to ");Serial.println(dial);
      //Give option to leave program or not
      inputFlag = 0;
    }
    else{
      Serial.print("Invalid Number Input: ");Serial.println(dialPos);
      Serial.print("Input should be from 0-60\n");
      Serial.print("Input a value for the initial dial position\n");
      dialPos = getTwoCharDigits(numbers);
    }
  }

  //no longer in manual mode
  Serial.print("Press # to Exit\n");
  key = customKeypad.getKey();
  while(key!='#'){
    key = customKeypad.getKey();
  };
  setIdleMenu();
  printHeaderAndMenu();
  OMD = 0;
}

// Mode print functions
char ModePrint(){
  switch (OMD){
    case 1:
      Serial.println("Manual");
      break;
    case 2:
      Serial.println("Automatic");
      break;
    case 3:
      Serial.println("Machine Setup");
      break;
    case 5:
      Serial.println("System Turned Off");
      break;
    default:
      Serial.println("Idle Mode");
    }
}

// Output Control Supervisor
void OCSfunction(){
  if(FFRA) {
    FFRA = false;//Clear first run flag
    setSetupMenu();
    printHeaderAndMenu();
    MSDfunction();
  }
  else if (lastOMD != OMD){
    lastOMD = OMD;
    Serial.println(machineMessage);
    Serial.print("Current Mode: "); 
    ModePrint();
    Serial.print("Dial Position is at: ");
    Serial.println(dial);
  }

}

// MSS: Machine Status Scan Supervisor
void MSSfunction() {
  String str;
  //skip this for the first run
  if (FFRA) OCSfunction();
  // scan the keypad to get the input
  char selection = customKeypad.getKey(); // get a key from the keypad
  if (selection) {
    switch (selection) {
      // manual operation mode
      case '1':
        if (OMD == 1){ //in manual mode
          MOSfunction(selection);
        }
        else if (OMD == 0){//in idle mode
          OMD = 1;
          setManualMenu();
          str = "Manual Mode Accepted";
          str.toCharArray(machineMessage,25);
          printHeaderAndMenu();
        }
        break;
      // automatic operation mode 
      case '2': //this key will...
        if (OMD == 1){ //in manual mode
          MOSfunction(selection);
        }
        else if (OMD == 0){//in idle mode
          OMD = 2;
          setAutomaticMenu();
          printHeaderAndMenu();
          str = "Automatic Mode Accepted";
          str.toCharArray(machineMessage,25);
        }
        break;
      case '3': //this key will...
        if (OMD == 1){ //in manual mode
          MOSfunction(selection);
        }
        else if (OMD == 0){//in idle mode
          OMD = 3;
          setSetupMenu();
          printHeaderAndMenu();
        }
        break;
      // Exit
      case '5':
        if (OMD == 0){
          OMD = 5;
          str = "Automatic Mode Accepted";
          str.toCharArray(machineMessage,25);
        }
        else {
          setIdleMenu();
          printHeaderAndMenu();
          OMD = 0;
        }
      default:
        break;
    }
  }
}

// MCS: Mode Control Supervisor 
void MCSfunction(){
  if (OMD==1) MOSfunction(0); // do nothing
  if (OMD==2) ACSfunction();  
  if (OMD==3) MSDfunction();
}

// automatic lock opener
void ACSfunction(){
  int combo1 = -1, combo2 = -1, combo3 = -1, task = 0;
  char number[2];
  int number1 = 0;
  int number2 = 0;
  int number3 = 0;
  char key;
  String str;
  while(OMD ==2) {
    switch(task){
    // task 0
    case 0:
      // prompt to enter the first combination number combo1 and update machine message
      Serial.println ("Enter Combination Numbers");

      task=1;
      
    case 1:
      // wait for input of combo1 with function: getTwoCharDigits(number)
      Serial.println ("Enter First Combination Number:");
      combo1 = getTwoCharDigits(number);
      number1 = atoi(number);
      Serial.println (number1);
     
      if (number1 > 0 && number1 <= 60){
        task = 2;
      }
      else {
        Serial.println ("Input should be from 0-60\n");
        //Serial.println (number1);
      }
      break;
     
      // check the number input if correct or not, reject the number over the range
      // if it over the range, prompt the user to input it again
      // if it is in the range, prompt to input the second number combo2, go to task 2

    case 2:
      // wait for input of combo2 with function: getTwoCharDigits(number)
      Serial.println ("Enter Second Combination Number:");
      combo2 = getTwoCharDigits(number);
      number2 = atoi(number);
      Serial.println (number2);
      // check the number input if correct or not, reject the number over the range
      // if it over the range, prompt the user to input it again
      // if it is in the range, prompt to input the third number combo3, go to task 3
      
      if (number2 > 0 && number2 <= 60){
        task = 3;
      }
      else {
        Serial.println ("Input should be from 0-60\n");
      }
      break;
      
    case 3:
      // wait for input of combo3 with function: getTwoCharDigits(number)
      Serial.println ("Enter Third Combination Number:");
      combo3 = getTwoCharDigits(number);
      number3 = atoi(number);
      Serial.println (number3);
      // check the number input if correct or not, reject the number over the range
      // if it over the range, prompt the user to input it again
      // if it is in the range, go to task 4
      
      if (number3 > 0 && number3 <= 60){
        task = 4;
      }
      else {
        Serial.println ("Input should be from 0-60\n");
        
        //Serial.println (number3);
      }
      break;
    
    case 4:  
      //unlock stuff
      //prompt the user to open the padlock by pressing '#'
      Serial.println ("");
      Serial.println ("The Numbers You Entered");
      Serial.print (number1);
      Serial.print (",");
      Serial.print (number2);
      Serial.print (",");
      Serial.print (number3);
      Serial.print (",");
      Serial.println ("");
      // the motor will start moving (lock opening), after the user press '#'
      Serial.println ("Press # to Unlock");
      key = customKeypad.getKey();
       while(key!='#'){
       key = customKeypad.getKey();
       };

      //int dialPos;
      //Move to first location
      Serial.print("Moving to: "); Serial.println(combo1);
      // move 1 circle CW with function moveCwToDialNO()
      moveCwToDialNO(60);
      delay(250);
      // move 1 circle CW with function moveCwToDialNO()
      moveCwToDialNO(60);
      delay(250);
      // move to the 1st number CW with function moveCwToDialNO()
      if (combo1 > dial) {
        moveCwToDialNO(combo1-dial);
      }
      else {
        moveCwToDialNO(combo1-dial+60);
      }
      delay(1000);
      //Move to second number
     
      Serial.print("Moving to: "); Serial.println(combo2);
      // move 1 circle CCW with function moveCcwToDialNO()
      moveCcwToDialNO(-60);
      delay(250);
      // move to the 2nd number CCW with function moveCcwToDialNO()
      
      if (combo2 < combo1) {
        moveCwToDialNO(combo2 - combo1);
      }
      else {
        moveCwToDialNO(-60 + combo2 - combo1);
      }
      delay(1000);
      // Move to third number
      
      Serial.print("Moving to: "); Serial.println(combo3);
      
      // move to the 3rd number CW with function moveCwToDialNO()
      if (combo3 > combo2) {
        moveCwToDialNO(combo3 - combo2);
      }
      else {
        moveCwToDialNO(combo3-combo2+60);
      }
      delay(1000);
      
      //pop the lock with the solenoid connected to port 1
      Serial.println("You can pull the shackle now!");

      // update machine message
      Serial.println ("Press # to go back to main menu");
      // prompt the user to input '#' to go back the main menu
       key = customKeypad.getKey();
       while(key!='#'){
       key = customKeypad.getKey();
       };
      // go back to the idle mode
      setIdleMenu();
      printHeaderAndMenu();
      OMD = 0;

    }
  }
}

// MOS: Manual Operation Supervisor
void MOSfunction(char selection) {
  String str;
  if (selection){
    str = "Manual Idle Mode";
    str.toCharArray(machineMessage,25);
    switch (selection){
      case '1': 
        str = "Manual Active Mode";
        str.toCharArray(machineMessage,25);
        driveTicks(1); //move 1 tick CW decrease
        dial++;
        if (dial == dial_ticks) dial = 0;
        break;
      case '2': 
        str = "Manual Active Mode";
        str.toCharArray(machineMessage,25);
        driveTicks(-1); //zero is CCW
        dial--;
        if (dial ==-1) dial = dial_ticks-1;
        break;
    }
  }
}

void printHeaderAndMenu(){
  Serial.println("EME-154 Mechatronics");
  Serial.println("Free Time System");
  Serial.println("Eason Chen");
  Serial.println("****************************************");
  Serial.print("            ");Serial.println(menuTitle);
  Serial.println("****************************************");
  Serial.println(action1);
  Serial.println(action2);
  Serial.println(action3);
  Serial.println(action4);
  // delay(10); // wait for printing
}
// ERT
void ERTfunction() {
  // take actions here to treat detected errors
}

void setup(){
  Serial.begin(SERIAL_BAUDRATE);
  // pin mode for encoder A, B
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  // set up interrupt for Pin A, B
  attachInterrupt(0, doEncoderA, CHANGE); 
  attachInterrupt(1, doEncoderB, CHANGE); 
  //Motor pins
  pinMode(MOTOR_ENABLE, OUTPUT); // speed with PWM output
  pinMode(DIR, OUTPUT);   // direction pin
  pinMode(BRAKE, OUTPUT);   // brake pin
  digitalWrite(DIR, LOW); //Establishes forward direction of Channel A
  digitalWrite(BRAKE, LOW);   //Disengage the Brake for Channel A
  // setup timer: given frequency for the motor controling
  setupTimer(controlLoopRate);

  // Initialization of FTS
  INZfunction();
}

void loop(){
  // the main FTS
  if (DIGfunction()){
    // if diagnostics succeed
    MSSfunction();
    MCSfunction();
    OCSfunction();
  } else {
    // go to ERT if diagnostics fail
    ERTfunction();
    OCSfunction();
  }
  // if press the button 5, stop the system
  if (OMD == 5) {
    Serial.println("System Exit!!");
    while(1);
  }
} 
