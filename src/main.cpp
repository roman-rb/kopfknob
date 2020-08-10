#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <SPI.h>

//this is a test to see if the branch is separate

//comment out for production:
// #define USE_SERIAL
// #define SERIAL_INPUT_DEBUG
// #define TEST_ENCODER_TIME //tests how long the encoder knob outputs stay, and outputs them to the terminal

//define parameters for OLED Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//pins for H-Bridge Driver
#define MOT_ENABLE 6
#define MOT_1A 4
#define MOT_2A 5

//pins for Encoder Knob
#define KNOB_SW 7
#define KNOB_DT 8
#define KNOB_CLK 9

//pins for Encoder Feedback
#define ENC_A 2
#define ENC_B 3

// Motor Resolution (DFMotor 75 RPM): 
// 1470 (rising edge, 1 channel)
// 2940 (rising/falling, 1 channel)
// 5880 (rising/falling, 2 channels)
#define MOTOR_RES 1470 


float targetSpeed = 0.25; //Hz
float pausedSpeedStored = 0; //target speed to be stored here and returned to after the pause
#define TARGET_INCREMENT 0.01 //how much to change targetSpeed per encoder tick

uint8_t motorOutput = 255; //value of analogwrite command sent to EN pin of H-Bridge
bool motDir = 1; //1 is CW, 0 is CCW

#define CONTROL_PERIOD 100 //period for control cycle in milliseconds
uint32_t prevControlTime = 0; //time since control loop was last run (milliseconds)
float kP = 7;
float kI = 12;
float kD = 0;


float prevError = 0; //previous error (for derivative control)
float intError = 0; //integral of error (for integral control)

uint32_t motorPos = 0; //motor position in pulses
uint32_t motorPos_old = 0; //motor position at the previous control loop (for calculating angular velocity)

bool knobFlag = 0; //prevent knob doublecounting
bool pauseFlag = 0; //flag to pause the motor
uint32_t lastPause = 0;
uint32_t prevKnobTime = 0;
#define DEBOUNCE_DELAY 300 //time in ms to ignore knob inputs (for signal debouncing)

void EncAFalling() { //count the motor pulse pin up by 1
  motorPos += 1;
}

void setup() {
  // put your setup code here, to run once:

  //input pin initializations
  pinMode(KNOB_SW,INPUT_PULLUP);
  pinMode(KNOB_DT,INPUT);
  pinMode(KNOB_CLK,INPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  
  pinMode(LED_BUILTIN, OUTPUT);

  //output pin initializations
  pinMode(MOT_ENABLE, OUTPUT);
  pinMode(MOT_1A, OUTPUT);
  pinMode(MOT_2A, OUTPUT);

  //initialize serial monitor
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(500);
  display.clearDisplay();
  display.setTextColor(WHITE);

  #ifdef USE_SERIAL
    Serial.begin(115200);
  #endif

  // digitalWrite(MOT_1A, LOW);
  // digitalWrite(MOT_2A, HIGH);
  // analogWrite(MOT_ENABLE, 128);

  attachInterrupt(digitalPinToInterrupt(ENC_A), EncAFalling, FALLING);

// //test display
//   display.setTextSize(2);
//   display.setTextColor(WHITE);
//   display.setCursor(0, 10);
//   // Display static text
//   display.println("Hello, world!");
//   display.setTextSize(1);
//   display.println("Made By Roman Boychuk");
//   display.println("roman@boych.uk");
//   display.display(); 
//   display.setTextSize(1);
//   display.println("Covered up?");
//   display.display();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);

  display.println("Made By:");
  display.println("Roman");
  display.println("Boychuk");
  display.setTextSize(1);
  display.println("roman@boych.uk");
  display.display();

  delay(3000);
}




void loop() {
  // put your main code here, to run repeatedly:

#if defined(TEST_ENCODER_TIME) && defined(USE_SERIAL) //testing micros between encoder pulses
while(digitalRead(ENC_A) == 1); //holding loop until CLK goes low
long CLKLOW = micros();
while(digitalRead(ENC_A) == 0); //wait till CLK goes high again
long CLKHIGH = micros();
// Serial.print(/n);
Serial.println(CLKHIGH - CLKLOW); //report time difference
#endif


#if defined(USE_SERIAL) && defined(SERIAL_INPUT_DEBUG) //initial debugging with Serial. Will slow down a lot during full speed testing

  Serial.print(digitalRead(KNOB_SW));
  Serial.print(digitalRead(KNOB_CLK));
  Serial.print(digitalRead(KNOB_DT));
  Serial.print(digitalRead(ENC_A));
  Serial.print(digitalRead(ENC_B));
  Serial.print("  ");

  Serial.println(motorPos);
#endif

if ((prevControlTime + CONTROL_PERIOD) < millis()){ //only run control loop after enough time has passed (approximately)
//control code goes here

int pulses = motorPos - motorPos_old; //calculate how many pulses have elapsed
motorPos_old = motorPos; //reset motorPos_old for next loop

float motorSpeed = float(pulses)/float(millis() - prevControlTime) * 1000 / MOTOR_RES; //motor speed in revolutions/second
if(targetSpeed<0) motorSpeed *= -1; //invert motor speed if target direction is CCW

float error = targetSpeed - motorSpeed; //calculate instantanous error
float derivError = (error-prevError) / (float(CONTROL_PERIOD)/1000); //calculate derivative error
intError += error*(float(CONTROL_PERIOD)/1000); //accumulate integral error

float motorVoltage = kP * error + kI * intError + kD * derivError; //calculate the motor voltage



if (motorVoltage<0) motorVoltage *= -1;


if (motorVoltage>5){ //output saturation
  motorOutput = 255; 
} else if(motorVoltage>0){
  motorOutput = motorVoltage*51;
} else {
  motorOutput = 0;
}





// if (motorVoltage > 0){ //scale normally within +/-5V
//   if(motorVoltage < 0) {
//     motorVoltage *= -1; //flip motor voltage before multiplyingg to get motor output to avoid overflow issues
//     CCWRotation = 1; //set CCW rotation flag
//   }
//   motorOutput = motorVoltage*51; //scales value from 0-5 to 0-255;
// }


//motor direction handling
if(targetSpeed > 0){ // set H bridge to move CW
  digitalWrite(MOT_1A, LOW);
  digitalWrite(MOT_2A, HIGH);
} else { // set H bridge to move CCW 
  digitalWrite(MOT_1A, HIGH);
  digitalWrite(MOT_2A, LOW);
}

if(pauseFlag) motorOutput = 0; //set motor output to 0 if motor is paused

// if (motorVoltage>5){ //clockwise rotation desired
//   motorOutput = 255;
// } else if (motorVoltage > 0){
//   motorOutput = motorVoltage*51; //scales value from 0-5 to 0-255;
// } else {
//   motorOutput = 0;
// }

prevControlTime = millis(); //set previous control time to current time for next iteration

analogWrite(MOT_ENABLE, motorOutput);

//update screen with status
display.clearDisplay();
display.setCursor(0,0);
display.setTextSize(2);
display.println("Rev/s:");
display.println(targetSpeed);


display.setTextSize(2);
if(pauseFlag) {
  display.println("Paused");
} else {
  display.println("");
} 

if(motorOutput == 255) display.println("Slow Down!");


//display.println("Made by roman@boych.uk"); //advertising ;)

display.display();

}





if ((digitalRead(KNOB_CLK) == 0) && !knobFlag) {
  if (digitalRead(KNOB_DT)) targetSpeed += TARGET_INCREMENT;
  else targetSpeed -= TARGET_INCREMENT;  
  knobFlag = 1; //set flag so that this code only runs once per falling edge of CLK
  if(targetSpeed == 0){ //reset the accumulated errors if user crosses zero point
    intError = 0;
    prevError = 0;
  }
}

if(pauseFlag == 1){
  targetSpeed = 0; //automatically will reset the target speed to 0 to prevent knob rotation after pause having any effect
}


if((knobFlag == 1) && (digitalRead(KNOB_CLK) ==1) &&(digitalRead(KNOB_SW)==1) && ((prevKnobTime+DEBOUNCE_DELAY)<millis())) knobFlag = 0; //reset flag once CLK goes back to high


// //motor direction handling
// if((targetSpeed > 0) && digitalRead(MOT_1A == 1)){ //if the target speed is + but the motor is turning in the negative direction
//   digitalWrite(MOT_1A, LOW);
//   digitalWrite(MOT_2A, HIGH);
// } else if((targetSpeed < 0) && digitalRead(MOT_1A == 0)){ //if the target speed is - but the motor is turning in the + direction
//   digitalWrite(MOT_1A, HIGH);
//   digitalWrite(MOT_2A, LOW);
// }



if((digitalRead(KNOB_SW) == 0) && ((lastPause + 500)<millis())){ //triggers only when the knob is pressed

prevKnobTime = millis(); //set the current knob time for debouncing
lastPause = millis();

  if(pauseFlag == 0){// system is being paused
    pausedSpeedStored = targetSpeed; //store target speed
    targetSpeed = 0; //set speed to 0
    pauseFlag = 1; //set pause flag 
  } else {
    targetSpeed = pausedSpeedStored; //restore saved speed
    pauseFlag = 0; //reset pause flag
  }
  
}

}