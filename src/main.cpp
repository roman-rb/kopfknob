#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <SPI.h>

//this is a test to see if the branch is separate


//define parameters for OLED Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//define parameters for Stepper Motor (28BYJ-48)
#define HALFSTEP 8
#define stepPin 4
#define dirPin 5
// #define motorPin3 6
// #define motorPin4 7

// #define USE_SERIAL 1

#define STEPS_PER_REV 3200

AccelStepper motor(AccelStepper::DRIVER, stepPin, dirPin); //initialize motor (pins 2 and 3 need to be inverted for 29BYJ motors)

float motorSpeed = 0.25; //motor speed in revolutions per second (default 0.5 Hz)
bool updateFlag = 1;


void increaseSpeed() {
  motorSpeed += 0.01;
  updateFlag = true;
  #ifdef USE_SERIAL
  Serial.println(motorSpeed);
  #endif
}

void decreaseSpeed() {
  motorSpeed -= 0.01;
  updateFlag = true;
  #ifdef USE_SERIAL
  Serial.println(motorSpeed);
  #endif
}

void setup() {
  // put your setup code here, to run once:

  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(2), increaseSpeed, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), decreaseSpeed, FALLING);

  motor.setMaxSpeed(2500);
  motor.setAcceleration(10);

#ifdef USE_SERIAL
  Serial.begin(9600);
#endif

  // if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
  //   //Serial.println(F("SSD1306 allocation failed"));
  //   for(;;);
  // }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  delay(500);
  display.clearDisplay();
  display.setTextColor(WHITE);

  // display.setTextSize(2);
  // display.setTextColor(WHITE);
  // display.setCursor(0, 10);
  // // Display static text
  // display.println("Hello, world!");
  // display.setTextSize(1);
  // display.println("Made By Roman Boychuk");
  // display.println("roman@boych.uk");
  // display.display(); 
  // display.setTextSize(1);
  // display.println("Covered up?");
  // display.display();


}

void loop() {
  // put your main code here, to run repeatedly:

  motor.setSpeed(motorSpeed * STEPS_PER_REV);
  motor.run();

  if(updateFlag){

    display.clearDisplay();
    display.setTextColor(WHITE);

    //update motor speed:
    display.setTextSize(2);
    display.setCursor(0,10);
    display.print("Speed:");
    display.println(motorSpeed);

    //advertising ;)
    display.setTextSize(1);
    display.println("Made By Roman Boychuk");
    display.println("roman@boych.uk");

    display.display();

    updateFlag = 0;
  }



#ifdef USE_SERIAL //initial debugging with Serial. Will slow down a lot during full speed testing
  // if (digitalRead(9)==0)
  // {
  //   Serial.println("LEFT BTN");
  // }

  // if (digitalRead(10)==0)
  // {
  //   Serial.println("RIGHT BTN");
  // }
  // delay(10);

  Serial.println(updateFlag);

#endif


}
