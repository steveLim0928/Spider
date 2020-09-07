#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <String.h>


#define servoMinPWM 100
#define servoMaxPwM 495

#define leftTopTibia 0
#define leftMidTibia 2
#define leftBotTibia 4
#define rightTopTibia 6
#define rightMidTibia 12
#define rightBotTibia 10

#define leftTopFemur 1
#define leftMidFemur 3
#define leftBotFemur 5
#define rightTopFemur 7
#define rightMidFemur 9
#define rightBotFemur 11

#define tibiaLen 45
#define femurLen 35
#define tibiaFemurBase 30
#define topTibiaFemur 60

#define heightMax 40
#define heightMin 5

double prevHeightVal = 0;
double prevXVal = 0;

double filteredHeight;
double filteredX;

// RF Stuff
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN 10

const byte thisSlaveAddress[5] = {'S','p','1','d','E'};

RF24 radio(CE_PIN, CSN_PIN);

char dataReceived[32]; // this must match dataToSend in the TX
bool newData = false;
// RF Ends


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//====================

int degToPWM(double degree) {
  int temp;
  temp = map(degree, 0, 180, servoMaxPwM, servoMinPWM);
  return temp;
}

//====================

void kinematics(double *tibia, double *femur, double x, double z) {

  *femur = atan(x/topTibiaFemur);
  *femur = (*femur*180)/3.14159265;

  double temp;

  temp = tibiaLen - z;
  temp = tibiaFemurBase - temp;

  *tibia = asin(temp/femurLen);
  *tibia = (*tibia*180)/3.14159265;
//  Serial.print(*femur);
//  Serial.print(" , ");
//  Serial.println(*tibia);
}

//====================

double filter(double current, double prev, double filterStrength) {
  return (current + (prev * filterStrength)) / (filterStrength + 1);
}

//====================

void getData() {
    if ( radio.available() ) {
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
        Serial.println("Data received ");
    }
}

//====================

void setup() {

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  Serial.begin(9600);
  Serial.println("Start");

  int num;
  for (num = 0; num < 13; num++) {
    pwm.setPWM(num, 0, 297);
  }

  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();

  delay(10);
}

//====================

void loop() {

  getData();

  double heightIn = 0, xIn = 0; // stick input

  String input(dataReceived);

  String temp, temp2;

  input = dataReceived;

  temp = input.substring(0, 4);
  heightIn = atof(&temp[0]);
  temp = input.substring(4, 8);
  xIn = atof(&temp[0]);
  

  filteredHeight = filter(heightIn, prevHeightVal, 10);
  filteredX = filter(xIn, prevXVal, 10);

  prevHeightVal = filteredHeight;
  prevXVal = filteredX;

  Serial.print(" , ");
  Serial.print(filteredHeight);
  Serial.print(" , ");
  Serial.println(filteredX);

  xIn = map(filteredX, 0, 1024, -20, 20);
  heightIn = map(filteredHeight, 0, 1024, 1, 31);

  double tibia, femur;
  kinematics(&tibia, &femur, xIn, heightIn);

  int pulseH;
  pulseH = degToPWM(90 + tibia);

  int pulseXLeft;
  pulseXLeft = degToPWM(90 - femur);
  int pulseXRight;
  pulseXRight = degToPWM(90 + femur);
  //Serial.println(pulse);

  int num;
  for (num = 0; num < 13; num+= 2) {
    pwm.setPWM(num, 0, pulseH);
  }
  for (num = 1; num < 6; num+= 2) {
    pwm.setPWM(num, 0, pulseXRight);
  }
  for (num = 7; num < 13; num+= 2) {
    pwm.setPWM(num, 0, pulseXLeft);
  }
  //delay(10);

}