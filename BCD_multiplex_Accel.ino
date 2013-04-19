/* BCD Multiplex counter, to be used for controlling parabolic mirror, reading accelerometer...
Interrupt coder for reading out Rotary encoder from:
http://playground.arduino.cc/Main/RotaryEncoders#Example2

I2C EEPROM Handling from/inspired by:
http://tronixstuff.wordpress.com/2010/10/29/tutorial-arduino-and-the-i2c-bus-part-two/

Accelerometer code and library from: http://www.seeedstudio.com/wiki/Grove_-_3-Axis_Digital_Accelerometer%28%C2%B11.5g%29


*/

#include "Wire.h"
//I2c Peripherals
//#define EEPROM1 0x50 // EEPROM device Address
//start accelerometer, include library
#include "MMA7660.h"
MMA7660 accelemeter;


const int DEBUG = 0;

//global Variables
int numberL; //Numbers on the 2 4x7 segment displays, left
int numberR; //Numbers on the 2 4x7 segment displays, right


//input pins

//rotary encoder code
#define encoder0PinA  2
#define encoder0PinB  3
volatile signed int encoder0Pos = 0;
//end rotary encoder code

int REPush = 22;
int Button1 = 24;
int Button2 = 26;

//output pins
int DataPins[4] = {8,9,10,11}; // A,B,C,D outputs to 74HC4511
int ClockPin = 4; //ClockPin of 74HC4017
int ResetPin = 5; //ResetPin of 74HC4017
//Additional stuff on 7Seg Displays:
int MinusPin = 6;
int DPPin = 7;
//LEDs
int LED = 13;
int LED1 = 28;
int LED2 = 30;





void setup() {

 if(DEBUG){
    Serial.begin(9600);
  }  
  
  //set output pins
  for(int a = 0; a < 4; a++){
    pinMode(DataPins[a], OUTPUT);
  }
  pinMode(ClockPin, OUTPUT);
  pinMode(ResetPin, OUTPUT);
  pinMode(MinusPin, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(DPPin, OUTPUT);
  pinMode(REPush, INPUT);
  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  

//rotary encoder code
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
//end rotary encoder code

accelemeter.init();

}

void loop() {

  int phiout;
//accelerometer  
  int8_t x;
  int8_t y;
  int8_t z;
  const float pi =3.1415927;
  float ax,ay,az;
  float anglX, anglY, anglZ;
  float theta, phi, psi;
  float thetaK, phiK, psiK;
  accelemeter.getXYZ(&x,&y,&z);
  accelemeter.getAcclemeter(&ax,&ay,&az);
  theta= (atan(2*(ay/(-1*sqrt(az*az+ax*ax))))/pi)*180;
  phi=   (atan(2*(az/(-1*sqrt(ay*ay+ax*ax))))/pi)*180;
  psi=   (atan(2*(ax/(-1*sqrt(ay*ay+az*az))))/pi)*180;
    if(DEBUG){
/*
      Serial.println("Accelerometer raw data");
      Serial.print("x = ");
      Serial.println(x);
      Serial.print("y = ");
      Serial.println(y);
      Serial.print("z = ");
      Serial.println(z);
      Serial.println("Acceleration, g");      
      Serial.print("ax = ");
      Serial.println(ax);
      Serial.print("ay = ");
      Serial.println(ay);
      Serial.print("az = ");
      Serial.println(az);
*/
      Serial.print(F("Inclinometer angles theta, phi, psi: "));
      Serial.print(theta, 4);
      Serial.print(F(", "));
      Serial.print(phi, 4);
      Serial.print(F(", "));
      Serial.print(psi, 4);
      Serial.println(F(""));
    }
//end accelerometer  
  
  
  phiout=10*phi;
  if(DEBUG){
      Serial.print("phiout=");
      Serial.println(phiout);
  }
  
  
//  for(int number = 0; number <= 200; number++){
//    for(int count = 0; count < 5; count++){
if (digitalRead(REPush) == LOW){
  numberL=encoder0Pos;
}
    printNumber(phiout, encoder0Pos);

if (digitalRead(Button1) == LOW){
  digitalWrite(LED1, 1);
}else{
  digitalWrite(LED1, 0);
}
if (digitalRead(Button2) == LOW){
  digitalWrite(LED2, 1);
}else{
  digitalWrite(LED2, 0);
}
//      if(DEBUG){
//        Serial.print("Number: ");
//        Serial.println(number);
//      }
//    delay(200);
//    }
//  }
}


void printNumber(int LNum, int RNum){

    if(DEBUG){
      Serial.print("LNum=");
      Serial.println(LNum);
  }
  
  boolean IsLNeg, IsRNeg;
  int LOnes, LTens, LHun, LThou, LHunTemp, LThouTemp;
  int ROnes, RTens, RHun, RThou, RHunTemp, RThouTemp;

  //Check if numbers to display are negative, save sign
  if(LNum < 0){
    IsLNeg=1;
    LNum=abs(LNum);
  }else{
    IsLNeg=0;
  }

  if(RNum < 0){
    IsRNeg=1;
    RNum=abs(RNum);
  }else{
    IsRNeg=0;
  }
    
  LOnes=LNum % 10;
  LNum=LNum/10;  
  LTens=LNum % 10;
  LNum=LNum/10;
  LHun=LNum % 10;
  LNum=LNum/10;
  LThou=LNum % 10;
  LNum=LNum/10;
  LThouTemp=LThou;
  LHunTemp=LHun;
  
  ROnes=RNum % 10;
  RNum=RNum/10;  
  RTens=RNum % 10;
  RNum=RNum/10;
  RHun=RNum % 10;
  RNum=RNum/10;
  RThou=RNum % 10;
  RNum=RNum/10;
  RThouTemp=RThou;
  RHunTemp=RHun;
  

 //get rid of leading zeros  if (hundreds==0 && tens==0, put h t to >9)
  if(LThouTemp==0){
    LThou=10;
  }
  if(LThouTemp==0 && LHunTemp==0){
    LThou=10;
    LHun=10;
  }
  
  if(RThouTemp==0){
    RThou=10;
  }
  if(RThouTemp==0 && RHunTemp==0){
    RThou=10;
    RHun=10;
  }

int OutNum[] = {ROnes, RTens, RHun, RThou, LOnes, LTens, LHun, LThou};

/*
  if(DEBUG){
    Serial.print("LThousands: ");
    Serial.println(LThou);
    Serial.print("LHundreds: ");
    Serial.println(LHun);
    Serial.print("LTens: ");
    Serial.println(LTens);
    Serial.print("LOnes: ");
    Serial.println(LOnes);
    Serial.print("RThousands: ");
    Serial.println(RThou);
    Serial.print("RHundreds: ");
    Serial.println(RHun);
    Serial.print("RTens: ");
    Serial.println(RTens);
    Serial.print("ROnes: ");
    Serial.println(ROnes);
  }
*/    
  digitalWrite(ClockPin, 0);
  digitalWrite(ResetPin, 1);
  digitalWrite(ResetPin, 0);
  
  
  
  for(int n = 0; n < 8; n++){
    digitalWrite(ClockPin, 0);
    /*
    if(DEBUG){
        Serial.print("Number to write: ");
        Serial.println(OutNum[n]);
      }
      */
    if((n==3 &&IsRNeg==1)||(n==7 &&IsLNeg==1)){
      digitalWrite(MinusPin,1);
    }
    if(n==1||n==5){
     digitalWrite(DPPin,1);
    } 
    for(int c = 0; c < 4; c++){      
      digitalWrite(DataPins[c], bitRead(OutNum[n],c));
    }
    delay(2);
    digitalWrite(MinusPin,0);
    digitalWrite(DPPin,0);
    digitalWrite(ClockPin, 1);   
  }
}


// Rotary Encoder Code
void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }

  Serial.println (encoder0Pos, DEC);
}
//end Rotary Encoder Code

