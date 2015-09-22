/* This sketch is just an example to show how to measure RPM and provide a PWM signal to a fan
 *  It has other parts which will be used in my wind tunnel project. These parts are not discussed in the 
 *  Youtube video and can be deleted if you wish. However, they should not create problems
 *  
 *  The sketch is for Leonardos because it uses the keyboard function. If this function is removed it runs also on other Arduinos
 *  
 *  
 */

#include <TimerOne.h>
#include <TimerThree.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define ANEMO_INT 4
#define VENT_PWM 10
#define RPM_RAW 3
#define HEATER 5
#define KEYB_PIN 15
#define TESTPIN 16

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */


//Aneometer
volatile unsigned long lastAnemo = micros();
volatile unsigned long duratAnemo = 300000;
volatile boolean validAneo = false;
float averageAnemo = 0;

//Ventilator
volatile unsigned long lastVent = micros();
volatile unsigned long duratVent = 300000;
volatile boolean validVent = false;
float averageVent = 0;
int ventPwm = 500;
int ii = 0;
long integral = 0;
boolean keyb = false;
float  actSpeed, lastSpeed;
int iii;
long lastEntry = 0;
int pmw;


void setup()
{
  Timer1.pwm(VENT_PWM, 1023);
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Start Setup");
  Keyboard.begin();
  delay(5000);


  // ADS1115
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.begin();


  // Aneometer
  lastAnemo = micros();
  pinMode(ANEMO_INT, INPUT);
  attachInterrupt(ANEMO_INT, countAnemo , RISING);

  // Ventilator
  Timer1.initialize(62);  // 62 us = 16 kHz
  Timer1.pwm(VENT_PWM, 10);
  //pinMode(RPM_RAW,INPUT_PULLUP);
  attachInterrupt(RPM_RAW, countVent, RISING);

  // Heater
  pinMode(9, INPUT);  // nur altes Board wegen Fehler
  pinMode(14, INPUT);  // nur altes Board wegen Fehler
  pinMode(KEYB_PIN, INPUT_PULLUP);
  Timer3.initialize(500000); // 0.5 sec
  pinMode(TESTPIN, OUTPUT);
  setHeater(1);
 // while (speedVent() > 0.0);
  Serial.println(" Start ");
  lastEntry = millis();
}

void loop()
{

  for (iii = 50; iii < 1100; iii = iii + 50) {

    pmw = iii;
    if (pmw > 1023) pmw = 1023;

    Timer1.pwm(VENT_PWM, pmw);

    while ((millis() - lastEntry) < 500) {
      checkkeyb();
      actSpeed = speedVent();
    }
    lastEntry = millis();
    actSpeed = speedVent();

    if (keyb) keybOut();

    Serial.print(" PWM  ");
    Serial.println(pmw);
    Serial.print(" actSpeed ");
    Serial.print(actSpeed);
  }

  Timer1.pwm(VENT_PWM, 1);
  boolean keyb = false;
  while (1 == 1);

}

void checkkeyb() {
  if (digitalRead(KEYB_PIN) == false) {
    keyb = !keyb;
    delay(100);
    while (digitalRead(KEYB_PIN) == false);
  }
}


float speedPitot() {

  return 0.000125 * ads.readADC_SingleEnded(2);
}


float speedAnemo() {
  if (validAneo == true) {
    float speed = 8.2 * 33300.0 / duratAnemo;
    averageAnemo = ((4.0 * averageAnemo) + speed) / 5.0;
    if (speed < 0.9) averageAnemo = 0.0;
    validAneo = false;
  }
  return averageAnemo;
}

float speedVent() {
  long eingang = millis();
  while ((millis() - eingang) < 150 && (validVent == false)) ;

  if (validVent == 1)  {
    float speedVent = 30000000.0 / duratVent;
    averageVent = ((3.0 * averageVent) + speedVent) / 4.0;
    validVent = false;
  } else averageVent = 0.0;
  return averageVent;
}


void countAnemo (void)
{
  long tt = micros() - lastAnemo;
  if (tt > 0) duratAnemo = tt;
  lastAnemo = micros();
  validAneo = true;

}

void countVent(void)
{
  long tt = micros() - lastVent;

  digitalWrite(TESTPIN, 0);
    duratVent = tt;
    lastVent = micros();
    validVent = true;
}

void setSpeed(float _speed) {

#define MAX 100000

  float _p = 0.010;
  float _i = 0.00004;

  float abweichung =  actSpeed - _speed  ;
  integral += abweichung;

  if (integral > MAX) integral = MAX;
  if (integral < -MAX) integral = -MAX;

  float pPart = (_p * abweichung);
  float iPart = (_i * integral);


  float delta = pPart + iPart;
  int target = (int)(ventPwm - delta);

  if (target > 1023) target = 1023;
  if (target < 1) target = 1;

  ventPwm = target;
  if (ventPwm < 50) ventPwm = 50;



  Timer1.pwm(VENT_PWM, ventPwm);

  if ( ii == 10) {
    Serial.print(" _Speed ");
    Serial.print(_speed);
    Serial.print(" actSpeed ");
    Serial.print(actSpeed);
    Serial.print(" abweichung ");
    Serial.print(abweichung);
    Serial.print(" pPart ");
    Serial.print(pPart);
    Serial.print(" iPart ");
    Serial.print(iPart);
    Serial.print(" delta ");
    Serial.print(delta);
    Serial.print(" ventPwm ");
    Serial.println(ventPwm);
    ii = 0;
  }
  ii++;
}

void keybOut() {

  Keyboard.print(pmw);
  Keyboard.print(char(9));
  Keyboard.print(actSpeed);
  Keyboard.print(char(9));
  Keyboard.println();

}

void setHeater(int duty) {
  float tt = duty / 100.0  * 1023.0 ;
  /* Serial.print(duty);
   Serial.print(" ");
   Serial.println(tt);
   */
  Timer3.pwm(HEATER, (int)tt );
}




