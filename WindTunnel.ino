

#include <TimerOne.h>
#include <TimerThree.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define ANEMO_INT 4
#define VENT_PWM 10
#define RPM_RAW 3
#define HEATER 5
#define KEYB_PIN 15
#define TESTPIN1 16
#define TESTPIN2 8


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
int ventPwm;
int lastVentPwm = 0;
int ii = 0;
boolean keyb = true;

//PID
long integral = 0;
int lastSpeed;
float targetPWM;

long lastEntry = 0;
int pmw;
unsigned long lastTime, lastKeyb;
boolean first = true;
int lastDeltaSpeed = 0;


void setup()
{
  Timer1.pwm(VENT_PWM, 0);
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Start Setup");
  Keyboard.begin();


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
  pinMode(TESTPIN1, OUTPUT);
  pinMode(TESTPIN2, OUTPUT);
  digitalWrite(TESTPIN2, HIGH);
  setHeater(1);
  while (speedVent() > 0.0);
  Serial.println(" Start ");
  lastEntry = millis();
  Timer1.pwm(VENT_PWM, 0);
  lastSpeed = speedVent();
}

void loop()
{
  if (first) {
    //   Keyboard.print("i");
    //   Keyboard.print(char(9));
    Keyboard.print("PWM");
    Keyboard.print(char(9));
    Keyboard.print("Speed");
    Keyboard.println(char(9));

    first = false;
  }
  setSpeed(2000);
  //delay(600);
  checkkeyb();
}

void checkkeyb() {
  if (digitalRead(KEYB_PIN) == false) {
    keyb = false;
    digitalWrite(TESTPIN2, LOW);
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

int speedVent() {
  long eingang = millis();
  while ((millis() - eingang) < 150 && (validVent == false)) ;

  if (validVent == 1)  {
    float speedVent = 30000000.0 / duratVent;
    averageVent = ((3.0 * averageVent) + speedVent) / 4.0;
    validVent = false;
  } else averageVent = 0.0;
  return int(averageVent);
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

  digitalWrite(TESTPIN1, 0);
  duratVent = tt;
  lastVent = micros();
  validVent = true;
}

void setSpeed(int _speed) {


#define MAX 100000


  float _p = 0.1;
  float _i = 0.001;
  float _d = 1.0;


  int _deltaTime = millis() - lastTime;

  // Calculate delta speed
  int actSpeed = speedVent();
  int _deltaSpeed =  _speed - actSpeed;


  float hi = ( _deltaSpeed - lastDeltaSpeed);

  float _deltaSpeedDiff = hi;

  //  if (abs(_deltaSpeedDiff) < 10) _deltaSpeedDiff = 0;

  integral += (_deltaSpeed);

  float pPart = (_p * float(_deltaSpeed));
  float iPart = (_i * float(integral));
  float dPart = (_d * float(_deltaSpeedDiff));


/*  Serial.print(actSpeed);
  Serial.print(" deltaSpeed ");
  Serial.print(_deltaSpeed);
  Serial.print(" lastDeltaSpeed ");
  Serial.print(lastDeltaSpeed);
  Serial.print(" dd ");
  Serial.print(lastDeltaSpeed - _deltaSpeed);
  Serial.print(" hi ");
  Serial.print(hi);
  Serial.print(" dPart ");
  Serial.print(dPart);
  Serial.print(" Diff ");
  Serial.println(_deltaSpeedDiff);
  */

 // pPart = 0.0;
 //    iPart = 0.0;
  dPart = 0.0;
  

  float delta = pPart + iPart + dPart;

  targetPWM = 5.75 * delta;

  if (targetPWM > 1023.0) targetPWM = 1023.0;
  if (targetPWM < 1.0) targetPWM = 1.0;

  ventPwm = int(targetPWM);
  if (ventPwm < 50) ventPwm = 50;

  Timer1.pwm(VENT_PWM, ventPwm);

  if (keyb) {
    if (millis() - lastKeyb > 700) {

      Keyboard.print(ventPwm);
      Keyboard.print(char(9));
      Keyboard.print(actSpeed);
      Keyboard.println(char(9));
      lastKeyb = millis();
    }
    /*  else {
        Serial.print(" _Speed ");
        Serial.print(_speed);
        Serial.print(" actSpeed ");
        Serial.print(actSpeed);
        Serial.print(" _deltaSpeed ");
        Serial.print(_deltaSpeed);
        Serial.print(" pPart ");
        Serial.print(pPart);
        Serial.print(" iPart ");
        Serial.print(iPart);
        Serial.print(" dPart ");
        Serial.print(dPart);
        Serial.print(" delta ");
        Serial.print(delta);
        Serial.print(" ventPwm ");
        Serial.println(ventPwm);
      }
      */
  }
  ii++;
  lastSpeed = actSpeed;
  lastDeltaSpeed = _deltaSpeed;
  lastTime = millis();
  lastVentPwm = ventPwm;
}

void setHeater(int duty) {
  float tt = duty / 100.0  * 1023.0 ;
  /* Serial.print(duty);
   Serial.print(" ");
   Serial.println(tt);
   */
  Timer3.pwm(HEATER, (int)tt );
}




