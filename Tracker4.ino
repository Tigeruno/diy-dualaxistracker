#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RTClib.h"
#include <PWM.h>    // library written by Sam Knight, thankyou
#include <TimeLib.h>

RTC_DS1307 rtc;

/* TRACKER4 version 7/20/19  by David Hanson
  thanks to dronebot for many videos on programming the arduino
  thanks to UT space program and "instructables" for the tracking algorythm

  6/19/19 added speedlimit varible to control azimuth speed
  6/26/19 add RTC toss millis for timing
  6/27/19 add PWM library and pin safe for (azimuth motor)
  6/27/19 changed pwm pin to D9, was on D5
  6/30/19 added two functions, quit_tracking() and park()
  7/8/19 added Time.h for monthly sunrise and sunset calculations
  7/10/19 added eeprom (save pulse count from actuator) made time RLC functional
*/
// A0 = Northwest, A1 = Northeast, A2 = Southwest, A3 = Southeast
byte ldrpin[] = {A0, A1, A2, A3}; // array for input pins with LDRs
int sensor[] = {0, 0, 0, 0}; // zero sensor values
int diff[] = {0, 0, 0, 0};   // check for biggest diff

#define pwm 9     // start or stop motor (azimuth)  analog use with PWM library
#define dir 13    // travel direction (East/West)   digital
#define pwm2 6    // motor 2 (linear actuator)
#define dir2 12   // direction for motor 2 (linear actuator)

#define E_limit A6 // proximity sensor EAST stop 
#define W_limit A7 // proximity sensor WEST stop
#define pulse 4    // count pulses
// A4 and A5 used by I2C RTC

int aveTOP = 0;              // tally mean of the 2 top sensors
int aveRIGHT = 0;            // tally mean of 2 right sensors
int aveBOTTOM = 0;           // tally mean of 2 bottom sensors
int aveLEFT = 0;             // tally mean of 2 left sensors

const int sensitivity = 10;       // determines the accuracy of tracking in azimuth and North/South
int speedset = 170;
byte i = 0;
byte quad = 5;               // what quadrant should I move to?

// timestamp for hitting the limit switch
// unsigned long prevMillis = 0;
unsigned long currentMillis = 0;

const int WEST = 1;         // west or right direction
const int NORTH = 0;        // North or up
int weststop = 0;           // set flag if west limit reached

unsigned int Pulse_ct = 0;     // counts pulses from linear actuator
int eeprom_address = 0;        // set address for eeprom write (Pulse_ct)

// Sunrise,Sunset,Hour,Minute    Jan            Feb           Mar            Apr            May           June          July           August        Sept           Oct            Nov            Dec
byte sunRS[13][4] = {0, 0, 0, 0, 7, 10, 17, 20, 7, 0, 17, 50, 6, 45, 18, 20, 6, 13, 19, 45, 6, 15, 20, 5, 6, 8, 20, 21, 6, 17, 20, 19, 6, 38, 20, 0, 6, 57, 19, 48, 7, 19, 18, 40, 7, 44, 18, 42, 7, 8, 17, 12};
byte sunHR = 0;   // array index for hour sunrise
byte sunHS = 2;   // array index for hour sunset
byte sunMR = 1;   // array index for minute sunrise
byte sunMS = 3;   // array index for minute sunset
DateTime t;

// Set the PWM Frequency  3900 frequency (dronebot recommended this freq)
int32_t frequency = 3900;  // set pwm freq

void setup() {
  Wire.begin();

  InitTimersSafe();    // pwm setup  (retains millis normal on timer0)
  bool success = SetPinFrequencySafe(pwm, frequency);  // only this pin has freq change from normal

  Serial.begin(9600);
  while (!Serial) ; // wait until Arduino Serial Monitor opens
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //    setSyncProvider(syncProvider);
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  // Define Pins
  pinMode(dir, OUTPUT);    // determines direction of azimuth motor
  pinMode(dir2, OUTPUT);   // Linear actuator direction
  pinMode(pwm2, OUTPUT);   // Linear actuator enable or stop
  pinMode(pulse, INPUT);   // count the pulses from Linear actuator
  pinMode(W_limit, INPUT_PULLUP); // prox sensor WEST (NPN normally open)
  pinMode(E_limit, INPUT_PULLUP); // prox sensor EAST

}

void loop() {
  t = rtc.now();      // place current time in structure t
  EEPROM.get(4, Pulse_ct);
  delay (200);
  /*  Serial.print("Pulse = ");
    Serial.println(Pulse_ct);
    Serial.print("Time ");
    Serial.print(t.hour());
    Serial.print(":");
    Serial.println(t.minute());  */
  quad = 5;             // start over after a quad change from tracking

  // Track every 12 minutes on the minute and 2 hours after sunrise
  if (t.hour() >= (sunRS[t.month()][sunHR] + 2) && t.minute() % 12 == 0 && !weststop)
    quad = track();    // get the quadrant that Head Unit needs to move to
  /*  Serial.print("% = ");
    Serial.println(t.minute() % 12);
  */
  switch (quad) {
    case 0:    {              // Left or East
        JOG(1200, !WEST);
        delay(100);
        break;
      }
    case 1:  {                // Right or West
        if (weststop == 0) JOG(1200, WEST);
        delay(100);
        break;
      }
    case 2:  {     // extend the linear actuator move south
        NS(2000, !NORTH);
        delay(100);
        break;
      }
    case 3:  {    // retract linear actuator move north
        NS(2000, NORTH);
        delay(100);
        break;
      }
  }                  //end of switch(quad)

  if (t.hour() == (sunRS[t.month()][sunHS] - 1)) {
    delay(100);
    quit_tracking(!WEST);     // return to east
    JOG(1000, WEST);           // move off of limiter
    park();                   // lay array 5 degrees flat to protect from wind damage
    digitalWrite(dir, WEST);
    dayover(t);
  }

  if (analogRead(E_limit) > 600) {     // we hit East limit switch
    pwmWrite(pwm, 0);                  // if so stop the travel and change direction and move back off the limiter
    JOG(1100, WEST);                    // move away from E_limit sensor
    delay (100);
  }

  /*  Serial.print("else if = ");
    Serial.println(sunRS[t.month()][sunHS] - 1);  */
}

void JOG(unsigned long steps, int setdir) {  // run azimuth motor for number of millis
  /*  Serial.println("Jog is running");  */
  digitalWrite(dir, setdir);

  unsigned long prevMillis = millis(); // grab current time
  // check if "stepinterval" time has passed 600 moves approximately 7 degrees azimuth
  while ((currentMillis = millis() - prevMillis) <= steps && (analogRead(W_limit) < 500) && (analogRead(E_limit) < 500))
    pwmWrite(pwm, speedset);         // run motor while true above at speedlimit variable

  pwmWrite(pwm, 0);
  delay(100);                        // stop after jog
  if (analogRead(E_limit) > 600)  {   // hit east limiter now move off
    prevMillis = millis();
    digitalWrite(dir, WEST);
    while ((millis() - prevMillis) <= 1100)
      pwmWrite(pwm, speedset);
  }
  else if (analogRead(W_limit) > 600) {   // hit west limiter
    weststop = true;
    digitalWrite(dir, !WEST);
    prevMillis = millis();
    while ((millis() - prevMillis) <= 1100)    // move off limiter
      pwmWrite(pwm, speedset);
  }
  pwmWrite(pwm, 0);
  delay(100);
}

void NS(unsigned long duration, int inout) {  // run linear actuator for number of millis
  /*  Serial.print("Pulse count ");
    Serial.println(Pulse_ct);
    delay(500);  */
  digitalWrite(dir2, inout);
  unsigned long startMillis = millis(); // grab current time
  unsigned long saveMillis;

  // check if "stepinterval" time has passed
  while ((saveMillis = millis()) <= (startMillis + duration)) {
    digitalWrite(pwm2, HIGH);              // run linear actuator
    if (digitalRead(pulse) == HIGH)  {
      if (inout == NORTH)  { // check for direction
        if (Pulse_ct <= 0) Pulse_ct = 0;
        else Pulse_ct--;
      }
      else Pulse_ct++;      // count the pulses while traveling

      while (millis() <= (saveMillis + 25) || digitalRead(pulse) != LOW);
    }
  }
  digitalWrite(pwm2, LOW);                  // stop
  delay(100);
}

int track() {  // determine which quadrant is darkest, and send quad value for proper movement of Head Unit
  byte result = 5;    // default no LDR sensor value difference enough to make a move
  byte biggest = 0;   // get biggest difference in array
  byte index = 0;     // save index of biggest for return from function

  // read the sensors
  for (i = 0; i < 4; i++)
    sensor[i] = analogRead(ldrpin[i]);  // get all the sensor values

  aveTOP = (sensor[0] + sensor[1]) / 2;    // thanks University Tennessee for this algorythm
  aveRIGHT = (sensor[1] + sensor[3]) / 2;  // and "Instructables"
  aveBOTTOM = (sensor[2] + sensor[3]) / 2;
  aveLEFT = (sensor[0] + sensor[2]) / 2;

  diff[0] = aveRIGHT - aveLEFT;  // mod added, choice 1 was always first before without testing for bigger
  diff[1] = aveLEFT - aveRIGHT;
  diff[2] = aveBOTTOM - aveTOP;
  diff[3] = aveTOP - aveBOTTOM;

  for (i = 0; i < 4; i++)  {        // determine the biggest difference to send for change
    if (diff[i] > biggest)  {
      biggest = diff[i];
      index = i;
    }
  }
  if (diff[index] > sensitivity)  result = index;
  return result;
}

void quit_tracking(int dirEAST)  {
  EEPROM.put(4, Pulse_ct);     // save Pulse_ct to eeprom address

  digitalWrite(dir, dirEAST);                        // set direction back to East
  delay(100);
  while (analogRead(E_limit) < 500)            // move 200 degrees from West limiter to East limiter
    pwmWrite(pwm, speedset);                   // move slower than normal speed

  pwmWrite(pwm, 0);
  weststop = 0;                            // clear flag for hit west limiter
}

void park()  {  // retrack till flat
  digitalWrite(dir2, NORTH);   // set to retract actuator  (what happens if hit limiter???)
  unsigned long saveMillis;

  digitalWrite(pwm2, HIGH);              // run linear actuator
  while (Pulse_ct > 0) {
    saveMillis = millis();
    if (digitalRead(pulse) == HIGH)  {
      if (Pulse_ct <= 0) Pulse_ct = 0;
      else Pulse_ct--;                          // count the pulses while traveling
      while ((millis() <= (saveMillis + 25)) || digitalRead(pulse) == LOW);
    }
  }
  digitalWrite(pwm2, LOW);    // stop travel
}

void dayover(DateTime t) {

  while (t.hour() != sunRS[t.month()][sunHR] + 2);   // delay till sunrise
  while (t.minute() <= sunRS[t.month()][sunMR]);
}
