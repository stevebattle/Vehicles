/*
   Copyright 2013 Steve Battle

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include "Arduino.h"
#include "Vehicles.h"
#include <math.h>

// constants

//Standard PWM DC control
#define M1 4    //M1 Direction Control
#define E1 5    //M1 Speed Control
#define E2 6    //M2 Speed Control
#define M2 7    //M1 Direction Control

// Debugging LED
#define LED 13

// Analogue sensors
#define RANGE_SENSOR  A0
#define LEFT_SENSOR   A4
#define RIGHT_SENSOR  A5
#define BUTTON_INPUT  A7

// Digital sensors
#define BUMP_SENSOR 8

#define NUM_BUTTONS 5
#define DEBOUNCE 10
#define WINDOW 10
#define INT_MAX 32767

Vehicle::Vehicle() {
   pinMode(LED, OUTPUT);  // debug LED
   pinMode(BUMP_SENSOR, INPUT);
   digitalWrite(BUMP_SENSOR,HIGH); // enable 20K pull-up resistor on switch
   _buttons[0] = 30;
   _buttons[1] = 150;
   _buttons[2] = 360;
   _buttons[3] = 535;
   _buttons[4] = 760;
   _buttonValue = -1;
   _frontAverage = 0;
   _range_sensor = RANGE_SENSOR; // re-assigned on R3
   _button_input = BUTTON_INPUT; // re-assigned on R3
}

int Vehicle::buttonPressed() {
  int in = analogRead(_button_input);
  for (int k=0; k<NUM_BUTTONS; k++) {
    if (in<_buttons[k]) {
      if (_buttonValue==k) {
        if (_buttonCount>DEBOUNCE) return 0;
        return _buttonCount++ < DEBOUNCE ? 0 : k+1;
      }
      if (_buttonValue<0) _buttonValue = k;
      return 0;
    }
  }
  _buttonValue = -1; _buttonCount = 0;
  return 0;
}

float Vehicle::normalize(int sensor, int value) {
  if (value<_min[sensor]) _min[sensor] = value;
  if (value>_max[sensor]) _max[sensor] = value;
  return (value - _min[sensor]) / (float)(_max[sensor] - _min[sensor]);
}

void Vehicle::motors(float left, float right) {
  if (fabs(left)<0.1 && fabs(right)<0.1) { // stop
    digitalWrite(E1,LOW);   
    digitalWrite(E2,LOW);  
  }
  // 1 battery pack cannot power rapid direction switching - brake and pause
  if (left*_m1<0 || right*_m2<0) {
	analogWrite (E1, 0);    
    digitalWrite(M1, LOW);
    analogWrite (E2, 0);    
    digitalWrite(M2, LOW);
    delay(500);
  }
  // left
  if (left>1) left = 1;
  else if (left<-1) left = -1;
  analogWrite (E1, fabs(left)*255);    
  digitalWrite(M1, left>=0 ? LOW : HIGH);
  _m1 = left;
  // right
  if (right>1) right = 1;
  else if (right<-1) right = -1;
  analogWrite (E2, fabs(right)*255);
  digitalWrite(M2, right>=0 ? LOW : HIGH);
  _m2 = right;
}

/* multiply l*m (rows,cols) matrix by m*n matrix */

void Vehicle::multiply(float a[], float b[], float c[], int l, int m, int n) {
  for (int i=0; i<l; i++) {
    for (int j=0; j<n; j++) {
      c[i*n+j] = 0;
      for (int k=0; k<m; k++) {
        c[i*n+j] += a[i*m+k]*b[k*n+j];
      }
    }
  }
}

/* mutation with lateral symmetry */

void Vehicle::mutate(float a[], int m, int n, float min, float max) {
  int i = random(m-1)+1, j = random(n-1)+1; // i,j >0
  a[(m-i)*n+(n-j)] = a[i*n+j] = random(min*100,max*100)/100.0;
}

/* sigmoids contains offset and slope for squashing function */

void Vehicle::squash(float a[], float sigmoids[], int n) {
  for (int i=0; i<n; i++) {
    float f = sigmoids[i];
    // apply sigmoid, result in range [0,1] 
    // pass 1 through unchanged, 0 maps to 0.5
    if (a[i]<1) a[i] = 1/(1+exp(-f*a[i]));
  }
}

float Vehicle::leftSensor() {
  return normalize(LEFT,analogRead(LEFT_SENSOR));
}

float Vehicle::rightSensor() {
  return normalize(RIGHT,analogRead(RIGHT_SENSOR));
}

float Vehicle::frontSensor() {
  float r = normalize(RANGE,analogRead(_range_sensor));
  return _frontAverage = smooth(WINDOW,_frontAverage,r);
}

float Vehicle::smooth(int window, float average, float value) {
  return ((window-1) * average + value) / (float) window;
}

float Vehicle::oscillator(int frequency) {
  float n = millis() % frequency / (float) frequency;
  return sin(n*2*M_PI);
}

void Vehicle::sensorInput(float data[], int n) {
  if (n>0) data[0] = 1.0;
  if (n>1) data[1] = leftSensor();
  if (n>2) data[2] = frontSensor();
  if (n>3) data[3] = rightSensor();
}

void Vehicle::r3() {
  _button_input = A0;
  _range_sensor = A1;
}

/* Analog Sensor */

float AnalogSensor::normalize(int value, int * min, int * max) {
  if (value<*min) *min = value;
  if (value>*max) *max = value;
  return (value - *min) / (float)(*max - *min);
}

AnalogSensor::AnalogSensor(int analogInput) {
	_input = analogInput;
}

int AnalogSensor::input() {
	return analogRead(_input);
}

/* Digital Sensor */

DigitalSensor::DigitalSensor(int digitalInput) {
	_input = digitalInput;
}

int DigitalSensor::input() {
	return digitalRead(_input);
}

/* Light Sensor */

LightSensor::LightSensor(int analogInput) : AnalogSensor(analogInput) {
	_max = 0;
	_min = INT_MAX;
}

float LightSensor::input() {
	int l = AnalogSensor::input();
	return AnalogSensor::normalize(l, & _min, & _max);
}

/* Bumper */

Bumper::Bumper(int digitalInput) : DigitalSensor(digitalInput) {
}

float Bumper::input() {
  int b = DigitalSensor::input();
  return b ? 0.0 : 1.0;	
}

/* Buttons */

Buttons::Buttons(int analogInput) : AnalogSensor(analogInput) {
   _buttons[0] = 30;
   _buttons[1] = 150;
   _buttons[2] = 360;
   _buttons[3] = 535;
   _buttons[4] = 760;
   _buttonValue = -1;
}

int Buttons::pressed() {
  int in = AnalogSensor::input();
  for (int k=0; k<NUM_BUTTONS; k++) {
    if (in<_buttons[k]) {
      if (_buttonValue==k) {
        if (_buttonCount>DEBOUNCE) return 0;
        return _buttonCount++ < DEBOUNCE ? 0 : k+1;
      }
      if (_buttonValue<0) _buttonValue = k;
      return 0;
    }
  }
  _buttonValue = -1; _buttonCount = 0;
  return 0;
}

/* Motors */

Motors::Motors(int m1, int e1, int m2, int e2) {
	_m1 = m1;
	_e1 = e1;
	_m2 = m2;
	_e2 = e2;
}

void Motors::output(float left, float right) {
  if (fabs(left)<0.1 && fabs(right)<0.1) { // stop
    digitalWrite(_e1,LOW);   
    digitalWrite(_e2,LOW);  
  }
  
  // 1 battery pack cannot power rapid direction switching - brake and pause
  if (left*_prevM1<0 || right*_prevM2<0) {
	analogWrite (_e1, 0);    
    digitalWrite(_m1, LOW);
    analogWrite (_e2, 0);    
    digitalWrite(_m2, LOW);
    delay(500);
  }
  
  // left
  if (left>1) left = 1;
  else if (left<-1) left = -1;
  analogWrite (_e1, fabs(left)*255);    
  digitalWrite(_m1, left>=0 ? LOW : HIGH);
  _prevM1 = left;
  
  // right
  if (right>1) right = 1;
  else if (right<-1) right = -1;
  analogWrite (_e2, fabs(right)*255);
  digitalWrite(_m2, right>=0 ? LOW : HIGH);
  _prevM2 = right;
}