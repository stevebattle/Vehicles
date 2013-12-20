/*
   Copyright 2013 Steve Battle
 
   This work is licensed under a Creative Commons Attribution 3.0 Unported License.
   
   You may obtain a copy of the License at
 
       http://creativecommons.org/licenses/by/3.0
*/

#include "Arduino.h"
#include "Vehicles.h"
#include <math.h>

// constants


#define NUM_BUTTONS 5
#define DEBOUNCE 10
#define WINDOW 10
#define INT_MAX 32767

/* multiply l*m (rows,cols) matrix by m*n matrix */

void multiply(float a[], float b[], float c[], int l, int m, int n) {
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

void mutate(float a[], int m, int n, float min, float max) {
  int i = random(m-1)+1, j = random(n-1)+1; // i,j >0
  a[(m-i)*n+(n-j)] = a[i*n+j] = random(min*100,max*100)/100.0;
}

/* sigmoids contains offset and slope for squashing function */

void squash(float a[], float sigmoids[], int n) {
  for (int i=0; i<n; i++) {
    float f = sigmoids[i];
    // apply sigmoid, result in range [0,1] 
    // pass 1 through unchanged, 0 maps to 0.5
    if (a[i]<1) a[i] = 1/(1+exp(-f*a[i]));
  }
}

float smooth(int window, float average, float value) {
  return ((window-1) * average + value) / (float) window;
}

float oscillator(int frequency) {
  float n = millis() % frequency / (float) frequency;
  return sin(n*2*M_PI);
}

/* Analog Sensor */

float AnalogSensor::normalize(int value) {
  if (value<_min) _min = value;
  if (value>_max) _max = value;
  return (value - _min) / (float)(_max - _min);
}

AnalogSensor::AnalogSensor(int analogInput) {
	_input = analogInput;
	_max = 0;
	_min = INT_MAX;
}

int AnalogSensor::read() {
	return analogRead(_input);
}

float AnalogSensor::input() {
	return AnalogSensor::normalize(AnalogSensor::read());
}

/* Digital Sensor */

DigitalSensor::DigitalSensor(int digitalInput) {
	_input = digitalInput;
}

int DigitalSensor::input() {
	return digitalRead(_input);
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
  int in = AnalogSensor::read();
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