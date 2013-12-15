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

#ifndef Vehicle_h
#define Vehicle_h

#include "Arduino.h"

class Vehicle {

public:
	Vehicle();
	int buttonPressed();
	float leftSensor();
	float rightSensor();
	float frontSensor();
	void motors(float left, float right);
	void multiply(float a[], float b[], float c[], int l, int m, int n);
	void mutate(float a[], int m, int n, float min, float max);
	void squash(float a[], float sigmoids[], int n);
	void sensorInput(float data[], int n);
	float smooth(int window, float average, float value);
	float oscillator(int frequency);
	void r3();

private:
	float normalize(int sensor, int value);
	int _buttonValue, _buttonCount;
	float _m1, _m2;
	enum _sensors { LEFT, RIGHT, RANGE };
	int _buttons[5];
	int _min[3], _max[3];
	float _frontAverage;
	int _range_sensor;
	int _button_input;
};

class AnalogSensor {
public:
	AnalogSensor(int analogInput);
	int input();
	float normalize(int value, int * min, int * max);
private:
	int _input;
};

class DigitalSensor {
public:
	DigitalSensor(int digitalInput);
	int input();
private:
	int _input;
};

class LightSensor : public AnalogSensor {
public:
	LightSensor(int analogInput);
	float input();
private:
	int _min, _max;
};

class Bumper : public DigitalSensor {
public:
	Bumper(int digitalInput);
	float input();
};

class Buttons : public AnalogSensor {
public:
	Buttons(int analogInput);
	int pressed();
private:
	int _buttons[5];
	int _buttonValue, _buttonCount;
};

class Motors {
public:
	Motors(int m1, int e1, int m2, int e2);
	void output(float left, float right);
private:
	int _m1, _e1, _m2, _e2, _prevM1, _prevM2;
};

#endif
