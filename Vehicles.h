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

class AnalogSensor {
public:
	AnalogSensor(int analogInput);
	int read();
	float input();
private:
	float normalize(int value);
	int _input;
	int _min, _max;
};

class DigitalSensor {
public:
	DigitalSensor(int digitalInput);
	int input();
private:
	int _input;
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
