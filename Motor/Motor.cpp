/*
  Test.h - Test library for Wiring - implementation
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// include core Wiring API
#include "WProgram.h"

// include this library's description file
#include "Motor.h"

// include description files for other libraries used (if any)
#include "HardwareSerial.h"

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

Motor::Motor(int givenValue)
{
  // initialize this instance's variables
  value = givenValue;

  // do whatever is required to initialize the library
  //pinMode(13, OUTPUT);
  Serial.begin(9600);
}

//Defaults       secure_path="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"




// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

void Motor::doSomething(void)
{
  // eventhough this function is public, it can access
  // and modify this library's private variables
  Serial.print("value is ");
  Serial.println(value);

  // it can also call private functions of this library
  //doSomethingSecret();
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

void Motor::doSomethingSecret(void)
{
  //digitalWrite(13, HIGH);
  //delay(200);
  //digitalWrite(13, LOW);
  //delay(200);
}

