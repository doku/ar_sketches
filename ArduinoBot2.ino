#include <PID_v1.h>


/*
Metal detector robot
 
 */

#include <Servo.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
//#include <tmotor>
//#include <Test.h>
//#include <Motor.h>
//include <MotorController.h>

unsigned long time;
const int clockrate = 5; //millisec
const bool debug = true;
char debugmsg[255] = "";

Servo servo_pan;
Servo servo_tilt;

const int wheel_pwm_1 = 4; // speed control pin, 0-255, front left wheel
const int wheel_pwm_2 = 5; // front right
const int wheel_pwm_3 = 6; // back left
const int wheel_pwm_4 = 7; // back right wheel

const int wheel_current_draw_1 = A0; // make sure it's less than 4 amps, which means 4v on this pin. which means sensorValue * (5.0/1023.0) <= 4 volt; which means reading less than 818.4
const int wheel_current_draw_2 = A1; // make sure analogRead(A1) <= 818.4; 
const int wheel_current_draw_3 = A2;
const int wheel_current_draw_4 = A3;

const int wheel_direction_1 = 22; // false == forward
const int wheel_direction_2 = 23; //false == forward
const int wheel_direction_3 = 24; // false == forward
const int wheel_direction_4 = 25; // false == forward

const int detector_pin = 46;

const int sonar_pin = A4;

const int servo_pan_pin = 45;
const int servo_tilt_pin = 47;

const int encoder_register = PINC;
const int encoderAPinARegNum = 7; // PINC and 7 is the register address for pin 30
const int encoderAPinBRegNum = 6; // address for pin 31
const int encoderAInterrupt = 0;  // interrupt 0 == GIOP 2
volatile bool encoderAPinBLast;
volatile unsigned int encoderAPos = 0;

const int encoderBPinARegNum = 5; // PINC and 7 is the register address for pin 30
const int encoderBPinBRegNum = 4; // 
const int encoderBInterrupt = 1; // interrupt 1 == GIOP 3
volatile bool encoderBPinBLast;
volatile unsigned int encoderBPos = 0;

const int encoderCPinARegNum = 3; // PINC and 7 is the register address for pin 30
const int encoderCPinBRegNum = 2; // address for pin 31
const int encoderCInterrupt = 2;// = GIOP 21
volatile bool encoderCPinBLast;
volatile unsigned int encoderCPos = 0;

const int encoderDPinARegNum = 1; // PINC and 7 is the register address for pin 30
const int encoderDPinBRegNum = 0; // address for pin 31
const int encoderDInterrupt = 3; // = GIOP 20
volatile bool encoderDPinBLast;
volatile unsigned int encoderDPos = 0;

// GPS
Adafruit_GPS GPS(&Serial3);
const bool GPSECHO = false; // true for verbosity
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// init values
int servo_pan_pos = 90; // centered position
const int servo_pan_max = 175; //left
const int servo_pan_min = 5; //right
bool servo_pan_dir = true;
int servo_tilt_pos = 115; // flat position
const int servo_tilt_min = 50; // low
const int servo_tilt_max = 130; // high
bool servo_tilt_dir = true;

int detector_cycle = 0;
int testwheel_dir = 0;
int testwheel_pwm = 0;
int dir = HIGH;
int cycle = 0;
int cycle_500_counter = 0;
int cycle_500 = 50; // = .5 sec
unsigned int pos = 0;

double back_left_velocity, back_left_output, back_left_setpoint;
double back_right_velocity, back_right_output, back_right_setpoint;
PID back_left_PID(&back_left_velocity, &back_left_output, &back_left_setpoint, 2,10,.5, DIRECT);
PID back_right_PID(&back_right_velocity, &back_right_output, &back_right_setpoint, 2,10,.5, DIRECT);

unsigned long prevTime = 0;

void gogogo(int speed, int direction);

//Test teste(5);
//MotorController Bot();


void setup(){
  if(debug == true){
    Serial.begin(115200); 
    Serial.println("start");
  }
  time = millis();
  pinMode(wheel_pwm_1, OUTPUT); // wheel speed 0-255
  pinMode(wheel_pwm_2, OUTPUT);
  pinMode(wheel_pwm_3, OUTPUT);
  pinMode(wheel_pwm_4, OUTPUT);
  pinMode(wheel_direction_1, OUTPUT); //HIGH is forward, LOW is backwards
  pinMode(wheel_direction_2, OUTPUT);
  pinMode(wheel_direction_3, OUTPUT);
  pinMode(wheel_direction_4, OUTPUT);

  pinMode(detector_pin, INPUT);
  // pinMode(48,OUTPUT); // temp led pin
  // pinMode(46,INPUT);
  //pinMode(44,INPUT);

  //pinMode(servo1_pin, OUTPUT);
  //pinMode(servo2_pin, OUTPUT);
  servo_pan.attach(servo_pan_pin);
  servo_tilt.attach(servo_tilt_pin);
  servo_tilt.write(servo_tilt_pos);

  // encoder interrupt setup
  attachInterrupt(encoderAInterrupt, encoderA, CHANGE);
  encoderAPinBLast = bitRead(encoder_register,encoderAPinBRegNum);
  attachInterrupt(encoderBInterrupt, encoderB, CHANGE);
  encoderBPinBLast = bitRead(encoder_register,encoderBPinBRegNum);
  attachInterrupt(encoderCInterrupt, encoderC, CHANGE);
  encoderCPinBLast = bitRead(encoder_register,encoderCPinBRegNum);
  attachInterrupt(encoderDInterrupt, encoderD, CHANGE);
  encoderDPinBLast = bitRead(encoder_register,encoderDPinBRegNum);

  //GPS
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  
  back_left_velocity = 0;
  back_right_velocity = 0;
  back_left_setpoint = 10;
  back_right_setpoint = 10;
  back_left_PID.SetMode(AUTOMATIC);
  back_right_PID.SetMode(AUTOMATIC);
  
  prevTime = millis();

  //Serial.println("wheel 1 init");
  //initTestWheel(wheel_direction_1, wheel_pwm_1, wheel_current_draw_1);
  //Serial.println("Wheel 2 init");
  //initTestWheel(wheel_direction_2, wheel_pwm_2, wheel_current_draw_2);
  //Serial.println("Wheel 3 init");
  //initTestWheel(wheel_direction_3, wheel_pwm_3, wheel_current_draw_3);
  //Serial.println("Wheel 4 init");
  //initTestWheel(wheel_direction_4, wheel_pwm_4, wheel_current_draw_4);
  //int wheel_dir = wheel_direction_1;
  //int wheel_pwm = wheel_pwm_1;
  //int wheel_draw = wheel_current_draw_1;

  //gogogo(200,1);
  //motortest();
  //testwheel();
  //gogogo(100, 0);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
  // writing direct to UDR0 is much much faster than Serial.print 
  // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop2(){
  if(false){
    gogogo(255, 0);
    delay(10000);
    gogogo(0,0);
    delay(1000);
    gogogo(255,1);
    delay(10000);
    gogogo(0,0);
    delay(1000);
    gogogo(255, 0);
    delay(10000);
    gogogo(0,0);
    delay(1000); 
    gogogo(255,1);
    delay(10000);
    gogogo(0, 0);
    delay(1000);
  }

  if(false){
    turninplace(100,0);
    delay(1000);
    turninplace(100,1);
    delay(1000);
  }
  if(true){

    sonar_scan(); 
    delay(100);
  }

  if(false){
    if(pos != encoderAPos){
      pos = encoderAPos;
      Serial.print("Encoder A: ");
      Serial.println(encoderAPos,DEC);
    } 
  }
}


void loop(){
  
  

        
  

  
  if(true){
  if (time+clockrate < millis()){
    time = millis();
    //cycle_500_counter++;
    //if(cycle_500_counter >= cycle_500){
    //  cycle_500_counter = 0; 
    //}
    cycle++;
    if(cycle >= 100){
      cycle = 0; 
    }
    if(cycle % 1 == 0){
       back_left_setpoint = 10;
       back_right_setpoint = 10;
       back_left_velocity = encoderAPos / (millis() - prevTime);
       back_right_velocity = encoderBPos / (millis() - prevTime);
       //Serial.println(encoderAPos);
       encoderAPos = 0;
       encoderBPos = 0;
        //Serial.println(encoderAPos);
       //Serial.println(back_left_velocity);
        
        back_left_PID.Compute();
        back_right_PID.Compute();
        analogWrite(wheel_pwm_3, back_left_output);
        analogWrite(wheel_pwm_1, back_left_output);
        analogWrite(wheel_pwm_4, back_right_output);
        analogWrite(wheel_pwm_2, back_right_output);
        
        prevTime = millis();
        
        
        if(true){
          //if(pos != encoderAPos){
            //pos = encoderAPos;
            Serial.print("Encoder A: ");
            Serial.println(back_right_velocity);
          //} 
        }

    
      
    }
    
    if(cycle % 1 == 0){
      if(false){
        //gogogo(50,0);
        if(sonar_scan()){
          gogogo(0,0);
          delay(1000);
          turninplace(70,0);
          delay(2000); 
          gogogo(0,0);
          delay(1000);
          gogogo(50,0);
        }else{
         gogogo(50,0); 
        }
      }
    }

      //startup_test();
      //sonar_test();
      //gps_test(); 
      //teste.doSomething();
      //Serial.println(motorcon(7));
      //Bot.doFunction();
    }

    /*
    if(pos != encoderAPos){
     pos = encoderAPos;
     Serial.println(encoderAPos,DEC);
     }
     */

    /*if(cycle % 2 == 0){ // 20 millisec
     servo_test();
     }
     */
    //sonar_test(cycle1_counter);
    //detector_test();
    //wheeltest(time);
    if(debug == true && cycle == 0 && debugmsg != ""){
      Serial.println(debugmsg);
    }
  }
}

void startup_test(){

  //detector_test();
  //sonar_test();
  //servo_test();
}

void gps_test(){
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  //if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  //if (millis() - timer > 2000) { 
  //timer = millis(); // reset the timer

  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); 
  Serial.print(':');
  Serial.print(GPS.minute, DEC); 
  Serial.print(':');
  Serial.print(GPS.seconds, DEC); 
  Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); 
  Serial.print('/');
  Serial.print(GPS.month, DEC); 
  Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); 
  Serial.print((int)GPS.fix);
  Serial.print(" quality: "); 
  Serial.println((int)GPS.fixquality); 
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); 
    Serial.print(GPS.lat);
    Serial.print(", "); 
    Serial.print(GPS.longitude, 4); 
    Serial.println(GPS.lon);

    Serial.print("Speed (knots): "); 
    Serial.println(GPS.speed);
    Serial.print("Angle: "); 
    Serial.println(GPS.angle);
    Serial.print("Altitude: "); 
    Serial.println(GPS.altitude);
    Serial.print("Satellites: "); 
    Serial.println((int)GPS.satellites);
  }
  //}

}

void encoderA(){
  encoderAPos++;
  //(encoderAPinBLast == bitRead(encoder_register, encoderAPinARegNum))?encoderAPos++:encoderAPos--;
  //encoderAPinBLast = bitRead(encoder_register, encoderAPinBRegNum);
}
void encoderB(){
  encoderBPos++;
  //(encoderBPinBLast == bitRead(encoder_register, encoderBPinARegNum))?encoderBPos++:encoderBPos--;
  //encoderBPinBLast = bitRead(encoder_register,encoderBPinBRegNum);
}
void encoderC(){
  (encoderCPinBLast == bitRead(encoder_register, encoderCPinARegNum))?encoderCPos++:encoderCPos--;
  encoderCPinBLast = bitRead(encoder_register,encoderCPinBRegNum);
}
void encoderD(){
  (encoderDPinBLast == bitRead(encoder_register, encoderDPinARegNum))?encoderDPos++:encoderDPos--;
  encoderDPinBLast = bitRead(encoder_register,encoderDPinBRegNum);
}

void servo_test(){

  if(servo_pan_pos >= servo_pan_max){
    servo_pan_dir = false;
  }
  else if(servo_pan_pos <= servo_pan_min){
    servo_pan_dir = true; 
  }

  if(servo_tilt_pos >= servo_tilt_max){
    servo_tilt_dir = false; 
  }
  else if(servo_tilt_pos <= servo_tilt_min){
    servo_tilt_dir = true; 
  }
  if(servo_pan_dir == true){
    servo_pan_pos++; 
  }
  else{
    servo_pan_pos--; 
  }
  if(servo_tilt_dir == true){
    servo_tilt_pos++; 
  }
  else{
    servo_tilt_pos--; 
  }

  servo_pan.write(servo_pan_pos);
  Serial.print("pan pos: ");
  Serial.print(servo_pan.read());

  servo_tilt.write(servo_tilt_pos);
  Serial.print(" Tilt pos: ");
  Serial.println(servo_tilt.read());

}

void sonar_test(){
  Serial.print("Sonar Reading: ");
  Serial.println(analogRead(sonar_pin)); // <= 40 is about 2 feet 147uS/inch

}

void detector_test(){
  if(digitalRead(detector_pin) == HIGH){
    //digitalWrite(48, HIGH);
    Serial.println("Metal Detected");
  }

  /*
  if(detector_cycle >= 500){
   detector_cycle = 0;  
   }
   detector_cycle++;
   
   switch (detector_cycle){
   case 5:
   
   break;
   default:
   
   
   }
   */


}

void initTestWheel(int wheel_dir, int wheel_pwm, int wheel_draw){

  //int wheel_dir = wheel_direction_1;
  //int wheel_pwm = wheel_pwm_1;
  //int wheel_draw = wheel_current_draw_1;
  //bool dir = false;
  digitalWrite(wheel_dir, LOW);
  for(int i = 0; i <= 255; i++){
    analogWrite(wheel_pwm, i);
    delay(5); 
  }
  Serial.println(analogRead(wheel_draw));

  for(int i = 255; i >= 0; i--){
    analogWrite(wheel_pwm, i);
    delay(5);
  }
  Serial.println(analogRead(wheel_draw));
  //dir = true;
  digitalWrite(wheel_dir, HIGH);
  for(int i = 0; i <= 255; i++){
    analogWrite(wheel_pwm, i);
    delay(5); 
  }
  Serial.println(analogRead(wheel_draw));
  for(int i = 255; i >= 0; i--){
    analogWrite(wheel_pwm, i);
    delay(5);
  }
  Serial.println(analogRead(wheel_draw));
}

void testwheel(int time){
  int testwheel_dir = wheel_direction_4;
  int testwheel_pwm = wheel_pwm_4;
  //int dir = HIGH;
  if(dir == HIGH){
    dir = LOW;
  }
  else{
    dir = HIGH; 
  }
  digitalWrite(testwheel_dir, dir);
  for(int x = 0; x < 255; x++){
    analogWrite(testwheel_pwm, x);
    delay(3);
  }
  for(int x = 0; x>= 0 ; x--){
    analogWrite(testwheel_pwm, x);
    delay(3); 
  }
  delay(1000);
  //digitalWrite(testwheel_dir, false);
  //digitalWrite(testwheel_pwm,55);
  //delay(1000);
}

void gogogo(int speed, int direction = 0 ){ // speed 0 - 255, direction 0 = forward

  digitalWrite(wheel_direction_1, direction);
  digitalWrite(wheel_direction_2, direction);
  digitalWrite(wheel_direction_3, direction);
  digitalWrite(wheel_direction_4, direction);
  analogWrite(wheel_pwm_1, speed);
  analogWrite(wheel_pwm_2, speed);
  analogWrite(wheel_pwm_3, speed);
  analogWrite(wheel_pwm_4, speed);

}

void turninplace(int speed, int direction) // speed 0-255, direction 0 = right
{
  digitalWrite(wheel_direction_1, !direction);
  digitalWrite(wheel_direction_2, direction);
  digitalWrite(wheel_direction_3, !direction);
  digitalWrite(wheel_direction_4, direction);
  analogWrite(wheel_pwm_1, speed);
  analogWrite(wheel_pwm_2, speed);
  analogWrite(wheel_pwm_3, speed);
  analogWrite(wheel_pwm_4, speed);



}

boolean sonar_scan(){

  double tempread = 0;
  if(servo_pan_pos >= 130){
    servo_pan_dir = false;
  }
  if(servo_pan_pos <= 70){
    servo_pan_dir = true;
  }
  if(servo_pan_dir == true){
    servo_pan_pos = servo_pan_pos + 2;
  }
  else{
    servo_pan_pos = servo_pan_pos - 2; 
  }


  servo_pan.write(servo_pan_pos);
  Serial.print("Sonar Reading: ");
  tempread = analogRead(sonar_pin);
  delay(50);
  tempread = tempread + analogRead(sonar_pin);
  delay(50);
  tempread = tempread + analogRead(sonar_pin);
  delay(50);
  tempread = tempread + analogRead(sonar_pin);
  tempread = tempread / 4;
  Serial.println(tempread);
  if(tempread <= 45){ // <= 40 is about 2 feet 147uS/inch
    return true;
  }
  else 
    return false;
}

//void turnarc(int speed, int direction, int rad)



