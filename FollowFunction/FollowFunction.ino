// Include all the needed libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

//Temp var
const int leftWheel = 13;
const int rightWheel = 12;

// Location variables
double myLocation[2];
double phoneLocation[2];

// Pins for echolocation
const int trigPinLeft = 8, echoPinLeft = 9;
const int trigPinRight = 10, echoPinRight = 11;
// Uncomment when a 3th HC-SR04 is available along with the other Front-functions
// const byte trigPinFront = 9, echoPinFront = 10;

// Variables for echolocation
long duration;
float distance;

// Constants for GPS
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
 
// Assign a unique ID to this sensor at the same time
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() 
{
  // Setup temps
  Serial.begin(115200);
  pinMode(leftWheel, OUTPUT);
  pinMode(rightWheel, OUTPUT);

  // Setup echolocation
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  /* pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT); */

  // Setup GPS
  ss.begin(GPSBaud);

  myLocation[0] = 0;
  myLocation[1] = 0;

  // Setup compass
  if(!mag.begin())
  {
    Serial.println("No HMC5883 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("Starting FollowFunction");
}
 
void loop() 
{ 
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      sensors_event_t event; 
      mag.getEvent(&event);
     
      // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
      // Calculate heading when the magnetometer is level, then correct for signs of axis.
      float heading = atan2(event.magnetic.y, event.magnetic.x);
      
      // Add 'Declination Angle', which is the 'Error' of the magnetic field in your location.
      float declinationAngle = -0.01;
      heading += declinationAngle;
      
      // Correct for when signs are reversed.
      if(heading < 0)
        heading += 2*PI;
        
      // Check for wrap due to addition of declination.
      if(heading > 2*PI)
        heading -= 2*PI;
       
      // Convert radians to degrees for readability.
      float myAngle = heading * 180/M_PI;

      // Set locations of the arduino and the phone into variables
      setMyLocation();
      setPhoneLocation();

      // Calculate the angle between the two locations
      double followAngle = calcAngle(myLocation, phoneLocation);

      Serial.print("my angle: ");
      Serial.print(myAngle);
      Serial.println("°N");
      Serial.print("follow angle: ");
      Serial.print(followAngle);
      Serial.println("°N");
      
      // Check where the arduino is going and adjust our position accordingly
      if ((myAngle + 10) > followAngle && (myAngle - 10) < followAngle)
      {
        goStraight();
        //Serial.println("\nForward\n");
      }
      else if (myAngle > followAngle && myAngle - followAngle < 180)
      {
        goLeft();
        //Serial.println("\nLeft\n");
      }
      else 
      {
        goRight();
        //Serial.println("\nRight\n");
      }
      
      delay(100);
    }
  }
  
}

void setMyLocation()
{
  if (gps.location.isValid())
  {
    myLocation[0] = gps.location.lat();
    myLocation[1] = gps.location.lng();
  }
  else
  {
    myLocation[0] = 50.741846;
    myLocation[1] = 4.020175;
  }
}

void setPhoneLocation(){
  phoneLocation[0] = 50.741983; 
  phoneLocation[1] = 4.020102;
}

double calcAngle(double p1[], double p2[])
{
  double result = atan((p2[0]-p1[0])/(p2[1]-p1[1])) * 180 / PI;
  if (result < 0)
    result += 360;
  return result;
}

boolean checkCollisionLeft()
{
  // Clears the trigPin
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPinLeft, HIGH);
  // Calculating the distance
  distance = duration*0.034/2;
  if(distance < 20){
    return false;
  }
  else{
    return true;
  }
}

boolean checkCollisionRight()
{
  // Clears the trigPin
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPinRight, HIGH);
  // Calculating the distance
  distance = duration*0.034/2;
  if(distance < 20){
    return false;
  }
  else {
   return true;
  }
}

/* boolean checkCollisionFront()
{
  // Clears the trigPin
  digitalWrite(trigPinFront, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPinFront, HIGH);
  // Calculating the distance
  distance = duration*0.034/2;
  if(distance < 20)
    return false;
  else
   return true;
}*/

void goStraight()
{
  //if(checkCollisionFront())
  //{
    digitalWrite(leftWheel, HIGH);
    digitalWrite(rightWheel, HIGH);
  /*}
  else if(checkCollisionLeft()){
    goLeft();
  }
  else {
    goRight();
  }
  */
}

void goLeft()
{
  if(checkCollisionLeft()) {
    digitalWrite(leftWheel, LOW);
    digitalWrite(rightWheel, HIGH);
  }
  else {
    goStraight();
  }
}

void goRight()
{
  if(checkCollisionRight()) {
    digitalWrite(leftWheel, HIGH);
    digitalWrite(rightWheel, LOW);
  }
  else {
    goStraight();
  }
}
