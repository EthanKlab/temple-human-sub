#include <Wire.h>
//#include <Adafruit_Sensor.h> // figure out where to get libraries
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
//#include "MS5837.h"
#include <Servo.h>
//#include <PID_v1.h>

// variables to control pid's
//
double setpointPitch, inputPitch, outputPitch;
double setpointRoll, inputRoll, outputRoll;
double setpointYaw, inputYaw, outputYaw;

// name servos
//
Servo dive1;
Servo dive2;
Servo rudder1;
Servo rudder2;

// set up pid control
//
PID pitchPid(&inputPitch, &outputPitch, &setpointPitch,2,5,1, DIRECT);
PID rollPid(&inputRoll, &outputRoll, &setpointRoll,2,5,1, DIRECT);
PID yawPid(&inputYaw, &outputYaw, &setpointYaw,2,5,1, DIRECT); //PID does not name a type

int maxDepth = 5.4; // max depth of isr pool is 22ft or 6.7m

// initializing variables
//
int maxRollDev = 0;
int minRollDev = 0;
int maxYawDev = 0;
int minYawDev = 0;
int maxPitchDev = 0;
int minPitchDev = 0;

MS5837 sensor;  //depth sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55); // gyro/accel/magnet

void setup() {
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  if(!bno.begin()) // initialize gyro board

  Wire.begin();
  sensor.init(); // initialize depth sensor
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);   //sets up external clock source
  
// setpoints for PID's. 
  setpointPitch = 0;
  setpointRoll = 0;
  setpointYaw = 0;
  
  pitchPid.SetMode(AUTOMATIC);
  rollPid.SetMode(AUTOMATIC);
  yawPid.SetMode(AUTOMATIC);

  // attach servos to pwm pins
dive1.attach(3);
dive2.attach(5);
rudder1.attach(6);
rudder2.attach(9);


}

void loop() {

  sensors_event_t event; 
  bno.getEvent(&event);     // gets readings from gyro board

  Serial.print("Depth: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" m");

  if (event.orientation.y > maxYawDev || event.orientation.y < minYawDev){
    inputYaw=event.orientation.y;
    yawPid.Compute();
    rudder1.write(outputYaw);
    rudder2.write(outputYaw);
    if (event.orientation.x > maxRollDev || event.orientation.x < minRollDev){
      inputRoll=event.orientation.x;
      rollPid.Compute();
      dive1.write(outputRoll);
      dive2.write(-(outputRoll));
    }
  }
  else if (sensor.depth() > maxDepth){
    depthCorrection();
  }
  
  else if (event.orientation.z > maxPitchDev || event.orientation.z < minPitchDev){
    inputPitch=event.orientation.z;
    pitchPid.Compute();
    dive1.write(outputPitch);
    dive2.write(outputPitch);
     if (event.orientation.x > maxRollDev || event.orientation.x < minRollDev){
      inputRoll=event.orientation.x;
      rollPid.Compute();
      rudder1.write(outputRoll);
      rudder2.write(-(outputRoll));
    }
  }
}

void depthCorrection(){
  
}
