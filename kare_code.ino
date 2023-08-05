/********************************************
     libraries
********************************************/

#include <PID_v1.h>  //PID by Brett Beauregard - automatic PID calculator 
#include <List.hpp>  //List by Niklas Kaaf - only needed if using average values
#include <DoubleLinkedList.hpp>  //""
#include <OneWire.h>  //OneWire by Jim Studt - library for DS18S20 use 
#include <DallasTemperature.h>  //DallasTemperature by Miles Burton - simplifies OneWire use


/********************************************
     variables
********************************************/
//pin constants for temperature control
#define TEMP_PIN 3
#define TEMP_RELAY 8  //3rd relay
//pin constants for pressure control
//#define AIR_VALVE 4  //1st relay
#define AIR_PUMP 11  //pwm pin
#define FSR_PIN A0

OneWire oneWire(TEMP_PIN);  //initiates DS18S20 temperature sensor
DallasTemperature sensors(&oneWire);  //initiates sensors object for DallasTemperature
DeviceAddress tempDeviceAddress;

//global variables for temperature control
double targetTemperature;  //initiate variable
double temperature=0.0;  //temperature sensor reading is stored here
int delayInMillis=750;
unsigned long pastMillis = 0;  //use for keeping track fo 750 ms time intervals between temperature readings

//DoubleLinkedList<double> pastVals;  //only needed if using average values

//PID set up for temperature control
double targetPressure, Input, Output;  //initiates variables for PID

//Specify the links and initial PID tuning parameters
double kP=30, kI=40, kD=0.3; //default: 30,40,.3
PID pid(&Input, &Output, &targetPressure, kP, kI, kD, P_ON_E, DIRECT);  //initiates PID object


/********************************************
     functions
********************************************/

void monitorPressure(double targetPressure, double force, bool includeBounds = false, bool includePWM = false, bool includeTarget = false, bool monitorTemperature = false){
//manage printing of pressure
  //pressure monitoring
  // lower bound and upper bound set serial plotter scale to +- 0.1N of target
  if(includeBounds){
    Serial.print("hb_pressure:");
    Serial.print(targetPressure+.1);
    Serial.print(",");
    Serial.print("lb_pressure:");
    Serial.print(targetPressure-.1);
    Serial.print(",");
  }
  if(includePWM){
    //pwm output monitoring
    Serial.print("pwm_output:");
    Serial.print(Output);
    Serial.print(",");
  }

  //target and actual pressure plots
  if(includeTarget){
    Serial.print("targetPressure:");
    Serial.print(targetPressure);
    Serial.print(",");
  }
  Serial.print("force:");

  if(monitorTemperature){
    Serial.print(force);
    Serial.print(",");
  } else{
    Serial.println(force);
  }  

}

void monitorTemperature (double targetTemperature, double temperature, bool includeBounds = false, bool includeTarget = false){
//manage printing of temperature;
  if(includeBounds){
    // lower bound and upper bound set serial plotter scale to +- 5C of target
    Serial.print("lb_temperature:"); 
    Serial.print(targetTemperature-5);
    Serial.print(",");
    Serial.print("hb_temperature:");
    Serial.print(targetTemperature+5);
    Serial.print(",");
  }

  //target and actual temperature plots
  if(includeTarget){
    Serial.print("target_temperature:");
    Serial.print(targetTemperature);
    Serial.print(",");
  }
  Serial.print("celsius:");
  Serial.println(temperature); 
}

/********************************************
     set up and loop
********************************************/

void setup()
{
  //set pin modes
  pinMode(TEMP_PIN, INPUT);
  pinMode(TEMP_RELAY, OUTPUT);
  pinMode(AIR_PUMP,OUTPUT);
  targetTemperature = 34; //set target temperature here

  double force = convertForce(analogRead(FSR_PIN));  //reads force
  Input = force;  //sends force reading to PID
  targetPressure = 1;  //set target pressure here

  //turn the PID on
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(100, 255);  //pwm signals; 100 is the highest value that doesn't inflate the bag
  
  //start serial monitor
  Serial.begin(19200);

  //start temperature sensor
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, 12);
  //for non blocking
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  pastMillis = millis();
}

//************************//

void loop()
{
 // sensors.requestTemperatures();  //signals for temperature reading

    //************    pressure control    ************//
  double force = convertForce(analogRead(FSR_PIN));  //reads force
  Input = force;  //sends force reading to PID

  pid.Compute();  //runs PID 
  analogWrite(AIR_PUMP, Output);  //sends PID pwm calculation to air pump

  //************    temperature control    ************//

  if (millis() - pastMillis >= delayInMillis) 
  //used for non-blocking temperature readings - collects a reading every 750 ms
  {
    temperature = sensors.getTempCByIndex(0);
    sensors.requestTemperatures(); 
    pastMillis = millis(); 
  }
  
  //delay(1);

    //temperature relay control    **********
  if(temperature > targetTemperature+0.1){ 
    //turns current off if temperature reading is above target temperature
    digitalWrite(TEMP_RELAY, HIGH);
  }else{ 
    //turns current on if temperature reading is below target temperature
    analogWrite(TEMP_RELAY, LOW);
  }

  //************    monitoring    ************//
  //parameters+defaults: double targetPressure, double force, bool includeBounds = false, bool includePWM = false, bool includeTarget = false, bool monitorTemperature = false
  monitorTemperature(targetTemperature, temperature);
  //parameters+defaults: double targetTemperature, double temperature, bool includeBounds = false, bool includeTarget = false
  monitorPressure(targetPressure, force, true, true, true, true);

}

/********************************************
     addtl functions
********************************************/

double convertForce(int analogReading) {
  //Voltage = Vcc * R / (R+FSR) where R 10K and Vcc = 5V
  // FSR = (VCC-V)*R / V
  double fsrVoltage = map(analogReading, 0, 1023, 0, 5000);  //analogRead outputs out of 1023, voltage reading is up to 5000mV
  double fsrResistance = 5000 - fsrVoltage;
  fsrResistance *= 10000;  //for the 10K resistor
  fsrResistance /= fsrVoltage;
  double fsrConductance = 1000000;  //microohms
  fsrConductance /= fsrResistance;

  double fsrForce;
  //approximation from FSR guide graphs
  if (fsrConductance <= 1000) {
    fsrForce = fsrConductance / 80/2;
  } else {
    fsrForce = fsrConductance - 1000;
    fsrForce /= 30;
  }
  return fsrForce;
}

double calculateAvgForce(double force, int amountCollected, DoubleLinkedList<double> pastVals){ //function can delay reaction
  //where force = fsr reading and amountCollected = amount of values to average
  int size = pastVals.getSize();
  if( size < amountCollected ){ //if there are less than the number of values you want to collect saved
    pastVals.add(force);
  } else{
    //this will run most of the time - deletes oldest value and adds newest value
    pastVals.removeFirst();
    pastVals.add(force);
  }
  double sum  =0;
  for (int i = 0; i < size; i++) {
    sum += pastVals.getValue(i);
  }
  return sum/size;
}
