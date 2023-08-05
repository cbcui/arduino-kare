# arduino-kare
This is the code for an early prototype of the University of Chicagoo Tian Lab's Kare system. Kare is an artificial kangaroo care device system for premature infants. This code contains closed-loop feedback systems for a heat mat and an airbag system.

The heating control system is an on/off feedback control system activated by relay. The pneumatic control system is a PID feedback control system activated by PWM. Note that values PID gain values and Force Sensing Resistor scalars need to be tuned for optimal usage.

Intended hardware:

Arduino Uno Rev3
ZR370-02PM Micro Air Pump 
Ohmite FSR01 Force Sensing Resistor
DS18S20 Temperature Sensor
Arduino Four-Relay Shield containing Finder 30.22.7.005.0010s
SparkFun COM-11289 heat mat

Circuit block diagram:
![circuit](https://github.com/cbcui/arduino-kare/assets/80178830/198d3aef-8cce-46f1-b46a-41f09ef47db5)
