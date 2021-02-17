/*
  Read Analog, Display as PWM of LED
  ------------------------------------------------------------------------------
  
  [ See pinout: https://goo.gl/ijL0of ]
  
  Reads an analog input pin, maps the result to a range from 0 to 255
  and uses the result to set the pulsewidth modulation (PWM) of an output pin.
  Also prints the results to the serial monitor.

  The circuit:
  * potentiometer connected to A2.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  * LED connected from digital pin 1 to ground via resistor

  IMPORTANT!  See the non-default settings below which are required.
     
  Recommended Settings For This Sketch
  ------------------------------------------------------------------------------
  (* indicates non default)
  
  Tools > Board                 : ATTiny13
  Tools > Processor Version     : ATTiny13
  Tools > Use Bootloader        : No (ISP Programmer Upload)
  Tools > Processor Speed       : 9.6MHz Internal Oscillator
  Tools > Millis, Tone Support  : Millis Available, No Tone
  Tools > Millis Accuracy       : 1.666%
* Tools > Print Support         : Dec Only Supported
  Tools > Serial Support        : Half Duplex, Read+Write
     
  Serial Reminder
  ------------------------------------------------------------------------------
  The Baud Rate is IGNORED on the Tiny13 due to using a simplified serial.
  
  The actual Baud Rate used is dependant on the Processor Speed.
  
  9.6MHz will be 57600 Baud
  4.8MHz will be 9600 Baud
  1.2MHz will be 9600 Baud
  
  If you get garbage output:
  
    1. Check baud rate as above
    2. Check if you have anything else connected to TX/RX like an LED
    3. Check OSCCAL (see Examples > 05.Tools > OSCCAL_Helper    
  
  Pinout
  ------------------------------------------------------------------------------
  For ATTiny13 Arduino Pinout: https://goo.gl/ijL0of  
  
  Important: 
    pinMode() must only be used with the "digital pin numbers" 0 .. n
    pins default to INPUT, you do not need to pinMode() to INPUT if you are only
    ever doing an analogRead() from the pin. 
    
    analogRead() must only be used with the "analog pin numbers" A0 .. n
        
  
  Space Saving Tips
  ------------------------------------------------------------------------------
  
  You don't have much flash or ram to work with.  Remember to think about 
  datatype sizes!  Use the options under the Tools menu to reduce capabilities
  hopefully in exchange for more code size (especially Millis and Print).

  Running short on memory, try this tool to help you track down optimisable areas:
    http://sparks.gogo.co.nz/avr-ram-use.html  
  
  Good Luck With Your Itsy Bitsy Teeny Weeny AVR Arduineee
  
*/
#include <EEPROM.h>


const uint8_t POTENTIOMETER_PIN = A2;                 // Analog input pin that the potentiometer is attached to
const uint8_t BATTERY_PIN       = A1;                 // Analog input pin that battery voltage divider is attached to
const uint8_t analogOutPin      = 1;                  // Analog output pin that the voltage output is attached to
const uint8_t jumper_pin        = A3;                 // Analog input pin reading state of programming jumper
const uint8_t eepromLOCATION_low_battery      = 0;    // uint16_t, 16bits for low_battery threshold value
const uint8_t eepromLOCATION_ok_battery       = 2;    // uint16_t, 16bits for ok_battery threshold value
const uint8_t eepromLOCATION_charge_battery   = 4;    // uint16_t, 16bits for charge_battery threshold value

uint16_t low_battery EEMEM    = 812 ; 
uint16_t ok_battery EEMEM     = 831 ; 
uint16_t charge_battery EEMEM = 900 ;  // arduino IDE 1.8.x is broken - just silently ignores that. *sigh*
                                        // upload eeprom manually using avrdude (eeprom.sh included, just change your programmer port etc)
                                        


// for gnd - 10k (tap) 10k 10k + divider (1/3)
// 11.82  = 785
// 12.2   = 812
// 12.5   = 831
// 13.5   = 898
// 13.52  = 900
// 15     = 1000

// 0.015V per bit.
// lp2950 5V reference as VCC. 


uint16_t sensorValue            = 0;        // value read from the pot
uint16_t batteryValue           = 0;        // value read from battery
uint16_t jumper_Value           = 0;        // state of programming jumper 
uint16_t eeprom_battery_low     = 0;        // low voltage threshold from eeprom
uint16_t eeprom_battery_ok      = 0;        // charged battery state from eeprom
uint16_t eeprom_battery_charge  = 0;        // charging battery voltage from eeprom  
uint8_t  outputValue            = 0;        // value output to the PWM (analog out)

uint8_t  serial_delay_counter   = 0; 
void setup() 
{
  Serial.begin(57600);         // NOTICE the baud rate specified is ignored on the T13
                              //  Instead it is hard coded as follows...
                              //  Processors at 9.6MHz ==> 57600
                              //  Processors at 4.8 and 1.2MHz ==> 9600 
  
  pinMode(analogOutPin, OUTPUT);
  batteryValue  = analogRead(BATTERY_PIN);          // read battery level
  if (analogRead(jumper_pin) < 16) {                  // if jumper is grounded completely
    sensorValue = analogRead(POTENTIOMETER_PIN);    // read state of potentiometer 
    if (sensorValue < 50) {                          // if pot is at low setting, we are storing low battery value threshold to eeprom
      EEPROM.write(eepromLOCATION_low_battery, batteryValue);
      EEPROM.write(eepromLOCATION_low_battery + 1, batteryValue >> 8);
    }
    if ((sensorValue >100) && (sensorValue <600)) {
      EEPROM.write(eepromLOCATION_ok_battery, batteryValue);
      EEPROM.write(eepromLOCATION_ok_battery + 1, batteryValue >> 8);
    }
    if (sensorValue > 600) {
      EEPROM.write(eepromLOCATION_charge_battery, batteryValue);
      EEPROM.write(eepromLOCATION_charge_battery + 1, batteryValue >> 8);
    }

  } // if analogRead(jumper_pin) <16 

  eeprom_battery_low    = (EEPROM.read(eepromLOCATION_low_battery + 1) << 8);
  eeprom_battery_low   |= EEPROM.read(eepromLOCATION_low_battery);

  eeprom_battery_ok    = (EEPROM.read(eepromLOCATION_ok_battery + 1) << 8);
  eeprom_battery_ok   |= EEPROM.read(eepromLOCATION_ok_battery);

  eeprom_battery_charge    = (EEPROM.read(eepromLOCATION_charge_battery + 1) << 8);
  eeprom_battery_charge   |= EEPROM.read(eepromLOCATION_charge_battery);  
}


void loop() {
  sensorValue   = analogRead(POTENTIOMETER_PIN);
  batteryValue  = analogRead(BATTERY_PIN);

  // map it to the range of the analog out:
  //  notice here that we don't use the Arduino map() function
  //  because we don't have space for it, instead we just 
  //  divide the output by 4
  //  this works because 1023 / 4 is 255.75 which is 
  //  close enough, and 4 being a binary divisor means
  //  the division will be optimised by the compiler to a bitshift
//  outputValue = sensorValue/4;

  if (batteryValue <= eeprom_battery_low) {
    outputValue = 0; // if battery is too discharged, disable output completely
  }

  if ((batteryValue > eeprom_battery_low) && (batteryValue < eeprom_battery_ok)){
    outputValue = sensorValue/8 ; // if battery is low, output just quarter of the value
  }

  if ((batteryValue >= eeprom_battery_ok) && (batteryValue < eeprom_battery_charge)){
    outputValue = sensorValue/6 ; // if battery is low, output just half the value
  }

  if (batteryValue >= eeprom_battery_charge) {
    outputValue = sensorValue/4; // if battery is on charge, output full value
  }
  
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);

  // print the results to the serial monitor:
//  Serial.print(sensorValue);
//  Serial.write(255);
  if (serial_delay_counter == 0) {
  Serial.write(batteryValue);
//  Serial.write((batteryValue>>4)&&15); 
  Serial.write(batteryValue>>8); 
// Serial.write((batteryValue>>12)&&15); 

//  Serial.println(batteryValue); 
//  Serial.println('V'); 
//  Serial.print(eeprom_battery_low);
//  Serial.print('L');
//  Serial.print(eeprom_battery_ok);
//  Serial.print('O');
//  Serial.println(eeprom_battery_charge);
  }
  serial_delay_counter++;
  
//  delay(50);
  
}
