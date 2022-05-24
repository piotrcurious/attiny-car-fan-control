/*

 PWMLIMIT version (3):

  The PWMLIMIT version is designed for P-channel mosfet drivers.
  Rationale - car fan is designed to work on a switch, switching resistors on each tap, on positive side of circuit.
  This way fan blower should be installed only on first tap, lowest speed. 
  Note that on other settings of the fan switch, voltage will come back from the fan thru p-mosfet body diode back, 
  and because usually lowest setting wire is thinner this creates fire risk in case of damage of wiring harness. 
  use fuse and/or diode in series to solve that. 

  Because P-mosfets are usually weak , and because we work on "1" setting, maximum PWM output is limited. 
  Two methods can be used :
  -low level configuring of timers - higher precission but less compatibility if one changes chips
  -simply dividing/bit shift of analogWrite PWM value - causes crude resolution, but is more compatible. 
  I use this method here.

  Because of very short pulse times, it is recommended to use proper mosfet driver circuit, 
  otherwise there will be large switching losses. 
  I use fast NPN transistor (from old tv - final stage video amplifier for CRT) to turn p-mosfet on, and fast optocoupler , designed for gate drive
  to turn p-mosfet off.
  To deal with both transistors on during transients, i use current limiting resistor .
  Remember to decouple power lines close to mosfet driver as transient states create huge current spikes.
  It is recommended to check the circuit with oscilloscope and install fuse in series, as there is no thermal protection. 
  TODO: perhaps software thermal protection using thermistor and it's raw output would be nice. 
  
  
  Read Analog, Display as PWM of LED
  ------------------------------------------------------------------------------
  
  [ See pinout: https://goo.gl/ijL0of ]
  
  Reads an analog input pin, maps the result to a range from 0 to 255
  and uses the result to set the pulsewidth modulation (PWM) of an output pin.
  Also prints the results to the serial output - in binary mode.
  If You wish. Otherwise You can set it up to use second PWM output instead. 

  The circuit:
  * potentiometer connected to A2.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  * Analog out pin D1
  * Analog out pin D0 / serial TX 
  * battery in - for 12V use + 10K-10K-(A1)-ground

  IMPORTANT!  See the non-default settings below which are required.
     
  Recommended Settings For This Sketch
  ------------------------------------------------------------------------------
  (* indicates non default)
  
  Tools > Board                 : ATTiny13
  Tools > Processor Version     : ATTiny13
  Tools > Use Bootloader        : No (ISP Programmer Upload)
  Tools > Processor Speed       : 9.6MHz Internal Oscillator
* Tools > Millis, Tone Support  : No Millis, No Tone
* Tools > Millis Accuracy       : 50%
* Tools > Print Support         : Dec Only Supported
 *Tools > Serial Support        : Half Duplex, Write Inherits stream
     
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
#include <EEPROM.h> // for storing battery threshold values 
#include <avr/interrupt.h>
#include <avr/sleep.h>

const uint8_t POTENTIOMETER_PIN = A2;                 // Analog input pin that the potentiometer is attached to
const uint8_t BATTERY_PIN       = A1;                 // Analog input pin that battery voltage divider is attached to
const uint8_t analogOutPin      = 1;                  // Analog output pin that the voltage output is attached to
const uint8_t analogOutPin2     = 0;                  // Analog output pin #2 - for voltage gauge output
//const uint8_t jumper_pin        = A3;                 // Analog input pin reading state of programming jumper
// non programmable version , use eeprom.sh to program eeprom with preset thresholds
const uint8_t eepromLOCATION_low_battery      = 0;    // uint16_t, 16bits for low_battery threshold value
const uint8_t eepromLOCATION_ok_battery       = 2;    // uint16_t, 16bits for ok_battery threshold value
const uint8_t eepromLOCATION_charge_battery   = 4;    // uint16_t, 16bits for charge_battery threshold value

const uint8_t THERMAL_PROTECTION_PIN = A3 ; // pin to connect thermistor used for thermal protection to . 
                                            // +5V to 10k thermistor, thermistor to A3, A3 to 10k resistor, resistor to ground. 
                                            // you can use variable 10k resistor to tune threshold or fiddle with code.
const uint16_t raw_temperature_threshold1 = 210; // temperature value for thermal throttle.  
const uint16_t raw_temperature_threshold2 = 300; // temperature value for thermal throttle.  


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
uint16_t last_sensorValue       = 0;        // last value read from the pot
uint16_t batteryValue           = 0;        // value read from battery
uint16_t temperatureValue       = 0;        // temperature value read from thermistor
//uint16_t jumper_Value           = 0;        // state of programming jumper 
uint16_t eeprom_battery_low     = 0;        // low voltage threshold from eeprom
uint16_t eeprom_battery_ok      = 0;        // charged battery state from eeprom
uint16_t eeprom_battery_charge  = 0;        // charging battery voltage from eeprom  
uint8_t  outputValue            = 0;        // value of output to the PWM (analog out)
uint8_t  outputValue2           = 0;        // value of output to the PWM (analog gauge display out)
uint8_t  serial_delay_counter   = 0;         // for sending serial data only each [n] times
                                             // default is each 256 reads 
const uint8_t  powerup_outputValue   = 0;         // start up value. 
bool     first_powerup          = 0;         // if this is first powerup

            // on power up, use this value. only when voltage jumps to charge,
            // or user turns the knob, turn on the output.  

const uint8_t PWM_LIMIT_DIVISOR = 4;    // final output is is divided by that to limit maximum PWM output. 

                                            // divided by 4 to limit max PWM . 20A/4=5A
                                            // 0.022ohm * 5A = 0.11V
                                            // 0.11V*5A = 0.55W to dissipate on mosfet. 
                                            // assuming worst case, fully resistive load, 
                                            // reality is - load is inductive, currents are smaller, but it is difficult to model as load is nonlinear
//#define USE_SOFTWARE_SERIAL_TERMINAL too big for attiny13 :(

#ifdef USE_SOFTWARE_SERIAL_TERMINAL // software serial terminal configuration
#include <SoftwareSerial_IR.h> // included library (github)

// ----------------------terminal protocol configuration options
//#define VT100_compatible_terminal  // define for vt100 terminal. remember to comment out other terminal types
#ifdef VT100_compatible_terminal
#include <VT100.h>
#endif
#define ATMEGA8_TVTERM_TERMINAL // define for atmega8 based TV TERM , author : https://www.serasidis.gr/circuits/TV_terminal/Small_TV_terminal.htm

// ---------------------terminal protocol configuration options

#define SOFTWARE_SERIAL_TERMINAL_TX_PIN 3 // software serial TX pin
//define SOFTWARE_SERIAL_TERMINAL_COLOR // not implemented

#endif USE_SOFTWARE_SERIAL_TERMINAL


//------------------------------------software serial terminal output options
#ifdef USE_SOFTWARE_SERIAL_TERMINAL
SoftwareSerial_IR terminal_port(SOFTWARE_SERIAL_TERMINAL_TX_PIN, false, false); //  TX only library . usage (TX_port_pin,inverse_logic,disable_interrupts)

#ifdef VT100_compatible_terminal
VT100_Control VT100 ;         // original library has a bug - either comment this line out or comment out constructor in library, or pull VT100 from my fork
// without commenting it out in the library you cannot have multiple terminals.
#endif VT100_compatible_terminal
#endif USE_SOFTWARE_SERIAL_TERMINAL
//----------------------------------END of software terminal output options


ISR(WDT_vect) {
} // only wake up from sleep mode

void setup() 
{
//  Serial.begin(57600);         // NOTICE the baud rate specified is ignored on the T13
                              //  Instead it is hard coded as follows...
                              //  Processors at 9.6MHz ==> 57600
                              //  Processors at 4.8 and 1.2MHz ==> 9600 
  
  pinMode(analogOutPin,  OUTPUT);
  pinMode(analogOutPin2, OUTPUT);

// ----------------sleep mode setup 
  // prescale timer to 8s so we can measure current
  WDTCR |= (1<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(1<<WDP0); // 8s
  // Enable watchdog timer interrupts
  WDTCR |= (1<<WDTIE);
  sei(); // Enable global interrupts
  // Use the Power Down sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
// ---           sleep mode setup 

#ifdef USE_SOFTWARE_SERIAL_TERMINAL
                      terminal_port.begin(1200);
#ifdef VT100_compatible_terminal
                      VT100.begin(terminal_port);
                      VT100.reset();
                      VT100.cursorOff();
                      VT100.clearScreen();
                      VT100.setCursor(0, 10);
#endif VT100_compatible_terminal
#ifdef ATMEGA8_TVTERM_TERMINAL
                      terminal_port.write(12); // clear screen
                      terminal_port.write(19);//X pos
                      terminal_port.write(10);
                      terminal_port.write(20);//Y pos
                      terminal_port.write((byte)0x0);
#endif ATMEGA8_TVTERM_TERMINAL
                      terminal_port.print(F("car fan blower controler"));
#endif USE_SOFTWARE_SERIAL_TERMINAL


// non programmable version, ignore jumper
/*
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
*/

// load thresholds from eeprom. this wastes SRAM, but saves flash...
// later on it allows selecting boundaries freely without much acrobatics. 
 
  eeprom_battery_low    = (EEPROM.read(eepromLOCATION_low_battery + 1) << 8);
  eeprom_battery_low   |= EEPROM.read(eepromLOCATION_low_battery);

  eeprom_battery_ok    = (EEPROM.read(eepromLOCATION_ok_battery + 1) << 8);
  eeprom_battery_ok   |= EEPROM.read(eepromLOCATION_ok_battery);

  eeprom_battery_charge    = (EEPROM.read(eepromLOCATION_charge_battery + 1) << 8);
  eeprom_battery_charge   |= EEPROM.read(eepromLOCATION_charge_battery);  
  analogWrite(analogOutPin, powerup_outputValue);
  first_powerup=true;
} //void setup {

     


void pot_to_output() {
  
  // map it to the range of the analog out:
  //  notice here that we don't use the Arduino map() function
  //  because we don't have space for it, instead we just 
  //  divide the output by 4
  //  this works because 1023 / 4 is 255.75 which is 
  //  close enough, and 4 being a binary divisor means
  //  the division will be optimised by the compiler to a bitshift
//  outputValue = sensorValue/4;

// we map output range depending on battery state. 

  if (batteryValue <= eeprom_battery_low) {
    outputValue = 0; // if battery is too discharged, disable output completely
    sleep_mode(); // then enter sleep mode for 8 seconds. 
  }

  if ((batteryValue > eeprom_battery_low) && (batteryValue < eeprom_battery_ok)){
    outputValue = sensorValue/8 ; // if battery is low, output just quarter of the value
  }

  if ((batteryValue >= eeprom_battery_ok) && (batteryValue < eeprom_battery_charge)){
    outputValue = sensorValue/6 ; // if battery is full, output just half the value
  }                               // this division wastes most flash for code. 

  if (batteryValue >= eeprom_battery_charge) {
    outputValue = sensorValue/4; // if battery is on charge, output full value
  }
  
  // change the analog out value:
  if (temperatureValue <= raw_temperature_threshold1) { // if temperature is within spec
  analogWrite(analogOutPin, outputValue/PWM_LIMIT_DIVISOR); 
                                            // divided by 4 to limit max PWM . 20A/4=5A
                                            // 0.022ohm * 5A = 0.11V
                                            // 0.11V*5A = 0.55W to dissipate on mosfet. 
                                            // assuming worst case, fully resistive load, 
                                            // reality is - load is inductive, currents are smaller, but it is difficult to model as load is nonlinear
  } 

  if (temperatureValue > raw_temperature_threshold1) { // if temperature is above spec
  analogWrite(analogOutPin, outputValue/PWM_LIMIT_DIVISOR/2); // half the power output 
                                            // divided by 8 to limit max PWM . 20A/8=2.5A
                                            // 0.022ohm * 5A = 0.055V
                                            // 0.055V*2.5A = 0.1375W to dissipate on mosfet. 
                                            // assuming worst case, fully resistive load, 
                                            // reality is - load is inductive, currents are smaller, but it is difficult to model as load is nonlinear
  } 

  if (temperatureValue > raw_temperature_threshold2) { // if temperature is way above spec
  analogWrite(analogOutPin, 0); // shutdown
                                            // 0
  } 


}

void debug_out() {
  // print the results to the serial monitor. 
  // there are many debug options here.
  // serial and print consume most memory.
  // if You wish to squeeze some extra logic, 
  // remove serial and print completely. 
  
//  Serial.print(sensorValue);
//  Serial.write(255);
//  if (serial_delay_counter == 0) {
//  Serial.write(batteryValue>>8);
//  Serial.write(batteryValue);
//  Serial.print("volt:");
//  Serial.write(128+1);
//  Serial.write( (127 & (batteryValue>>7)) );
//  Serial.write( (127 & batteryValue));
  
//  Serial.write((batteryValue>>4)&&15); 
 // Serial.write(batteryValue>>8); // currently two binary bytes with battery value are out. 
                                 // note this relies on delay implied by sending the value
                                 // only once per 256 reads 
                                 // as this defines which byte is first
                                
// Serial.write((batteryValue>>12)&&15); 

//  Serial.println(batteryValue); 
//  Serial.println('V'); 
//  Serial.print(eeprom_battery_low);
//  Serial.print('L');
//  Serial.print(eeprom_battery_ok);
//  Serial.print('O');
//  Serial.println(eeprom_battery_charge);
//  }  // if (serial_delay_counter == 0) {
//  serial_delay_counter++;       // increment serial delay counter, each 256 it overflows
                                // use if() logic here for other ways of countdown. 
//  if (serial_delay_counter == 32) { 
//    serial_delay_counter = 0; 
//    }
//  delay(50);                // do not use , requires millis. 

#ifdef USE_SOFTWARE_SERIAL_TERMINAL
#ifdef VT100_compatible_terminal
                      VT100.setCursor(2, 0);
                      VT100.clearLine();
#endif VT100_compatible_terminal
#ifdef ATMEGA8_TVTERM_TERMINAL
                      terminal_port.write(19);//X pos
                      terminal_port.write((byte)0x0);
                      terminal_port.write(20);//Y pos
                      terminal_port.write(2);
#endif ATMEGA8_TVTERM_TERMINAL
                      terminal_port.print(F("voltage:"));
                      terminal_port.print(batteryValue);
                      terminal_port.print(F(" "));

                      terminal_port.print(F("low:"));
                      terminal_port.print(eeprom_battery_low);
                      terminal_port.print(F(" "));

                      terminal_port.print(F("ok:"));
                      terminal_port.print(eeprom_battery_ok);
                      terminal_port.print(F(" "));

                      terminal_port.print(F("charge:"));
                      terminal_port.print(eeprom_battery_charge);
                      terminal_port.print(F(" "));

#endif  USE_SOFTWARE_SERIAL_TERMINAL

#ifdef USE_SOFTWARE_SERIAL_TERMINAL
#ifdef VT100_compatible_terminal
                      VT100.setCursor(3, 0);
                      VT100.clearLine();
#endif VT100_compatible_terminal
#ifdef ATMEGA8_TVTERM_TERMINAL
                      terminal_port.write(19);//X pos
                      terminal_port.write((byte)0x0);
                      terminal_port.write(20);//Y pos
                      terminal_port.write(3);
#endif ATMEGA8_TVTERM_TERMINAL
                      terminal_port.print(F("knob:"));
                      terminal_port.print(sensorValue);
                      terminal_port.print(F(" "));
#endif  USE_SOFTWARE_SERIAL_TERMINAL


}
void loop() {
  sensorValue       = analogRead(POTENTIOMETER_PIN);      // read sensor
  batteryValue      = analogRead(BATTERY_PIN);            // read battery
  temperatureValue  = analogRead(THERMAL_PROTECTION_PIN); // read temperature (raw)
  
     if (!first_powerup) {
      pot_to_output();
     }
     else{
//      if ( analogRead(BATTERY_PIN) >= eeprom_battery_charge){
        if ( batteryValue >= eeprom_battery_charge){ // 12 bytes shorter
        first_powerup=false; 
              analogWrite(analogOutPin,192/PWM_LIMIT_DIVISOR); // hard start - wind up the fan
              delay(2000); // start up the fan on high speed for 2 seconds
          }
       if ( analogRead(POTENTIOMETER_PIN) >= 950) {  //manual start - turning knob high.
        first_powerup=false;
       }
       }
 
//    debug_out();
      //-------------------analog gauge out code - eyecandy to visualise battery state
     if (batteryValue > eeprom_battery_low) { // if battery value above low threshold
      if ((batteryValue-eeprom_battery_low ) < 255) { // if battery value is within the range of the display
            analogWrite(analogOutPin2,batteryValue-eeprom_battery_low);   // tinker to map the scale up to your liking. 
      } else {
      analogWrite (analogOutPin2,255); // else display max value}
      }
     }
     }
