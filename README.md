# attiny-car-fan-control
Tiny controler for cabin blower fan 
Code is in bit trashy state, but it works, it requires just cleanup.

Idea is that car interior fan/blower in most cars use passive resistor.
This can be replaced by attiny13 with PWM driving either mosfet or another controller
(which is what i do) over analog wire. I chosen second chip solution because this allows the second
chip to run software PWM with variable frequency - user can set it by trimmer pot - so PWM resonance whine
is experimentally removed or put within range tolerated by user.

motivation/goals :
1) replace passive resistor which often breaks and is very expensive and difficult to source
 (much more than two attiny13 with mosfet and all parts, usually over 20e)
2) increase efficiency by using PWM / switch mode regulation.
small car fan draws about 15A, at 12V this is 180W. Most users want to run it on "1" or "2", which
means about 5A or 10A of current, so 60W or 120W. Usually this means 2/3 or 1/2 of power is wasted in resistor, so,
40W or 60W respectively. This is about as much as both day lights consume. (2x20W lightbulbs) on "1" setting(!) .
For 1mln of cars, this means 40MW of wasted power, about one hydroelectric power plant, and assuming 60% alternator efficiency,
this means  56MW, while adding up only 40% of fuel to energy conversion ratio, this means 89.6MW. That is a lot of wasted fuel.
3) while we use uC , we can add additional functions, like battery protection.
Surprisingly most cars use passive resistor, even past year 2000 ones.

attiny13 does following job :
* detects initial start up. This way fan does not start up when you turn ignition key.
 uC detects charging voltage , and starts up fan only then. 
 This avoids disturbing driver and discharging battery on cold days.
 Also like how many times one just wants to turn on the radio or fiddle with mirrors, and not really change fan settings...
* detects knob being put to 90% setting during initial start up - it assumes manual start up by user is requested by such action.
After start up :
* when battery level drops below 13.5 (charging) , reduces max fan speed to 80%
* when battery level drops below 12.7 (full charge), reduces max fan speed to 50%
* when battery level drops below 12.3V (60% discharge), disables fan completely.
* (optional) outputs voltage (or any other parameter) via serial out. 
 I settled on "midi like" protocol (7bit, and 8th bit reserved for
 "register number" and synchronization packet.
 in the "nodebug" this is disabled by commenting out as fight for memory space was priority. 

It all fits fine into attiny .

Additionally to ease up assembly using recycled parts (like mosfets from old motherboards, resistors from unknown sources),
there is version allowing programming EEPROM using "jumper(A3, with 1k pullup to 5V)" pin .
To do that , one needs to :
* program attiny with 'learning' version

* set desired voltage to board using f.e. regulated power supply,  mere potentiometer  (disconnect fan if using pot),
or just charge or discharge battery to desired levels
* set the knob of the board to max - to set 13.5V threshold (charge)
* disconnect power
* connect jumper to ground
* connect power back briefly, device will learn and store into EEPROM
* disconnect jumper

-set desired voltage like above, to 12.7 (battery full charge)
-set the knob of the board to middle 
-disconnect power
-connect jumper to ground
-connect power back briefly, device will learn

-disconnect jumper

-set desired voltage to 12.3V (60% discharge)
-set knob to middle
-disconnect power
-connect jumper to ground
-connect power back , device will learn
-disconnect jumper.

-remove learned chip from socket, insert to programmer, read eeprom using avrdude. 
-program with normal code , then write eeprom with avrdude using learned data.

repository contains: 
code, helper script, and helper 'reciever' code for arduino pro/anything else with lcd.
I did use "DIY Attiny" board set , lto compiler , intrc osc @ 9.6mhz .
remeber to burn fuses/bootloader before starting to curse on anything. 
reciever code works better with "attiny core" board manager (built in arduino pro displays floats in full precission)

the "nodebug" .ino is most final version, previous require re-make basing on that version, introducing #ifdefs etc.
maybe i will do it someday.
Same circuit, using just low gate voltage mosfet and perhaps no pot, can be used for other devices in the car,
allowing to cut them off if battery gets too discharged, like rear window heating, parking lights (for them instead of just dim,
uC can flash them at decreasing duty cycle to keep car visible longer even when battery goes flat) , trunk light , cabin light,
radio, lighter socket, etc. etc. - whole circuit is very cheap, and can be used on each line separately.

Only lighter socket and radio require better voltage divider (or divider switched on by spare pins of uC) to prevent draining
battery when ignition is off. Actually even though ADC requires 10k source , if one disables ADC input , there is no drain,
so voltage divider can be 300k ohm, with small capacitor on the pin to make ADC happy. This requires less frequent ADC reads
and extra code though.

Enjoy
