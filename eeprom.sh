#!/bin/bash
avrdude -c usbasp -p attiny13  -U eeprom:w:eeprom.eep:i
