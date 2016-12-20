# 33C3 Hot Wire 
Code and design files for the wireless sensor designed for a 25m long "hot wire" game.
Demo Video (sorry for German): https://www.youtube.com/watch?v=ximVu1-kXSI

## Usage
The sensor is turned on by pressing the button on the back. There is a startup melody that indicates successful startup. 

If you hold the butten for > 5s, the leds flash blue. If the button is released now, the sensor shuts down and enters ultra low current mode. If you keep pressing, the sensor enters a self-test mode testing all it's components. At the beginning of the self-test, it displays the battery voltage (1,1V -> 1,5V = Red -> Green).

## Hardware
The sensor uses a MSP430 cpu, a RFM12B radio module, a Microchip StepUp and a BMA280 Accelerometer. Additionaly there are some RGB led's, a Piezo buzzer and a mechanical button on the back.

Power consumption
- Deep Sleep: ~70µA (67µA for the RFM12B...)
- Active Mode: ~500µA (127µA for the BMA280, 67µA for the RFM12B)
- Max. current (leds + buzzer + radio): 300mA
 
The project was design using Altium Designer 16.1. Sorry if you can't open the design files, there is also a pdf export. I'll try to export the design as an eagle project, not sure if this works. 

## Software
The firmware is written using the MSP430 Energia toolchain. It uses a ported arduino library for the RFM12B by Felix Rusu. The header files for the BMA280 are written by Helge Wurst. 

The MSP430G2553 used in this project has a quiet small flash & RAM.
The code uses 14.964 Bytes (91%) of the flash memory and 414 Bytes (80%) of the RAM. Probably it could be way less...

## ToDo

- "menu" for choosing different game modes (now all are active)
- fix ACK transmission
- implemement more game modes using the accelerometer

