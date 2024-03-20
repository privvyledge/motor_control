# Knob

Control the position of a RC (hobby) [servo motor](http://en.wikipedia.org/wiki/Servo_motor#RC_servos) with your Arduino and a potentiometer.

This example makes use of the Arduino `Servo` library.

## Hardware Required

* an Arduino board
* Sparkmax ESC 
* 10k ohm potentiometer
* hook-up wires

## Circuit

ESC have two wires (out of 4 Pin): ground(Pin 4), and signal(Pin 3). The ground wire is typically black or brown and should be connected to a ground pin on the board. The signal pin is typically Red or orange and should be connected to pin 9 on the board.

The potentiometer should be wired so that its two outer pins are connected to power (+5V) and ground, and its middle pin is connected to analog input 0 on the board.

