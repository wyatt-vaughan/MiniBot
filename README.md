# MiniBot
Super cheap super tiny bots for....well I don't know yet

## Overview
There's 2 main components to this project. The MiniBots and the Coordinator

### The MiniBots
Very small robots powered by 2 super small stepper motors (PMO8-2) and powered
by a custom PCB and 150mAh lipo. The PCB has all the required functionality built
in and fits neatly in the base of the minibot. Onboard is an ESP32-C3, so these
aren't just dumb bots. They each have serious computation power and help to
distribute the complexity required for mass coordination of bots. The bot is 
fully 3D printed and only requires 2 M2x5mm bolts to mount the motors, and 
2 9mm ID x 12mm OD o-rings for the tire on the wheels. Total hardware cost 
excluding pcb is around $4.

### The Coordinator
This component consists of the main processing hardware (Raspberry PI plus ESP32) as
well as a board. The board has some electromagnets at a few key points which are
used to calculate accurate coordinates for each piece.
The intended use case here is as a robotic chess board, so you'll note those dimensions
align rather nicely for this. And there will probably be a chess clock on here too for
turn coordination.

## How does it work?
That's a long answer. Not really, I just don't feel like explaining it yet when I
haven't finished prototyping. So TBD :)

## PCB
It isn't done yet, so don't order it

## CAD
It isn't done yet but it is pretty close. At least for the MiniBots.
https://cad.onshape.com/documents/4f8eaef75458146767928ab5/w/f159d6d65b9091531e1ead34/e/13b51722310f27710438c727

## Firmware
It is very very far from done
