## Robokid robot vehicle software
---

### History
The original Robokid robot was built in 2008 for use by Primary P6/P7 pupils in 
Scottish schools.  The project ran for over 6 years and was done by over 17,000 pupils.
Typically, the project consisted of six activities over a six week period as a set of 
technical, art, and competition activities.

Due to being volunteered by my daughter I have resurrected the project and converted it
into an afternoon session for P4/P5 pupils.

This repository hold the robot software that has been updated.

### Hardware

* Freescale MC9S08AW60 8-bit microprocessor
* 2 MFA 918D DC motors
* 2 MPC17511 H-bridge controllers
* 2 KTIR0A21DS IR line sensors
* 2 KRC011 side-looking IR sensors
* KCDC03-106 dual 7-segment LED display
* Piezo sounder (for 1 octave worth of tones)
* 4 Push buttons
* 4 LED indicators
* 4 potentiometers (for input of parameters)
* 4 3-pin screw-terminal connectors for switch bump sensors (3/front,1/back)
* USB type-B socket with FT232RL controller
* TSOP34838 IR receiver
* 4 NiMH rechargeable batteries

### Tools

* Windows 10
* Freescale Codewarrior V6.3.2 IDE + C compiler
* PEmicro USB Multilink Interface (chip programming)
* Freescale USBDM interface (chip programming)
* Dlyfull B4 Battery Tester (for NiMH batteries)

### Notes
1. Codewarrior works on Linux Mint 22.2 (with Wine), but no success with chip programming tools (March 26)