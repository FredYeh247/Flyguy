# FLyguy - Programmable UAV   
## Introduction  
We hope to equip our UAV with the WE-I PLUS development board which includes a camera and AI processing capabilities. Our goal is to design a smart UAV system that can perform object detection and respond in real time. Using ARC iotdk board as the main control module for peripheral sensors and motors. Enabling obstacle avoidance and autonomously track moving targets. 

To enable common features such as hovering, landing, emergency shut down, we will use the ARC iotdk board as the main controller to control peripheral components.  

The Himax imaging camera on the WE-I development board can capture images of the UAV’s surroundings and feed them into a CNN model, enabling obstacle avoidance and autonomously track moving targets. 

## Demo Video
link
## System Architecture
![image](https://github.com/U3807/FLyguy/blob/main/Pics/1.PNG)  
![image](https://github.com/U3807/FLyguy/blob/main/Pics/2.PNG)  
## Hardware
* Main controller : ARC iotdk board
* Object detection board : WE-I Plus
* RC controller : AT9S 
* Radiolink : R9DS
* Ultrasonic sensor : HC-SR04
* Quad frame : QAV250
* Electronic Speed Control: BLHeli-32
* Motor : Velox V2306
## Required Software
* <https://github.com/foss-for-synopsys-dwc-arc-processors/embarc_osp/tree/embarc_mli>
* <https://github.com/HimaxWiseEyePlus/himax_tflm#training-your-own-model>
## User Guide

