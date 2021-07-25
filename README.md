# FLyguy - Programmable UAV   
## Introduction  
We hope to equip our UAV with the WE-I PLUS development board which includes a camera and AI processing capabilities. Our goal is to design an intelligent UAV system that can perform object detection and respond in real time. Using ARC iotdk board as the main control module for peripheral sensors and motors. Enabling UAV attitude or showing captured image of person on local PC monitor. 

To enable common features such as hovering, landing, emergency shut down, we will use the ARC iotdk board as the main controller to control peripheral components. The Himax camera on the WE-I development board can capture images of the UAV’s surroundings and feed them into a CNN model, enabling backend AI applications. 

We leverage this exceptional advantages to implement an intelligent UAV, which can be used to perform gestures classification and person detection. Once detect the gestures, the corresponding command is send to main controller and modify UAV attitude. Likewise, if the person is being detected, image of he/she will be transmitted to PC by radio links.   

## Demo Video
* <https://drive.google.com/drive/folders/12FVz6vw5qo8o85kHHyj3vj9wvGWjw89_?fbclid=IwAR3bbGfDvNibZDc32LbP6FHP-WIRA9qwwHd0a5z1ahYwqFZcv2dVruvvwSE>
## System Architecture
![image](https://github.com/U3807/FLyguy/blob/main/Pics/1.PNG)  
![image](https://github.com/U3807/FLyguy/blob/main/Pics/2.PNG)  
## Required Hardware
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
* Download source code from the link above
* Put our folder Gesture and folder Person_detection in ``` himax_tflm\Synopsys_WEI\User_Project\```
## User Manual 
## Gesture data collection and training
* Collect pictures from himax camera for machine learning datasets in  ``` himax_tflm\Synopsys_WEI\User_Project\Camera```
* Press B on keyboard and the pixel value of taken picture will show on the teraterm terminal
* Save those printed value in terminal and save as  ```Python\Dataset\xxx.txt```
* Start training in ```Python\training``` and burn the tensorflow_lite model in the himax board   
* Press reset button to inference on Himax WE-I Board and the predicted direction will show on teraterm terminal  
![image](https://github.com/U3807/FLyguy/blob/main/Pics/predicted.PNG) 
## Peson detection and take photo
* Setup the radio connection
* Run the code in ```Python\sendpic```
* If Himax WE-Iborad detect a person's face, the person's picture will be send through UART and displayed on screen 

