# Arduino-Mega-for-Ascent-Smart-Climber

Contributors
```
Tim Hooper
Elena Peng
```
Goal
```
To detect and adjust climber angle, communicate commands between the control board 
and climber, store performance data, reset, and process signals from remote buttons.
```
- Angle/ Inclination
```
Ten levels (level 0-9) with maximum angle = 74.8° and minimum angle = 55°.
```
- Communication and Storage
```
Process commands connect to the Arduino UNO motor controller and to the Bluetooth 
adapter linked to the tablet. Accumulate data (time and distance) when the tablet 
connected via Bluetoothe requests no update (tablet app is not on performacne page).
```
- Reset Climber
```
Move the handles and pedals back to the starting location. Adjust the climber to the 
most upright angle (74.8°).
```
- Remote Buttons on Climber Handles
```
(1) Long press: Emergency stop. 
(2) Short click: Direct the application screen to “Voice Control Page”. 
(3) Double click: Press microphone button and ready to receive commands. 
```
