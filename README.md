# Assignment-



## **Goals and Status**



   -  [x] The button states are read every 100ms using an RTOS task. This ensures smooth debouncing and timely detection of button presses.

   - [x] **Red LED** shall TURN ON if **switch 1** is pressed.
   -  [x] **Green LED** shall turn ON if **switch 2** is pressed.
   - [x] If both switches are pressed at the same time, the program maintains the current state of both LEDs without changing their ON/OFF status. This prevents toggling when both buttons are pressed.
   - [x] **If no button is pressed, LEDs shall turn on-off one by one until a button press is detected.**
   - [x] **Display the LED status and button input status on a 16x2 LCD at all times.**
     - Button 1 (Switch 1) state: Pressed/Not Pressed
     - Button 2 (Switch 2) state: Pressed/Not Pressed
     - LED 1 (Red LED) state: ON/OFF
     - LED 2 (Green LED) state: ON/OFF

 
---


## **1.3 Guidelines**

### **1.3.1** 
- [x] Only use **FreeRTOS components** to share data between tasks, such as:
  - Mutex
  - Semaphore
  - Queues

### **1.3.2** 
- [x] Ensure that all operations (LED control, button input handling, and LCD display updates) are mapped to **different tasks running independently** to maintain modularity and concurrency within the system.


**Task submitted by**: Devesh Shevde  
**Date os submission**: September 15, 2024
