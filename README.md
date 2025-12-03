# Line-Following-Robot
PID-based Line Following Robot using Arduino and IR sensors

##  Overview
This project implements a **PID-based line following robot** using **Arduino** and **5 IR sensors**. The robot follows a black line on a white surface with smooth motion and automatic error correction.

- **Motors:** 2 DC motors with L298N driver  
- **Sensors:** 5 IR sensors for line detection  
- **Controller:** PID (Proportional, Integral, Derivative)  
- **Programming:** Arduino C/C++  

---

## Features
- Smooth and accurate line following  
- Adjustable base and max motor speeds  
- PID controller for error correction  
- Handles sharp turns and edge cases  
- Expandable with obstacle detection or wireless control  

---

## Hardware Requirements
- Arduino UNO / Nano  
- L298N Motor Driver Module  
- 5 IR Sensors (Left to Right)  
- DC Motors with Wheels  
- Jumper Wires, Breadboard, Battery/Power Supply  

---

##  Software / Code
**Arduino Sketch:** `LFR_PID.ino`  

**PID Algorithm in code:**
```cpp
float proportional = error * Kp;
integral = constrain(integral + error, -50, 50);
float integralTerm = integral * Ki;
float derivative = (error - previousError) * Kd;
int pid = proportional + integralTerm + derivative;
