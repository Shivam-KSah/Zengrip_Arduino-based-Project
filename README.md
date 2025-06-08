# 🤖 Arduino Nano Robot Controller with Servos, DC Motors, PCA9685 & iBUS

# 🤖 Arduino Nano Robot Controller with Servos, DC Motors, PCA9685 & iBUS

This project is a remote-controlled robot using an **Arduino Nano**, two **servo motors**, four **DC motors**, a **PCA9685 servo driver**, and an **L298N motor driver**. The robot is controlled using a FlySky **FS-i6** transmitter via **iBUS protocol**.

## 📁 Folder Structure



## 🔧 Hardware Used

- Arduino Nano
- PCA9685 PWM Servo Driver
- 2x Servo Motors (MG995 or similar)
- 4x DC Motors
- L298N Motor Driver
- FlySky FS-i6 Transmitter and Receiver (iBUS compatible)
- Power supply (LiPo battery recommended)
- Jumper wires, Breadboard or PCB

---


> ⚠️ Make sure to connect all GNDs together (Nano, L298N, PCA9685, power supply).

---

## 📦 Library Requirements

Install the following libraries in the Arduino IDE:
- `Adafruit PWM Servo Driver` (for PCA9685)
- `IBusBM` (for iBUS receiver communication)

Use Library Manager or install via ZIP.

---

## 🧠 Code Overview

### iBUS Channels Mapping:
| Channel | Function       |
|---------|----------------|
| CH1     | Steering (Left/Right) |
| CH3     | Throttle (Forward/Backward) |
| CH5     | Servo 1 Control (SWB) |
| CH6     | Servo 2 Control (SWC) |

### Servo Control:
- Servo 1 & Servo 2 are smoothly updated based on target positions (0°–180°) from the iBUS receiver.

### Motor Control Logic:
```cpp
if (throttle > 50)       // Move forward
else if (throttle < -50) // Move backward
else if (steering > 50)  // Turn right
else if (steering < -50) // Turn left
else Stop();
