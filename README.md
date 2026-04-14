# RAMSoc Robotic Arm Workshop

A 2-DOF robotic arm controlled by an Arduino Uno, using inverse kinematics to move to user-specified (x, y) coordinates.

## How It Works

The arm has two links (shoulder and elbow) driven by servo motors. Given a target coordinate, the code solves the inverse kinematics equations to find the required joint angles and drives the servos accordingly.

- **Link 1 (shoulder):** 45mm
- **Link 2 (elbow):** 103mm
- **Reachable range:** 58mm – 148mm from the base

## Hardware

- Arduino Uno
- 2× servo motors (shoulder on pin 10, elbow on pin 11)
- 1× claw servo (pin 9)

## Usage

1. Upload `robotic_arm_code.ino` to the Arduino via the Arduino IDE.
2. Open the Serial Monitor at **9600 baud**.
3. Type coordinates as `x,y` (in mm) and press Enter.

```
100,50
140,0
80,80
```

The arm will move to the target if it's within reach, or print an error if it's not.
