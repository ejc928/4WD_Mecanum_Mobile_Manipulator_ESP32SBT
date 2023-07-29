# 4WD Mecanum Mobile Manipulator

_This Project is for the completion of our Microcontrollers Subject._

Group: Company 6 - SCARA Manipulator



#  Introduction
This project is the compilation of our learnings during our Microprocessors and Microcontrollers Subject.

It involves the use of microcontrollers such as Arduino, and  ESP32, our knowledge about electronics, designing electrical circuits, and programming using Arduino IDE.

The group was tasked to design and program a mobile manipulator robot that can move along a maze, pick up objects,
and bring them to the designated position.



# Objectives
Here are the following objectives we wanted our robot to do:
  - The robot must move easily in all directions so that it won't get stuck in places.
  - The arm/manipulator must have the ability to reach the floor.
  - The gripper must have anti-slip material.
  - The robot and the arm must be controlled with ease.
  - Must be compact in size or small enough to fit the maze.

We began making our prototype baseed on these objectives.

# Materials
1. 3D Printed Manipulator Arm
    - The manipulator or arm used for the project is called the "EEZYbotARM MK1" made by daGHIZmo.
    - 3D Model: https://www.thingiverse.com/thing:1015238

![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/e9b89686-7d89-4667-bdd0-887cc7c31044)

2. NodeMCU ESP32S
    - Compact (fits the breadboard)
    - Fast
    - Built in Wifi and Bluetooth
    - Support for Dabble Gamepad Library
3. Breadboard
4. Breadboard Power Supply
5. 2 x 18650 Batteries
6. 4 x TT Gear Motors
7. IR Receiver
8. 4 x MG90S Servos
9. Rocker Switch
10. White and Red LEDs
11. Mecanum Wheels Set

Bonus:
 - 3 x HC-SR04 Ultrasonic Sensors
 - 4-Channel Line tracking Module

# Wiring Diagram
![8V](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/441d6b49-8d10-42e6-9268-8fd87dc9ea78)

# Chassis
These are the parts to be 3D Printed for our main chassis.

These parts are designed using Onshape.

## Base
![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/b73f4367-f697-4487-948a-b0008baa2274)
## Top Cover
![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/125d8373-d6be-41ed-8091-89ef3c9fbcb6)
## Wiring Cover
![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/190dc432-8a8a-4c91-8440-6f068fe14d42)

# Making
The chassis will be printed out of eSun PLA+

![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/20201192-51c4-4e19-b513-ebcd0788abca)

Printer Used: Ender 3 S1 Pro running Klipper

## Assembly:
![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/c7cffc04-0ac8-4d8c-83ec-6d492335b2d8)

## Coding:
![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/ef249a92-e1a9-488b-9478-385336a91df9)

# Finished Project
![image](https://github.com/ejc928/4WD_Mecanum_Mobile_Manipulator_ESP32SBT/assets/107923200/53a8eb06-1d9d-4f6c-aa86-e449e995cc28)

## Features:
   - Wireless control
     - IR Remote for mode changing
     - Dabble App by STEMpedia for Arm and Motor control
   - Wide range of movement
   - Partial Line Tracking (WIP)
   - Partial Maze Solving by wall following (WIP)
   - Arm can reach ground.
   - Items held are stable.
   - Front and Back LEDS change based on robot direction.















