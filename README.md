# Smart Stoplight Controller

<img width="500" height="300" alt="Screenshot 2025-12-01 at 1 48 32 PM" src="https://github.com/user-attachments/assets/94ab8b03-fe8d-4994-9f4a-b9d0341f0884" />


## Overview
The **Smart Stoplight Controller** is an intelligent traffic control system built on the **ESP32** using the **FreeRTOS** real-time operating system. It manages a two-way intersection with adaptive traffic light timing based on vehicle presence and includes a pedestrian crosswalk with a countdown display. By leveraging FreeRTOS concurrency, the system avoids blocking delays and responds dynamically to real-world events, improving efficiency and safety over fixed-timing traffic lights.

## Objectives
- Design an adaptive traffic light controller for a two-way intersection  
- Implement real-time multitasking using FreeRTOS  
- Support vehicle detection and pedestrian crossing requests  
- Ensure safe, responsive, and non-blocking system behavior  

## Hardware Components
- ESP32 microcontroller  
- LEDs for two traffic lights (TL1 and TL2)  
- PIR sensors for vehicle detection  
- Push buttons for pedestrian crosswalk requests  
- TM1637 seven-segment display for countdown timer  

## System Design
The application uses **FreeRTOS primitives** to manage concurrency and synchronization:
- **Tasks** for sensor monitoring, traffic light control, display updates, and logging  
- **Software timers** for light phase durations and crosswalk countdowns  
- **Interrupts** for immediate response to vehicle detection and button presses  
- **Mutexes** to protect shared state variables  
- **Semaphores and queues** for safe event signaling and serial output  

## Traffic Control Logic
A **state machine** governs all traffic light transitions, including:
- Green, Yellow, and Safety Red phases for both directions  
- Dedicated pedestrian crosswalk state  

### Timing Parameters
- Green: 18 seconds (reduced to 8 seconds when vehicles are detected waiting)  
- Yellow: 4 seconds  
- Safety Red: 2 seconds  
- Crosswalk: 20-second countdown + 2-second safety buffer  

Vehicle detection interrupts allow the system to dynamically shorten green phases to optimize traffic flow. Pedestrian requests are queued and safely serviced after the current yellow phase, with both lights set to red during crossing.

## Results
- Smooth, non-blocking operation with concurrent task execution  
- Adaptive green light reduction reduced idle waiting time  
- Accurate pedestrian countdown display with no missed button events  
- Reliable synchronization with no race conditions or corrupted output  
- Demonstrated clear advantages over delay-based, non-RTOS designs  

## Skills Demonstrated
- FreeRTOS multitasking and synchronization (tasks, timers, mutexes, semaphores)  
- Embedded state machine design  
- Interrupt-driven event handling  
- Real-time control system design  
- Modular and scalable embedded architecture  

## Conclusion
This project demonstrates how **FreeRTOS-enabled concurrency** can significantly enhance embedded control systems. The Smart Stoplight Controller adapts to traffic and pedestrian demands in real time, ensuring efficiency, safety, and reliability. The design highlights the benefits of event-driven, multitasking architectures for scalable traffic management and intelligent infrastructure.
