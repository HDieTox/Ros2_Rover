# Autonomous Rover with ROS2 🚀  
*ROS2-based robot for autonomous navigation*

## Current State 🧪  
**Functional Prototype** with core navigation capabilities:  

- ✅ Simultaneous Localization and IMU
- ✅ Autonomous navigation with waypoints
- ❌ centimetrics precision
- ❌ **Real-world deployment**
- ❌ video perception (e.g., object recognition)  

---

## Board Used ⚙️
- Raspberry Pi CM5 IO board with **ROS2 JAZZY** (Ubuntu 24.04)
- STM32 B-L475E
- UBlox C099 F9P
- Polulu Trex JR 

## Project Structure and Architecture  📂

@startuml
skinparam packageStyle rectangle
skinparam monochrome false

title ROS2 Rover Project Structure

' Define packages for clarity
package "Power Supply" #FFE4B2 {
  
  [12V Battery] as Battery12V
  [Polulu Trex Jr (driver)] as TrexJr
  [STM32 B-L475E] as STM32Power
  [RC Receiver] as ReceiverRC
  [PowerBank 5V 5A] as PowerBank5V
  [CM5 IO Board] as CM5_IOBoard
  [C099 F9P C01 (GPS)] as C099F9PPower
  
  Battery12V --> TrexJr : 12V
  TrexJr --> STM32Power : 5V
  STM32Power --> ReceiverRC : 5V
  PowerBank5V --> CM5_IOBoard : 5V 5A
  CM5_IOBoard --> C099F9PPower : USB 5V
}

package "Communications and Control" #ADD8E6 {
  
  [Turnigy Remote Control] as RemoteControl
  [STM32 B-L475E] as STM32Comm
  [Raspberry Pi CM5] as CM5
  [C099 F9P C01] as C099F9PComm
  [Polulu Trex Jr (driver)] as TrexJrComm
  [DC Motors] as Motors
  
  RemoteControl --> STM32Comm : PWM control, switch
  C099F9PComm --> CM5 : NMEA UART
  STM32Comm --> CM5 : IMU UART (Accel + Gyro)
  CM5 --> STM32Comm : Movement commands UART
  STM32Comm --> TrexJrComm : PWM
  TrexJrComm --> Motors : Motor control
}

package "ROS2 Packages in CM5" #D3D3D3 {
  
  [rover_gpsreceiver] as GPSReceiver
  [rover_navigation] as Navigation
  [robot_localization (EKF)] as Localization

  GPSReceiver --> Navigation : Position data
  Navigation --> Localization : Navigation data
  Localization --> Navigation : EKF correction
}

@enduml


```
Ros2_Rover/
├── new_board_rover/         # STM32 Cube IDE project for the STM32 board
│
├── rover_autonomous/       # Core control ROS2 node
│   ├── config/
│   │   └── ekf.yaml        # kalman Filter configuration file
│   │
│   ├── include/
│   │
│   ├── launch/
│   │   └── navigation.launch.py        # Launch file
│   │
│   ├── src/
│   │   ├── nav_controller.cpp
│   │   └── serial_stm.cpp
│   │
│   ├── CMakeLists.txt
│   └── package.xml
│
├── rover_gpsreceiver/       # Core control ROS2 node
│   ├── include/
│   │
│   ├── launch/
│   │   └── gps.launch.py        # Launch file
│   │
│   ├── src/
│   │   ├── minnmea.c
│   │   ├── serial_nmea_publisher.cpp
│   │   └── nmea_parser.cpp
│   │
│   ├── CMakeLists.txt
│   └── package.xml
│
└── SQUARE.plan             # Waypoints map created with QGroundControl
```

## Installation & Setup 🛠️

1. Clone Repository
    ```bash 
    mkdir -p rover_ws/src
    git clone https://github.com/HDieTox/Ros2_Rover
    mv ./Ros2_Rover/rover_autonomous ./rover_ws/src
    mv ./Ros2_Rover/rover_gpsreceiver ./rover_ws/src
    mv ./Ros2_Rover/SQUARE.plan ./rover_ws/src
    ```
2. Build with colcon:
    ```bash 
    cd ./rover_ws && colcon build --symlink-install && . install/setup.bash
    ```
## Usage 🧭

in 2 different terminals : 

- `ros2 launch rover_gpasreceiver gps.launch.py`
- `ros2 launch rover_autonomous navigation.launch.py mission_file:=./src/SQUARE.plan`