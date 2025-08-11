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

![](https://www.plantuml.com/plantuml/svg/XLHVJzim77ptfpYnXnLjI53eDdYOw9yMmIX4JM4FquICQuf7ObU_EB1isdUVxU1YsBIfB_RTsRdpsJsiB2TRbmMhxfHUSU8bLbpSyQNCRLD8a1IMwsKX4q5fj16tP4g96rvKaZ6hhDFEplCkPkPU4Z8otznAv9PgOMjocjSOoHkbPTo_meqXY8AJiWrx1B6HcGT9oElLgcWsy6eo6UyDklZ900PysUbUOi2jbTHy1Q_YsC6UpKnH5pNc9B_ZXD1Pa79cDeFIWoVaLVbykjl5OEjqxtrl7BY0-0y7VZR4J0hfbmOsJcP3pmRPWEixz2xHwmV56ki56yDf3yVd61XEYy0xuEhuFCm3lxs_Zyb-Xk7s3Zf7MTuwzB13em-691ssjZwYZO03XBXnwfWaW6EzYGJnWYH352IUlIBrw2K-6qEA1bNgqSakye6Jib_igR-XASjQAy6jChe2rmiCZRPa2jTcVpJwC7wtRdDUarRBnhahZPLHrnvwW1wHVxJcFnTAuzNgMX8ro9Ht7S_zvNcd9ntN_l_QHELeYAcnXgh0jSEGvRdVfqxyGXmW-po5QBctg1wK5RSCgOrOWTEUJSTzNFHdyzZY2_vuUX5eTFf2o09lSDIGsMpBUlbbv_5Ubb9R23TtPLH_tph6Z6uP8XGLRLZ7Xa5CywpwyFwp-BgLzePSwQDT_rkNJluFucgvgYXzPEuLn3lw9DByNYt3HGgQi_MqbLmRUrKOmGlr8w3eZ3zDsjfE4vWnfDlxDCbMFg-fL1WkkEKCAUcrwLuuIDYeJmL_xkuyGHWYAJpW3-nGweN_o_qD)

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