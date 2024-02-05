# Differential Drive robot using micro-ros and ESP32

>PlatformIO is used for building and uploading firmware to Microcontroller ESP32

## Features
- ESP32 subscribes `cmd_vel` topic and converts it into required left wheel and right wheel velocities using kinematics equations.
- Current velocity of the motor is measured using encoder tick values.
- Required velocity is attained using PID controller ([Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library) is used).
- Encoder values are published to `left_wheel_tick` and `right_wheel_tick` topics. 

## Issues
- Robot is not moving in intended path. PID parameters are to be fine tuned.