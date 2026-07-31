# jamming_platform

ROS 2 package to run force-displacement experiments on a jamming platform, with:

- Dynamixel motor control for mechanical displacement.
- Pressure control on ESP32 through micro-ROS.
- Telemetry logging in ROS 2 and CSV.

## 1. Package Structure

- Main experiment node:
	- [src/force_displacement_experiment.cpp](src/force_displacement_experiment.cpp)
	- [include/jamming_platform/froce_displacement_experiment.h](include/jamming_platform/froce_displacement_experiment.h)
- Parameter configuration:
	- [config/force_displacement_experiment.yaml](config/force_displacement_experiment.yaml)
- ESP32 firmware (Arduino):
	- [firmware/pressure_control/pressure_control.ino](firmware/pressure_control/pressure_control.ino)
	- [firmware/state_transition_analysis/state_transition_analysis.ino](firmware/state_transition_analysis/state_transition_analysis.ino)
- Host-side serial capture utility:
	- [scripts/capture_state_transition.py](scripts/capture_state_transition.py)

## 2. What This Package Does

The executable [src/force_displacement_experiment.cpp](src/force_displacement_experiment.cpp) performs a pressure sweep and, for each setpoint, repeats a displacement experiment until a force or displacement limit is reached:

1. Waits for valid pressure and force measurements.
2. Moves the motor to the initial position (EXTENDED_POSITION_CONTROL_MODE).
3. Publishes a pressure setpoint.
4. Switches to VELOCITY_CONTROL_MODE and moves the motor.
5. Publishes telemetry samples while moving.
6. Stops, returns to origin, and repeats.

In addition, every sample published to experiment_data is also written to a CSV file in the experiemnt_results folder.

## 3. Prerequisites

### 3.1 PC Software (Ubuntu + ROS 2)

- ROS 2 installed and sourced correctly.
- colcon.
- Python 3.
- pyserial (for serial capture script).
- Package dependencies declared in [package.xml](package.xml):
	- rclcpp
	- std_msgs
	- sensor_msgs
	- geometry_msgs
	- dynamixel_sdk
	- dynamixel_ros2
- micro-ROS Agent on the PC to communicate with the ESP32.

### 3.2 Hardware

- ESP32 connected to the pneumatic system.
- Pressure sensor on ADC1_CHANNEL_7 (GPIO35), as defined in [firmware/pressure_control/pressure_control.ino](firmware/pressure_control/pressure_control.ino).
- Valves:
	- GPIO12: pressurization (rigid valve).
	- GPIO14: depressurization (soft valve).
- Dynamixel motor connected to the bus and a USB-serial adapter.
- A force sensor pipeline publishing geometry_msgs/msg/WrenchStamped on sensor_wrench.

Important: this package does not publish sensor_wrench by itself; that topic must come from another node/acquisition system.

## 4. Build the Workspace

From the workspace root:

```bash
cd /path/to/YOUR_WS
colcon build --packages-select jamming_platform
source install/setup.bash
```

## 5. ESP32 Firmware

This package includes two firmware sketches for different purposes.

### 5.1 pressure_control.ino (used in ROS 2 experiments)

File: [firmware/pressure_control/pressure_control.ino](firmware/pressure_control/pressure_control.ino)

Behavior:

- Connects to the PC through micro-ROS.
- Subscribes to goal_pressure (sensor_msgs/msg/FluidPressure).
- Publishes current_pressure (sensor_msgs/msg/FluidPressure).
- Publishes valve states on_rigid_valve_state and on_soft_valve_state (std_msgs/msg/Bool).
- Runs a hysteresis controller at 100 Hz.

Flashing steps (Arduino IDE):

1. Install ESP32 board support in Arduino IDE.
2. Install the micro_ros_arduino library.
3. Open [firmware/pressure_control/pressure_control.ino](firmware/pressure_control/pressure_control.ino).
4. Select the ESP32 board and serial port.
5. Compile and upload.

Notes:

- micro-ROS transport is initialized with set_microros_transports().
- If your micro_ros_arduino setup uses serial transport, the device must match the port used by micro_ros_agent.

### 5.2 state_transition_analysis.ino (transition analysis)

File: [firmware/state_transition_analysis/state_transition_analysis.ino](firmware/state_transition_analysis/state_transition_analysis.ino)

Behavior:

- Executes valve transitions and samples ADC at high frequency.
- Prints samples through Serial.
- Does not use ROS 2 at runtime.

Usage:

1. Upload the sketch to ESP32.
2. Open Serial Monitor at 115200.
3. Press Enter to start (the firmware waits for '\n').
4. Capture serial output for offline analysis.

### 5.3 Python serial capture script (recommended)

File: [scripts/capture_state_transition.py](scripts/capture_state_transition.py)

Purpose:

- Sends the start key (newline) to the firmware.
- Reads serial samples continuously.
- Writes CSV with: experiment, time, pressure.
- Uses fixed sampling rate at 10 kHz to reconstruct time.

Install dependency:

```bash
python3 -m pip install pyserial
```

Run example:

```bash
cd /path/to/YOUR_WS
python3 src/jamming_platform/scripts/capture_state_transition.py --port /dev/ttyUSB0
```

Useful options:

- --num-experiments: stop automatically after N experiments.
- --samples-per-transition: set sample count per transition (default 1000).
- --transitions-per-experiment: transitions per experiment (default 2).
- --output: output CSV file path.
- --slope and --offset: convert ADC to calibrated pressure.

Example with automatic stop and explicit output:

```bash
python3 src/jamming_platform/scripts/capture_state_transition.py \
	--port /dev/ttyUSB0 \
	--num-experiments 10 \
	--output state_transition_run.csv
```

## 6. Run the Full Experiment

### 6.1 Terminal A: ROS environment

```bash
cd /path/to/YOUR_WS
source install/setup.bash
```

### 6.2 Terminal B: micro-ROS Agent (ESP32)

Example using serial transport:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB_ESP32 -b 115200
```

Replace /dev/ttyUSB_ESP32 with your ESP32 serial device.

### 6.3 Terminal C: force publisher node

Start your force acquisition node/system so it publishes:

- sensor_wrench (geometry_msgs/msg/WrenchStamped)

### 6.4 Terminal D: run the experiment

```bash
cd /path/to/YOUR_WS
source install/setup.bash
ros2 run jamming_platform force_displacement_experiment \
	--ros-args --params-file src/jamming_platform/config/force_displacement_experiment.yaml
```

## 7. Experiment Parameters

Defined in [config/force_displacement_experiment.yaml](config/force_displacement_experiment.yaml).

### 7.1 Motor communication

- port_name: Dynamixel adapter serial port (example: /dev/ttyUSB0).
- protocol_version: Dynamixel protocol version.
- baudrate: bus baudrate.
- motor_id: actuator ID.

### 7.2 Kinematics and limits (in mm)

- initial_position_mm: linear reference position.
- position_tolerance_mm: tolerance to consider target reached.
- max_displacement_mm: maximum allowed displacement before stopping.
- goal_velocity_rpm: motor speed during the test phase.
- max_force_n: safety force threshold.

Internal conversion used by the node:

- Lead screw: 1 revolution = 1.25 mm.
- Degrees to mm: mm = deg * (1.25 / 360).

### 7.3 Pressure sweep

- pressure_start_kpa
- pressure_end_kpa
- pressure_step_kpa
- pressure_tolerance_kpa
- repetitions_per_pressure

Note: the code validates that pressure_step_kpa is negative for a descending sweep.

### 7.4 Timing

- loop_rate
- settle_time_s
- pressure_settle_timeout_s
- move_timeout_s

## 8. Topics ROS 2

### 8.1 Published by jamming_platform/force_displacement_experiment

- current_position (std_msgs/msg/Float64): linear position in mm.
- goal_pressure (sensor_msgs/msg/FluidPressure): pressure setpoint for ESP32.
- experiment_data (std_msgs/msg/Float64MultiArray):
	- [elapsed_s, current_pressure_kpa, position_mm, force_n]

### 8.2 Subscribed by jamming_platform/force_displacement_experiment

- current_pressure (sensor_msgs/msg/FluidPressure)
- sensor_wrench (geometry_msgs/msg/WrenchStamped)

### 8.3 Published by pressure_control firmware

- current_pressure (sensor_msgs/msg/FluidPressure)
- on_rigid_valve_state (std_msgs/msg/Bool)
- on_soft_valve_state (std_msgs/msg/Bool)

### 8.4 Subscribed by pressure_control firmware

- goal_pressure (sensor_msgs/msg/FluidPressure)

## 9. CSV Data Logging

The node automatically creates a results folder and output file:

- Default folder: experiemnt_results
- File name: force_displacement_YYYYMMDD_HHMMSS.csv

CSV columns:

- n_exp;rep;goal_pressure;current_pressure;position;force;time

Where:

- n_exp: pressure-level index.
	- 0 for pressure_start_kpa.
	- 1 for pressure_start_kpa + pressure_step_kpa.
	- and so on.
- rep: repetition index (1..repetitions_per_pressure).
- goal_pressure: current pressure setpoint.
- current_pressure: measured pressure.
- position: linear position in mm.
- force: force in N.
- time: elapsed time from experiment start in s.

## 10. Useful Runtime Checks

Check active topics:

```bash
ros2 topic list
```

View current pressure:

```bash
ros2 topic echo /current_pressure
```

View experiment samples:

```bash
ros2 topic echo /experiment_data
```

View linear position:

```bash
ros2 topic echo /current_position
```

Capture state transition data to CSV:

```bash
python3 src/jamming_platform/scripts/capture_state_transition.py --port /dev/ttyUSB0
```

## 11. Troubleshooting

### 11.1 Experiment does not start (waiting for sensors)

Likely cause:

- current_pressure or sensor_wrench is missing.

Actions:

- Verify micro_ros_agent is running and connected.
- Verify ESP32 is publishing current_pressure.
- Verify your force node is publishing sensor_wrench.

### 11.2 Motor does not move

Check:

- Correct port_name for the Dynamixel adapter.
- baudrate, protocol_version, and motor_id values.
- Motor power and torque state.

### 11.3 CSV file is not generated

Check:

- Write permissions in the current execution directory.
- Value of results_directory.

### 11.4 Pressure units do not match

The system uses sensor_msgs/msg/FluidPressure; make sure publisher and subscriber use the same scaling for fluid_pressure.

## 12. License

See [LICENSE](LICENSE).
