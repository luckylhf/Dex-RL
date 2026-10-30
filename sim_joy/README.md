# sim_joy

Joystick teleoperation package for Dex ROS simulation. For workspace setup and the complete simulation launch, see the [project README](../README.md#-ros2--mujoco-闭环).

## Features

- **Teleop Node**: Publishes `sensor_msgs/Joy` on `/sbus_data`; it also publishes `bodyctrl_msgs/SbusData` events on `/sbus_data/event` when available.
- **Standalone gamepad**: `teleop_joy` reads a physical gamepad through `inputs`, without a GUI, keyboard listener or console input.
- **GUI control**: `teleop_gui` provides Tkinter controls and a monitor, with its own internal teleoperation node.

## Usage

### Run the standalone gamepad node
```bash
ros2 run sim_joy teleop_joy
```

### Run GUI Control
```bash
ros2 run sim_joy teleop_gui
```
The GUI starts its teleoperation logic internally; do not start both entries for the same simulated robot. A working graphical display is required for the GUI.

## Dependencies
- `inputs`
- `pynput` (for keyboard listener support when enabled)
- `python3-tk` (GUI)
