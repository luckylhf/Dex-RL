from dataclasses import dataclass


@dataclass
class ControlFlag:
    fsm_state_command: str = 'gotoZERO'
    motion_cmd: str = ''
    enable: bool = True
    x_speed_command: float = 0.0
    y_speed_command: float = 0.0
    yaw_speed_command: float = 0.0

