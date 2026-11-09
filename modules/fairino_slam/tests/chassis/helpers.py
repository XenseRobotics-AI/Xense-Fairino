from fairino_slam.chassis.control_core import JOINTS


def hardware():
    return {"joint_names": JOINTS[:], "position": [0.0]*8, "velocity": [0.0]*8,
            "command_position": [0.0]*8, "command_velocity": [0.0]*8,
            "stateword": [0x40]*8, "errorcode": [0]*8, "hexerrorcode_curerrcode": 0}

