from typing import List

MOVE_GROUP_ARM: str = "left_arm"
MOVE_GROUP_GRIPPER: str = "left_hand"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.015, 0.015]


def joint_names(prefix: str = "left_") -> List[str]:
    return [
        prefix + "gripper_df1_joint",
        prefix + "gripper_df2_joint",
        prefix + "gripper_wrist_joint",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = "left_") -> str:
    return prefix + "gripper_wrist_link"


def gripper_joint_names(prefix: str = "left_") -> List[str]:
    return [
        prefix + "1_gripper_hand_joint",
        prefix + "2_gripper_hand_joint",
    ]
