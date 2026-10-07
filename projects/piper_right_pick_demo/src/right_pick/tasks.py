"""Task registration; success evidence is separate from model observations."""
from dataclasses import dataclass


@dataclass(frozen=True)
class CubePickPlace:
    name: str = "right_red_cube_pick_place"
    active_arm: str = "right"
    object_size_m: float = 0.03
    instruction: str = "用右臂抓起约3厘米红色方块，搬到旁边已确认的空桌面，释放并撤离；左臂保持静止。"
    required_evidence: tuple = ("lift", "transport", "release", "stable")


TASKS = {"right_red_cube_pick_place": CubePickPlace}


def get_task(name):
    try:
        return TASKS[name]()
    except KeyError:
        raise ValueError("Unregistered task: " + str(name))
