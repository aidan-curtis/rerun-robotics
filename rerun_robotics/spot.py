import os
from typing import Sequence

import numpy as np
import rerun as rr
from yourdfpy import URDF
import pathlib
from rerun_robotics.rerun_urdf import log_scene

# We load the pybullet panda
spot_assets_dir = os.path.join(pathlib.Path(__file__).parent, "./spot_description/")

spot_urdf_path = os.path.join(spot_assets_dir, "mobile_model.urdf")

default_conf = {
    "x": 0,
    "y": 0,
    "z": 0,
    "theta": 0,
    "fl.hx": 0.0,
    "fl.hy": 0.8,
    "fl.kn": -1.6,
    "fr.hx": 0.0,
    "fr.hy": 0.8,
    "fr.kn": -1.6,
    "hl.hx": 0.0,
    "hl.hy": 0.8,
    "hl.kn": -1.6,
    "hr.hx": 0.0,
    "hr.hy": 0.8,
    "hr.kn": -1.6,
    "arm0.sh0": 0.0,
    "arm0.sh1": 0.0,
    "arm0.hr0": 0.0,
    "arm0.el0": 0.0,
    "arm0.el1": 0.0,
    "arm0.wr0": 0.0,
    "arm0.wr1": 0.0,
    "arm0.f1x": 0.0,
}



def locate_franka_asset(fname: str) -> str:
    fname = fname.replace("package://", "")
    asset_path = os.path.join(spot_assets_dir, fname)
    return asset_path


class PandaRerun:
    """Helper class for commanding the Panda in rerun. Note the last joint is a mimic joint for the gripper."""

    def __init__(self, urdf: URDF):
        self._urdf = urdf
        rr.log("world_link", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        log_scene(scene=urdf.scene, node="world_link", path="spot", static=False, add_mesh=True)

    @property
    def joint_positions(self) -> np.ndarray:
        return self._urdf.cfg

    def set_joint_positions(self, joint_positions: Sequence[float]) -> None:
        if len(joint_positions) != self._urdf.num_actuated_joints:
            raise ValueError(
                f"We only support setting {self._urdf.num_actuated_joints}, not {len(joint_positions)} joints"
            )
        self._urdf.update_cfg(joint_positions)
        log_scene(scene=self._urdf.scene, node="world_link", path="spot", static=False, add_mesh=False)


def load_spot(initial_joint_positions=default_conf) -> PandaRerun:
    """
    Load franka to rerun.
    Modified from: https://github.com/rerun-io/rerun/blob/main/examples/python/ros_node/rerun_urdf.py
    """
    urdf = URDF.load(spot_urdf_path, filename_handler=locate_franka_asset)
    panda = PandaRerun(urdf)
    panda.set_joint_positions(initial_joint_positions)
    return panda
