from mini_bdx.placo_walk_engine.placo_walk_engine import PlacoWalkEngine
import time
import json
import mujoco
import mujoco.viewer
import pickle
from mini_bdx.utils.mujoco_utils import check_contact
import numpy as np

# DT = 0.01
DT = 0.002
decimation = 10
pwe = PlacoWalkEngine(
    "/home/antoine/MISC/mini_BDX/mini_bdx/robots/open_duck_mini_v2",
    model_filename="robot.urdf",
    init_params=json.load(open("placo_defaults.json")),
    ignore_feet_contact=True,
)
model = mujoco.MjModel.from_xml_path(
    "/home/antoine/MISC/openduckminiv2_playground/env/locomotion/open_duck_mini_v2/xmls/scene_mjx_flat_terrain.xml"
)
model.opt.timestep = DT
data = mujoco.MjData(model)

init_pos = np.array(
    [
        0.002,
        0.053,
        -0.63,
        1.368,
        -0.784,
        # 0.0,
        # 0,
        # 0,
        # 0,
        # 0,
        # 0,
        -0.003,
        -0.065,
        0.635,
        1.379,
        -0.796,
    ]
)

# angles = pickle.load(open("init_angles.pkl", "rb"))

data.ctrl[:] = init_pos
data.qpos[3 + 4 :] = init_pos
data.qpos[3 : 3 + 4] = [1, 0, 0.06, 0]


def get_feet_contact():
    left_contact = check_contact(data, model, "foot_assembly", "floor")
    right_contact = check_contact(data, model, "foot_assembly_2", "floor")
    return right_contact, left_contact


pwe.set_traj(0.05, 0, 0.0)
counter = 0
with mujoco.viewer.launch_passive(
    model, data, show_left_ui=False, show_right_ui=False
) as viewer:
    while True:
        # right_contact, left_contact = get_feet_contact()
        if counter % decimation == 0:
            pwe.tick(DT * decimation)
            angles = list(
                pwe.get_angles(
                    ignore=[
                        "neck_pitch",
                        "head_pitch",
                        "head_yaw",
                        "head_roll",
                        "left_antenna",
                        "right_antenna",
                    ]
                ).values()
            )
            data.ctrl[:] = angles

        counter += 1

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(DT)
