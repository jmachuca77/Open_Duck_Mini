import mujoco
import pickle
from mini_bdx.utils.mujoco_utils import check_contact
import mujoco.viewer
import time
import numpy as np


model = mujoco.MjModel.from_xml_path(
    "/home/antoine/MISC/mini_BDX/mini_bdx/robots/open_duck_mini_v2/scene_position.xml"
)
model.opt.timestep = 0.005
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
decimation = 4
ctrl_dt = model.opt.timestep * decimation
init_pos = np.array(
    [
        0.002,
        0.053,
        -0.63,
        1.368,
        -0.784,
        0.0,
        0,
        0,
        0,
        0,
        0,
        -0.003,
        -0.065,
        0.635,
        1.379,
        -0.796,
    ]
)

def quat_rotate_inverse(q, v):
    q = np.array(q)
    v = np.array(v)

    q_w = q[-1]
    q_vec = q[:3]

    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * (np.dot(q_vec, v)) * 2.0

    return a - b + c

def get_feet_contact():
    left_contact = check_contact(data, model, "foot_assembly", "floor")
    right_contact = check_contact(data, model, "foot_assembly_2", "floor")
    return [left_contact, right_contact]

def get_obs(data, prev_action, commands):
    base_quat = data.qpos[3 : 3 + 4].copy()
    base_quat = [base_quat[1], base_quat[2], base_quat[3], base_quat[0]]

    dof_pos = data.qpos.copy()

    dof_vel = data.qvel.copy()

    projected_gravity = quat_rotate_inverse(base_quat, [0, 0, -1])

    feet_contacts = get_feet_contact()
    # feet_contacts = [0, 0]

    obs = np.concatenate(
        [
            projected_gravity,
            dof_pos,
            dof_vel,
            feet_contacts,
            prev_action,
            commands,
        ]
    )

    return obs

target = [0] * 16# + init_pos
data.qpos = target
data.ctrl = target

A = 0.3
F = 2.0
joint_id = 2
counter = 0
saved_obs = []
with mujoco.viewer.launch_passive(
    model, data, show_left_ui=False, show_right_ui=False
) as viewer:
    t = 0
    while True:
        t += model.opt.timestep
        # step_start = data.time
        mujoco.mj_step(model, data)
        counter += 1
        if counter % decimation == 0:
            target[joint_id] = A * np.sin(2 * np.pi * F * t)# - init_pos[joint_id]
            data.ctrl[joint_id] = target[joint_id]
            obs = get_obs(data, data.ctrl, [0, 0, 0])
            saved_obs.append(obs)

        if len(saved_obs) > (1/ctrl_dt) * 10 :  # 10 seconds
            break


        viewer.sync()

        # time_until_next_step = model.opt.timestep - (data.time - step_start)
        # if time_until_next_step > 0:
        #     time.sleep(time_until_next_step)


pickle.dump(saved_obs, open("mujoco_saved_obs.pkl", "wb"))