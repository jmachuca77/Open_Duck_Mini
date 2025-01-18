import mujoco
import numpy as np
import mujoco_viewer
import time
from scipy.spatial.transform import Rotation as R
import pygame
import argparse
from mini_bdx.utils.mujoco_utils import check_contact

from mini_bdx_runtime.onnx_infer import OnnxInfer
import pickle

# from mini_bdx.onnx_infer import OnnxInfer
from mini_bdx_runtime.rl_utils import (
    # action_to_pd_targets,
    isaac_to_mujoco,
    mujoco_to_isaac,
)

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--onnx_model_path", type=str, required=True)
parser.add_argument("-k", action="store_true", default=False)
# parser.add_argument("--rma", action="store_true", default=False)
# parser.add_argument("--awd", action="store_true", default=False)
# parser.add_argument("--adaptation_module_path", type=str, required=False)
parser.add_argument("--replay_obs", type=str, required=False, default=None)
args = parser.parse_args()

if args.k:
    pygame.init()
    # open a blank pygame window
    screen = pygame.display.set_mode((100, 100))
    pygame.display.set_caption("Press arrow keys to move robot")

if args.replay_obs is not None:
    with open(args.replay_obs, "rb") as f:
        replay_obs = pickle.load(f)
        replay_obs = np.array(replay_obs)

# Params
dt = 0.005
linearVelocityScale = 1.0
angularVelocityScale = 1.0
dof_pos_scale = 1.0
dof_vel_scale = 1.0
action_scale = 1.0

# - left_hip_yaw : 0.0
# - left_hip_roll : 0.05
# - left_hip_pitch : -0.63
# - left_knee : 1.368
# - left_ankle : -0.78
# - neck_pitch : 0
# - head_pitch : 0
# - head_yaw : 0
# - head_roll : 0
# - left_antenna : 0
# - right_antenna : 0
# - right_hip_yaw : 0
# - right_hip_roll : -0.065
# - right_hip_pitch : 0.635
# - right_knee : 1.38
# - right_ankle : -0.79
isaac_init_pos = np.array(
    [
        0.0,
        0.05,
        -0.63,
        1.368,
        -0.78,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        -0.065,
        0.635,
        1.38,
        -0.79,
    ]
)


mujoco_init_pos = np.array(isaac_to_mujoco(isaac_init_pos))

model = mujoco.MjModel.from_xml_path(
    "/home/antoine/MISC/mini_BDX/mini_bdx/robots/open_duck_mini_v2/scene_position.xml"
)
model.opt.timestep = dt
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
viewer = mujoco_viewer.MujocoViewer(model, data)


NUM_OBS = 56

policy = OnnxInfer(args.onnx_model_path, awd=True)


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


def get_obs(data, prev_isaac_action, commands):
    base_quat = data.qpos[3 : 3 + 4].copy()
    base_quat = [base_quat[1], base_quat[2], base_quat[3], base_quat[0]]

    mujoco_dof_pos = data.qpos[7 : 7 + 16].copy()
    isaac_dof_pos = mujoco_to_isaac(mujoco_dof_pos)

    mujoco_dof_vel = data.qvel[6 : 6 + 16].copy()
    isaac_dof_vel = mujoco_to_isaac(mujoco_dof_vel)

    projected_gravity = quat_rotate_inverse(base_quat, [0, 0, -1])
    feet_contacts = get_feet_contact()

    obs = np.concatenate(
        [
            projected_gravity,
            isaac_dof_pos,
            isaac_dof_vel,
            feet_contacts,
            prev_isaac_action,
            commands,
        ]
    )

    return obs


prev_isaac_action = np.zeros(16)
commands = [0.0, 0.0, 0.0]
prev = data.time
last_control = data.time
control_freq = 30  # hz
i = 0
data.qpos[3 : 3 + 4] = [1, 0, 0.0, 0]

data.qpos[7 : 7 + 16] = mujoco_init_pos
data.ctrl[:16] = mujoco_init_pos


def handle_keyboard():
    global commands
    keys = pygame.key.get_pressed()
    lin_vel_x = 0
    lin_vel_y = 0
    ang_vel = 0
    if keys[pygame.K_z]:
        lin_vel_x = 0.3
    if keys[pygame.K_s]:
        lin_vel_x = -0.2
    if keys[pygame.K_q]:
        lin_vel_y = 0.2
    if keys[pygame.K_d]:
        lin_vel_y = -0.2
    if keys[pygame.K_a]:
        ang_vel = 0.3
    if keys[pygame.K_e]:
        ang_vel = -0.3

    commands[0] = lin_vel_x
    commands[1] = lin_vel_y
    commands[2] = ang_vel

    # commands = list(
    #     np.array(commands)
    #     * np.array(
    #         [
    #             linearVelocityScale,
    #             linearVelocityScale,
    #             angularVelocityScale,
    #         ]
    #     )
    # )
    pygame.event.pump()  # process event queue


start = time.time()
i = 0
while True:
    t = data.time
    if time.time() - start < 1:
        last_control = t

    if t - last_control >= 1 / control_freq:
        if args.replay_obs is not None:
            isaac_obs = replay_obs[i]
        else:
            isaac_obs = get_obs(data, prev_isaac_action, commands)
        isaac_action = policy.infer(isaac_obs)

        prev_isaac_action = isaac_action.copy()

        isaac_action = isaac_action * action_scale + isaac_init_pos

        mujoco_action = isaac_to_mujoco(isaac_action)
        data.ctrl[:14] = mujoco_action[:14].copy()

        if args.k:
            handle_keyboard()

        last_control = t
        i += 1
        if args.replay_obs is not None and i >= len(replay_obs):
            i = 0

    mujoco.mj_step(model, data, 3)
    viewer.render()
    prev = t
