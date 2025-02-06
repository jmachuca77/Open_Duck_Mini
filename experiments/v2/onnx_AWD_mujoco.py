import mujoco
import numpy as np

import mujoco.viewer
import time
import pygame
import argparse
from mini_bdx.utils.mujoco_utils import check_contact

from mini_bdx_runtime.onnx_infer import OnnxInfer
import pickle
from bam.model import load_model
from bam.mujoco import MujocoController
from mini_bdx_runtime.rl_utils import mujoco_joints_order

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--onnx_model_path", type=str, required=True)
parser.add_argument("-k", action="store_true", default=False)
parser.add_argument("--bam", action="store_true", default=False)
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
linearVelocityScale = 1.0
angularVelocityScale = 1.0
dof_pos_scale = 1.0
dof_vel_scale = 1.0
action_scale = 0.25


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

# model = mujoco.MjModel.from_xml_path(
#     "/home/antoine/MISC/mini_BDX/mini_bdx/robots/open_duck_mini_v2/scene_position.xml"
# )
model = mujoco.MjModel.from_xml_path(
    "/home/antoine/MISC/mujoco_menagerie/open_duck_mini_v2/scene.xml"
)
model.opt.timestep = 0.005
# model.opt.timestep = 1 / 240
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

if args.bam:
    sts3215_model = load_model("params_m6.json")
    mujoco_controllers = {}
    for joint_name in mujoco_joints_order:
        mujoco_controllers[joint_name] = MujocoController(
            sts3215_model, joint_name, model, data
        )


NUM_OBS = 56

policy = OnnxInfer(args.onnx_model_path, awd=True)

COMMANDS_RANGE_X = [-0.2, 0.3]
COMMANDS_RANGE_Y = [-0.2, 0.2]
COMMANDS_RANGE_THETA = [-0.3, 0.3]

prev_action = np.zeros(16)
commands = [0.3, 0.0, 0.0]
decimation = 4
data.qpos[3 : 3 + 4] = [1, 0, 0.0, 0]

data.qpos[7 : 7 + 16] = init_pos
data.ctrl[:16] = init_pos

replay_index = 0
saved_obs = []


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

    dof_pos = data.qpos[7 : 7 + 16].copy()

    dof_vel = data.qvel[6 : 6 + 16].copy()

    projected_gravity = quat_rotate_inverse(base_quat, [0, 0, -1])
    feet_contacts = get_feet_contact()

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


def handle_keyboard():
    global commands
    keys = pygame.key.get_pressed()
    lin_vel_x = 0
    lin_vel_y = 0
    ang_vel = 0
    if keys[pygame.K_z]:
        lin_vel_x = COMMANDS_RANGE_X[1]
    if keys[pygame.K_s]:
        lin_vel_x = COMMANDS_RANGE_X[0]
    if keys[pygame.K_q]:
        lin_vel_y = COMMANDS_RANGE_Y[1]
    if keys[pygame.K_d]:
        lin_vel_y = COMMANDS_RANGE_Y[0]
    if keys[pygame.K_a]:
        ang_vel = COMMANDS_RANGE_THETA[1]
    if keys[pygame.K_e]:
        ang_vel = COMMANDS_RANGE_THETA[0]

    commands[0] = lin_vel_x
    commands[1] = lin_vel_y
    commands[2] = ang_vel

    commands = list(
        np.array(commands)
        * np.array(
            [
                linearVelocityScale,
                linearVelocityScale,
                angularVelocityScale,
            ]
        )
    )
    print(commands)
    pygame.event.pump()  # process event queue


try:
    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        counter = 0
        while True:

            step_start = time.time()  # Was
            # step_start = data.time

            mujoco.mj_step(model, data)

            counter += 1
            if counter % decimation == 0:

                if args.replay_obs is not None:
                    obs = replay_obs[replay_index]
                else:
                    obs = get_obs(data, prev_action, commands)
                    saved_obs.append(obs)

                obs = list(obs) + list(np.zeros(18))
                action = policy.infer(obs)

                prev_action = action.copy()

                action = action * action_scale + init_pos

                # if args.bam:
                #     for i, joint_name in enumerate(mujoco_joints_order):
                #         mujoco_controllers[joint_name].update(action[i])
                # else:
                #     data.ctrl = action.copy()

                if args.k:
                    handle_keyboard()
                # print(commands)

                replay_index += 1
                if args.replay_obs is not None and replay_index >= len(replay_obs):
                    replay_index = 0

            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            # time_until_next_step = model.opt.timestep - (data.time - step_start)
            # if time_until_next_step > 0:
            #     time.sleep(time_until_next_step)

            # Was
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except KeyboardInterrupt:
    pickle.dump(saved_obs, open("mujoco_saved_obs.pkl", "wb"))
