import mujoco.viewer

import time
import mujoco
import argparse
import pickle
import numpy as np

from mini_bdx.utils.mujoco_utils import check_contact

from mini_bdx_runtime.onnx_infer import OnnxInfer


parser = argparse.ArgumentParser()
parser.add_argument("-o", "--onnx_model_path", type=str, required=True)
parser.add_argument("-k", action="store_true", default=False)
parser.add_argument("--replay_obs", type=str, required=False, default=None)
parser.add_argument("--rma", action="store_true", default=False)
parser.add_argument("--adaptation_module_path", type=str, required=False)
parser.add_argument("--zero_head", action="store_true", default=False)
args = parser.parse_args()

if args.k:
    import pygame

    pygame.init()
    # open a blank pygame window
    screen = pygame.display.set_mode((100, 100))
    pygame.display.set_caption("Press arrow keys to move robot")

if args.replay_obs is not None:
    with open(args.replay_obs, "rb") as f:
        replay_obs = pickle.load(f)
        replay_obs = np.array(replay_obs)


def pd_control(target_q, q, kp, target_dq, dq, kd, clip_val, init_pos, action_scale):
    """Calculates torques from position commands"""
    tau = (target_q * action_scale + init_pos - q) * kp# - (dq * kd)
    tau = np.clip(tau, -clip_val, clip_val)
    tau -= dq * kd
    # tau += (target_dq - dq) * kd
    return tau
    # return (target_q - q) * kp + (target_dq - dq) * kd


def quat_rotate_inverse(q, v):
    q = np.array(q)
    v = np.array(v)

    q_w = q[-1]
    q_vec = q[:3]

    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * (np.dot(q_vec, v)) * 2.0

    return a - b + c


def get_obs(data, prev_action, commands):
    base_quat = data.qpos[3 : 3 + 4].copy()
    base_quat = [base_quat[1], base_quat[2], base_quat[3], base_quat[0]]

    dof_pos = data.qpos[7 : 7 + 16].copy()

    dof_vel = data.qvel[6 : 6 + 16].copy()

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


def get_feet_contact():
    left_contact = check_contact(data, model, "foot_assembly", "floor")
    right_contact = check_contact(data, model, "foot_assembly_2", "floor")
    return [left_contact, right_contact]

def set_zero_head(pos):
    pos[5] = np.deg2rad(10)
    pos[6] = np.deg2rad(-10)
    # pos[5] = 0
    # pos[6] = 0
    pos[7] = 0
    pos[8] = 0
    return pos

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
        ang_vel = 0.2
    if keys[pygame.K_e]:
        ang_vel = -0.2

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

model = mujoco.MjModel.from_xml_path(
    "/home/antoine/MISC/mini_BDX/mini_bdx/robots/open_duck_mini_v2/scene.xml"
)
# model = mujoco.MjModel.from_xml_path(
#     "/home/antoine/MISC/mujoco_menagerie/open_duck_mini_v2/scene.xml"
# )
model.opt.timestep = 0.005
# model.opt.timestep = 1 / 120
# model.opt.timestep = 1 / 60  # /2 substeps ?
# model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
data = mujoco.MjData(model)
# mujoco.mj_step(model, data)
control_decimation = 4

data.qpos[3 : 3 + 4] = [1, 0, 0.0, 0]
data.qpos[7 : 7 + 16] = init_pos
data.ctrl[:16] = init_pos

NUM_OBS = 56

policy = OnnxInfer(args.onnx_model_path, awd=True)
if args.rma:
    adaptation_module = OnnxInfer(args.adaptation_module_path, "rma_history", awd=True)
    obs_history_size = 20
    obs_history = np.zeros((obs_history_size, NUM_OBS)).tolist()
    rma_decimation = 5  # 10 hz if control_decimation = 4
    rma_counter = 0

commands = [0.2, 0.0, 0.0]

# define context variables
prev_action = np.zeros(16)
target_dof_pos = init_pos.copy()
action_scale = 0.5

kps = np.array([6.55] * 16)
kds = np.array([0.65] * 16)

counter = 0
replay_counter = 0
latent = None
start = time.time()
with mujoco.viewer.launch_passive(
    model, data, show_left_ui=False, show_right_ui=False
) as viewer:
    adaptation_module_latents = []
    while True:
        step_start = time.time()

        tau = pd_control(
            target_dof_pos,
            data.qpos[7:].copy(),
            kps,
            np.zeros_like(kds),
            data.qvel[6:].copy(),
            kds,
            3.57,
            init_pos,
            action_scale,
        )
        data.ctrl[:] = tau

        mujoco.mj_step(model, data)
        counter += 1

        if counter % control_decimation == 0 and time.time() - start > 0:
            if args.replay_obs is not None:
                obs = replay_obs[replay_counter]
                replay_counter += 1
            else:
                obs = get_obs(data, prev_action, commands)
            if args.rma:
                rma_counter += 1

                # Shift to right and set new obs at index 0
                obs_history = np.roll(obs_history, 1, axis=0)
                obs_history[0] = obs

                # obs_history.append(obs)
                # obs_history = obs_history[-obs_history_size:]

                if rma_counter % rma_decimation == 0 or latent is None:
                    latent = adaptation_module.infer(np.array(obs_history).flatten())
                    adaptation_module_latents.append(latent)
                    pickle.dump(adaptation_module_latents, open("adaptation_module_latents.pkl", "wb"))
                obs = np.concatenate([obs, latent])

            obs = np.clip(obs, -100, 100)
            action = policy.infer(obs)
            action = np.clip(action, -5, 5)
            if args.zero_head:
                action = set_zero_head(action)
            prev_action = action.copy()


            target_dof_pos = np.array(action)  # * action_scale + init_pos

        viewer.sync()

        if args.k:
            handle_keyboard()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
