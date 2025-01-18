import mujoco
import numpy as np
import mujoco_viewer
from mini_bdx_runtime.rl_utils import mujoco_joints_order

model = mujoco.MjModel.from_xml_path(
    "/home/antoine/MISC/mini_BDX/mini_bdx/robots/open_duck_mini_v2/scene.xml"
)
model.opt.timestep = 0.001
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
viewer = mujoco_viewer.MujocoViewer(model, data)

target = [0] * 16
# data.ctrl[:] = np.zeros((16))
id = 11
while True:
    target[id] = 0.2*np.sin(2*np.pi*0.5*data.time)
    tau = 7*(np.array(target) - data.qpos) - 0.1*data.qvel
    print(tau)
    data.ctrl[:] = tau  
    for i, joint_name in enumerate(mujoco_joints_order):
        print(f"{joint_name}: {np.around(data.qpos[i], 2)}")
    print("==")
    mujoco.mj_step(model, data, 15)
    viewer.render()