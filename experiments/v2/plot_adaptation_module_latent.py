import pickle

# [[x_t0, y_t0, z_t0...], [x_t1, y_t1, z_t1...], ...]
# adaptation_module_latent = pickle.load(open("adaptation_module_latents.pkl", "rb"))
adaptation_module_latent = pickle.load(open("robot_latents.pkl", "rb"))

from matplotlib import pyplot as plt

plt.plot(adaptation_module_latent)
plt.show()

