# Training policies that transfer to the real robot (sim2real)

We want to train policies that transfer well to the real robot. This is the sim2real problem. It's a hard problem, especially for us since we are using cheap servomotors that are hard to model and not overly powerful. 

Below, I'll roughly explain the steps we went through to get there, but you won't have to do everything again yourself since we provide the results of each process.

## Make an accurate model of the robot (URDF/MJCF)

### Robot structure

In the [Onhape document](https://cad.onshape.com/documents/64074dfcfa379b37d8a47762/w/3650ab4221e215a4f65eb7fe/e/0505c262d882183a25049d05), we specify the material of each part. But to be more accurate, because we print with infill we override the mass of the parts with the (pretty accurate) estimation of the slicer. 

We use [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) to export URDF/MJCF descriptions. For MJX, we have to make a lightweight model, see our [config.json](https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/config.json). 

This gives us a MJCF (Mujoco format) [description of the robot](https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml) which describes the masses and moments of inertia of the full robot. 

### Motors

Another very important part of having an accurate model is modeling the motors's behavior. We use [BAM](https://github.com/Rhoban/bam/) for that. You don't have to go through the identification process yourself, we provide the results [here](https://github.com/Rhoban/bam/tree/main/params/feetech_sts3215_7_4V)

It's critical that the simulator simulates the motors accurately, because we will train a policy (a neural network) to output motor positions based on sensory inputs (motors positions/speeds, imu and feet sensors). If the motors behave differently in the simulation than in the real world, the policy won't work, or at worst, produce chaotic movements.

`BAM` allows us to export the main identified parameters to mujoco units (using `bam.to_mujoco`). These values are the ones we set in out mjcf model for the actuators and joints properties. 

- damping
- kp
- frictionloss
- armature
- forcerange

## Training policies 

