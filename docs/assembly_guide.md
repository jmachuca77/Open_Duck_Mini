# Assembly guide (INCOMPLETE)

## Requirements : 

You will need : 
- A soldering iron, and basic electronics tools and skills
- X m3 screws (TODO : add the exact number)
- Some wire
- Loctite Threadlocker blue 243

First, we build the robot's skeleton 

## Parts needed : 
- foot_top.stl x2
- foot_side.stl x2
- foot_bottom_tpu.stl x2 (TPU)
- foot_bottom_pla.stl x2
- knee_to_ankle_left_sheet.stl x4
- knee_to_ankle_right_sheet.stl x4
- leg_spacer.stl x4
- left_roll_to_pitch.stl x1
- right_roll_to_pitch.stl x1
- roll_motor_bottom.stl x2
- roll_motor_top.stl x2
- trunk_bottom.stl x1
- trunk_top.stl x1
- neck_left_sheet.stl x1
- neck_right_sheet.stl x1
- head_pitch_to_yaw.stl x1
- head_yaw_to_roll.stl x1
- head_roll_mount.stl x1

> General note : Everytime you screw something in the motors, you want to use a little loctite threadlocker. This will prevent the screws from coming loose due to the vibrations during the operation of the robot. It adds a little time to to the build, but you'll be glad you took the time ;)

> At any time, you can refer to the CAD here : https://cad.onshape.com/documents/64074dfcfa379b37d8a47762/w/3650ab4221e215a4f65eb7fe/e/0505c262d882183a25049d05

## Steps :

### Assemble the trunk

Place the bearings in `trunk_bottom` like so, and insert M3 inserts in these holes

<img src="https://github.com/user-attachments/assets/9ed8591a-7c96-4410-8d7e-9b7d88c6bd1f" alt="1" width="500px" >

Then assamble `trunk_bottom` and `trunk_top`, and screw them together with 2 `M3x10` screws through these holes

<img src="https://github.com/user-attachments/assets/ae36b396-a34a-4691-8e62-fd916cd1f76c" alt="1" width="500px" >

Mount the middle motor like so and screw it with the plastic screws that came with the motors : 

<img src="https://github.com/user-attachments/assets/d4a6ba6c-852f-440e-afb7-3b9ca66ad3dc" alt="1" width="500px" >

Insert `roll_motor_bottom` like this 

<img src="https://github.com/user-attachments/assets/31c031b7-58b8-4c4c-a3fa-1f14a310c2f2" alt="1" width="500px" >


### Assemble the feet

Both feet are the same. 

First, assemble `foot_bottom_tpu` with `foot_bottom_pla`. Insert M3 inserts in these holes :

<img src="https://github.com/user-attachments/assets/6749a5ba-cea9-4b0a-ac32-f32e130fd057" alt="1" width="500px" >

And screw the two parts together with two `M3x6` screws.

Then, insert M3 inserts in these holes in `foot_top` here : 

<img src="https://github.com/user-attachments/assets/1a77f2f8-56ea-43d2-91c7-78130456c45b" alt="1" width="500px" >

And assemble everything like so. Make sure the driver side of the motor is on the `foot_top` part side : 

<table>
  <tr>
    <td> <img src="https://github.com/user-attachments/assets/197cd9b7-05a6-46ed-9af5-d2def37970c8" alt="1" width="500px" > </td>
    <td> <img src="https://github.com/user-attachments/assets/f532fbcc-100b-4715-96a8-66697e4fda26" alt="1" width="500px" > </td>
   </tr> 
</table>

### Assemble the shins

Insert M3 Inserts in these holes of `leg_spacer` (on both sides. Insert 4 M3 inserts in total) :

<img src="https://github.com/user-attachments/assets/41eb01a2-d6f9-43b1-a83e-d14785907425" alt="1" width="500px" >

Then, first plug the motor cable in the foot's motor, and make it go through the `right_sheet` like so 

<img src="https://github.com/user-attachments/assets/a1432505-6b6c-4765-aea6-0a7a0b8b220b" alt="1" width="500px" >

Then assemble like below:

<img src="https://github.com/user-attachments/assets/54615217-fec9-46dd-8b40-e21a4f527b55" alt="1" width="500px" >

### Assemble the thighs 

The thigh is pretty much the same thing, except the `hip_pitch` motor is mounted this way (important for the zero position)

<img src="https://github.com/user-attachments/assets/951157a4-26dc-41bb-b97f-18dbbb0c1cd3" alt="1" width="500px" >

### Assemble the hips

Mount `left_roll_to_pitch` or `right_roll_to_pitch`, here the parts are symmetrical so you have to use the right one.

<img src="https://github.com/user-attachments/assets/368a85ad-1c58-4db2-bdd0-812334b6c784" alt="1" width="500px" >

Mount `roll_motor_top` to the `hip_yaw` servo (screw from the bottom). Don't mount the servo to the trunk yet.


<img src="https://github.com/user-attachments/assets/768f1664-030c-4432-9071-c52d79d3ef6b" alt="1" width="500px" >

Then mount `hip_roll` like this 

<img src="https://github.com/user-attachments/assets/e1287f67-320c-42cf-a5dc-84f4c64abb17" alt="1" width="500px" >

And insert the sub assembly like this 

<img src="https://github.com/user-attachments/assets/dba458ae-e9cb-4e16-a82e-cbe74d2350fa" alt="1" width="500px" >

Screw everything you can (with the plastic screws provided with the servos)

You can now mount the leg like this :

<img src="https://github.com/user-attachments/assets/a7169791-9f0a-4827-aea7-00ab1d0f6212" alt="1" width="500px" >

And do the same for the other leg :)

Your duck should now look like this 

<img src="https://github.com/user-attachments/assets/4921b29b-b38b-423d-9da1-1c8f84689e84" alt="1" width="500px" >

### Assemble the neck

You know the drill

<img src="https://github.com/user-attachments/assets/f96fe44f-a7bc-423e-a925-4aef3e7bf568" alt="1" width="500px" >

### Assemble the head

First, mount `head_pitch_to_yaw` like this 

<img src="https://github.com/user-attachments/assets/a4dba395-eb0d-4150-8980-8e14f1173d02" alt="1" width="500px" >

Then, independently mount `head_yaw_to_roll` and `head_roll_mount` to `head_roll dof`

<img src="https://github.com/user-attachments/assets/4648cd6c-391e-41e4-9617-4ecebfa9215b" alt="1" width="500px" >

Then 

<img src="https://github.com/user-attachments/assets/03f3bdae-06c3-4c37-b68f-14587edd6123" alt="1" width="500px" >

Then 

<img src="https://github.com/user-attachments/assets/04a219d8-bbaf-4910-b5d3-7ca67bce466c" alt="1" width="500px" >

Your duck should now look like this 

<img src="https://github.com/user-attachments/assets/71545d40-f0f5-411d-a8d5-1cd676a74e75" alt="1" width="500px" >

### Mount the servo driver board

TODO take a photo

### Mount the IMU

Like this 

<table>
  <tr>
    <td> <img src="https://github.com/user-attachments/assets/8772996d-5906-48fa-8cbe-6b6823982375" alt="1" width="500px" ></td>
    <td> <img src="https://github.com/user-attachments/assets/172f44c7-c7c6-47f8-894a-14024abd24f4" alt="2" width="500px" ></td>
   </tr> 
</table>

## Electronics

### Battery pack

### Head

## The rest

Body parts
