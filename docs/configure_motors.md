# Configure the motors

> For now, we use [LeRobot's](https://github.com/huggingface/lerobot) motor configuration script. We'll add a simple configuration script here soon.

Clone and install LeRobot.

Then for each motor, run the following command : 

```bash
python lerobot/scripts/configure_motor.py --port /dev/ttyACM0 --brand feetech --model sts3215 --baudrate 1000000 --ID <ID>
```

The motors ids are : 

```python
{
    "left_hip_yaw": 20,
    "left_hip_roll": 21,
    "left_hip_pitch": 22,
    "left_knee": 23,
    "left_ankle": 24,
    "neck_pitch": 30,
    "head_pitch": 31,
    "head_yaw": 32,
    "head_roll": 33,
    "right_hip_yaw": 10,
    "right_hip_roll": 11,
    "right_hip_pitch": 12,
    "right_knee": 13,
    "right_ankle": 14,
}
```