# Configure the motors

Clone and install (`pip install -e .`) the runtime repo on the `v2` branch : `https://github.com/apirrone/Open_Duck_Mini_Runtime`

You can either install it on your own computer or on the raspberry pi for the configuration, as you want. You'll just want a way to power the servos, for example, the battery pack.


Then for each motor, run the following command : 

```bash
python configure_motor.py --id <id>
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
