from typing import List
from collections import namedtuple

SERVO_QTY = 16

"""
These configurations describe both the servo hardware and their
constraints based on the way they're installed.

In the Servo named tuple the attributes are grouped and described
as follows:

channel - integer channel number from 0 to SERVO_QTY-1
servo_make_model - a string describing the servo
servo_angle_min_deg - integer minimum possible angle for the servo, usually 0
servo_angle_max_deg - integer maximum possible angle for the servo, usually 180
pulse_min_us - pulse duration in microseconds to command servo_angle_min_deg
pulse_max_us - pulse duration in microseconds to command servo_angle_max_deg

joint_name - a string name for the joint
joint_init_wait_ms - integer quantity of milliseconds to wait after
                     initializing the servo. the more time required
                     for the servo to move half its range, the greater
                     this value must be.
joint_angle_init_deg - integer initialization angle in degrees. since
                       there's no feedback from the servo, initialization
                       will often cause high acceleration.
joint_angle_min_deg - integer minimum possible angle for the joint
joint_angle_max_deg - integer maximum possible angle for the joint

"""

Servo = namedtuple('Servo',
                   ['channel',
                    'servo_make_model',
                    'servo_angle_min_deg',
                    'servo_angle_max_deg',
                    'pulse_min_us',
                    'pulse_max_us',
                    'joint_name',
                    'joint_init_wait_ms',
                    'joint_angle_init_deg',
                    'joint_angle_min_deg',
                    'joint_angle_max_deg'
                    ],
                   defaults=[
                       SERVO_QTY,
                       'DSSERVO,DS3235SG',
                       0,
                       180,
                       600,
                       2400,
                       None,
                       50,
                       90,
                       0,
                       180
                   ])


def servo_config() -> (List[Servo], dict, List[dict]):
    """
    Return three objects:
        - a list of Servo named tuples, each of which describes a servo.
        - a dictionary mapping servo names to the Servo named tuples
        - a list of dictionaries with two attributes: enabled and angle;

    All three objects contain SERVO_QTY instances. The two lists are
    in servo channel order so they can be indexed by channel.
    """

    servo_list = list()
    name_map = dict()
    servo_state_list = list()
    servo_list.append(Servo(channel=0,
                            joint_name='camera_pan',
                            joint_init_wait_ms=0))
    servo_list.append(Servo(channel=1,
                            servo_make_model='TIANKONGRC,MG995',
                            joint_name='camera_tilt',
                            joint_init_wait_ms=1000,
                            servo_angle_min_deg=50,
                            pulse_max_us=2500))
    servo_list.append(Servo(channel=2,
                            joint_name='shoulder_pan',
                            joint_init_wait_ms=300))
    servo_list.append(Servo(channel=3,
                            joint_name='shoulder_tilt',
                            joint_angle_init_deg=40))
    servo_list.append(Servo(channel=4,
                            joint_name='elbow',
                            joint_angle_init_deg=90))
    servo_list.append(Servo(channel=5,
                            joint_name='wrist_pan',
                            joint_angle_init_deg=60))
    servo_list.append(Servo(channel=6,
                            servo_make_model='TIANKONGRC,MG996R',
                            joint_name='wrist_tilt'))
    servo_list.append(Servo(channel=7,
                            servo_make_model='TIANKONGRC,MG995',
                            joint_name='gripper_pan'))
    servo_list.append(Servo(channel=8,
                            joint_name='gripper',
                            joint_angle_max_deg=75,
                            joint_angle_init_deg=0))
    servo_list.append(Servo(channel=9,
                            servo_make_model=None))
    servo_list.append(Servo(channel=10,
                            servo_make_model=None))
    servo_list.append(Servo(channel=11,
                            servo_make_model=None))
    servo_list.append(Servo(channel=12,
                            servo_make_model=None))
    servo_list.append(Servo(channel=13,
                            servo_make_model=None))
    servo_list.append(Servo(channel=14,
                            servo_make_model=None))
    servo_list.append(Servo(channel=15,
                            servo_make_model=None))

    for servo in servo_list:
        name_map[servo.joint_name] = servo
        servo_state_list.append(
            {'enabled': False,
             'angle': servo.joint_angle_init_deg})

    return servo_list, name_map, servo_state_list
