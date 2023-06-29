from typing import List
from collections import namedtuple

SERVO_QTY = 16

Servo = namedtuple('Servo',
                   ['channel',
                    'name',
                    'init_delay_s',
                    'angle_min_deg',
                    'angle_max_deg',
                    'angle_init_deg',
                    'pulse_min',
                    'pulse_max'
                    ],
                   defaults=[
                       SERVO_QTY,
                       'undefined',
                       0.05,
                       0,
                       180,
                       90,
                       600,
                       2400
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
                            name='camera_pan',
                            init_delay_s=0))
    servo_list.append(Servo(channel=1,
                            name='camera_tilt',
                            init_delay_s=1,
                            angle_min_deg=50,
                            pulse_max=2500))
    servo_list.append(Servo(channel=2,
                            name='shoulder_pan',
                            init_delay_s=0.3))
    servo_list.append(Servo(channel=3,
                            name='shoulder_tilt',
                            angle_init_deg=40,
                            angle_max_deg=115))
    servo_list.append(Servo(channel=4,
                            name='elbow',
                            angle_max_deg=60,
                            angle_min_deg=5,
                            angle_init_deg=5))
    servo_list.append(Servo(channel=5,
                            name='wrist_pan',
                            angle_init_deg=60))
    servo_list.append(Servo(channel=6,
                            name='wrist_tilt'))
    servo_list.append(Servo(channel=7,
                            name='gripper_pan'))
    servo_list.append(Servo(channel=8,
                            name='gripper',
                            angle_max_deg=75,
                            angle_init_deg=0))
    servo_list.append(Servo(channel=9))
    servo_list.append(Servo(channel=10))
    servo_list.append(Servo(channel=11))
    servo_list.append(Servo(channel=12))
    servo_list.append(Servo(channel=13))
    servo_list.append(Servo(channel=14))
    servo_list.append(Servo(channel=15))

    for servo in servo_list:
        name_map[servo.name] = servo
        servo_state_list.append(
            {'enabled': False,
             'angle': servo.angle_init_deg})

    return servo_list, name_map, servo_state_list
