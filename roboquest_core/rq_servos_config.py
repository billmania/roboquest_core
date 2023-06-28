from typing import List
from collections import namedtuple

SERVO_QTY = 16

Servo = namedtuple('Servo',
                   ['id',
                    'init_delay_s',
                    'angle_max_deg',
                    'angle_init_deg',
                    'pulse_min',
                    'pulse_max'
                    ],
                   defaults=[
                       SERVO_QTY,
                       0.05,
                       180,
                       90,
                       600,
                       2400
                   ])


def servo_config() -> (List[Servo], List[dict]):
    """
    Return two lists:
        - Servo named tuples, each of which describes a servo.
        - dictionary with two attributes: enabled and angle;

    Each list contains SERVO_QTY instances, in servo ID order.
    """

    servo_list = list()
    servo_state_list = list()
    servo_list.append(Servo(id=0, init_delay_s=0))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=1, init_delay_s=1, pulse_max=2500))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=2, init_delay_s=0.3))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=3))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=4))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=5))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=6))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=7))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=8))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=9))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=10))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=11))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=12))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=13))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=14))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})
    servo_list.append(Servo(id=15))
    servo_state_list.append(
        {'enabled': False,
         'angle': servo_list[len(servo_list)-1].angle_init_deg})

    return servo_list, servo_state_list
