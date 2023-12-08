from typing import List

SERVO_QTY = 16

"""
These configurations describe both the servo hardware and their
constraints based on the way they're installed. This mechanism exists
only as a way to create an initial or default servo configuration.
The actual servo configuration is managed by the ConfigFile class and
held in a persistent configuration file.

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


class Servo(object):
    """
    The configuration details for a servo, managed as a dictionary to
    support several requirements: write the details to a human-editable
    JSON file; provide default values for all elements of the class;
    instantiate the class with a variable quantity of arguments; provide
    direct access to each member of the object.
    """

    def __init__(
         self,
         channel=SERVO_QTY,
         servo_make_model='DSSERVO,DS3235SG',
         servo_angle_min_deg=0,
         servo_angle_max_deg=180,
         pulse_min_us=600,
         pulse_max_us=2400,
         joint_name=None,
         joint_init_wait_ms=50,
         joint_angle_init_deg=90,
         joint_angle_min_deg=0,
         joint_angle_max_deg=180
         ):
        """
        Create the dictionary.
        """

        self._servo_config = dict()
        self._servo_config['channel'] = channel
        self._servo_config['servo_make_model'] = servo_make_model
        self._servo_config['servo_angle_min_deg'] = servo_angle_min_deg
        self._servo_config['servo_angle_max_deg'] = servo_angle_max_deg
        self._servo_config['pulse_min_us'] = pulse_min_us
        self._servo_config['pulse_max_us'] = pulse_max_us
        self._servo_config['joint_name'] = joint_name
        self._servo_config['joint_init_wait_ms'] = joint_init_wait_ms
        self._servo_config['joint_angle_init_deg'] = joint_angle_init_deg
        self._servo_config['joint_angle_min_deg'] = joint_angle_min_deg
        self._servo_config['joint_angle_max_deg'] = joint_angle_max_deg

    def get(self):
        """
        Return the Servo's dictionary.
        """

        return self._servo_config


def servo_config() -> List[dict]:
    """
    Return a list of Servo dictionaries, each of which describes a servo.
    """

    servo_list = list()
    servo_list.append(Servo(channel=0,
                            joint_name='camera_pan',
                            joint_init_wait_ms=0).get())
    servo_list.append(Servo(channel=1,
                            servo_make_model='TIANKONGRC,MG995',
                            joint_name='camera_tilt',
                            joint_init_wait_ms=1000,
                            servo_angle_min_deg=50,
                            pulse_max_us=2500).get())
    servo_list.append(Servo(channel=2,
                            joint_name='shoulder_pan',
                            joint_init_wait_ms=300).get())
    servo_list.append(Servo(channel=3,
                            joint_name='shoulder_tilt',
                            joint_angle_init_deg=40).get())
    servo_list.append(Servo(channel=4,
                            joint_name='elbow',
                            joint_angle_init_deg=90).get())
    servo_list.append(Servo(channel=5,
                            joint_name='wrist_pan',
                            joint_angle_init_deg=60).get())
    servo_list.append(Servo(channel=6,
                            servo_make_model='TIANKONGRC,MG996R',
                            joint_name='wrist_tilt').get())
    servo_list.append(Servo(channel=7,
                            servo_make_model='TIANKONGRC,MG995',
                            joint_name='gripper_pan').get())
    servo_list.append(Servo(channel=8,
                            joint_name='gripper',
                            joint_angle_max_deg=75,
                            joint_angle_init_deg=0).get())
    servo_list.append(Servo(channel=9,
                            servo_make_model=None).get())
    servo_list.append(Servo(channel=10,
                            servo_make_model=None).get())
    servo_list.append(Servo(channel=11,
                            servo_make_model=None).get())
    servo_list.append(Servo(channel=12,
                            servo_make_model=None).get())
    servo_list.append(Servo(channel=13,
                            servo_make_model=None).get())
    servo_list.append(Servo(channel=14,
                            servo_make_model=None).get())
    servo_list.append(Servo(channel=15,
                            servo_make_model=None).get())

    return servo_list


def servo_map_and_state(servo_list: List[dict]) -> (dict, list):
    """
    Accept as input the list of servos and return two things:
    - a dictionary mapping each servo name to the corresponding servo
      dictionary
    - a list of dictionaries, one for each servo in channel number order,
      which holds the current state of that servo
    """

    name_map = dict()
    servo_state_list = list()

    for servo in servo_list:
        name_map[servo['joint_name']] = servo
        servo_state_list.append(
            {'enabled': False,
             'angle': servo['joint_angle_init_deg'],
             'command_angle': None,
             'command_timestamp': None})

    return name_map, servo_state_list
