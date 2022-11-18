#!/usr/bin/env python3

"""
Parse and replay a file generated by o80_robot_ball_logger.
Prior to run this executable, an instance of pam_mujoco must be start
with the mujoco_id "robot_ball_replay".
It is assumed the file is in the current directory and named o80_robot_ball_x 
(where 'x' is an integer).  
"""


import lightargs

Vector3d = typing.Tuple[float,float,float]
Vector4d = typing.Tuple[float,float,float,float]

def _get_frontends(
        robot_type: pam_mujoco.RobotType,
        mujoco_id: str
)-> typing.Tuple[pam_mujoco.MirrorRobotFrontEnd,pam_mujoco.BallFrontEnd]:
    """
    Returns the ball and the robot frontend. 

    Args:
      robot_type: pam_mujoco.RobotType.PAMY1 or pam_mujoco.RobotType.PAMY1
      mujoco_id: mujoco_id of the pam_mujoco instance this function will configure

    Returns
      the frontends
    """
    
    robot = pam_mujoco.MujocoRobot(robot_type, "robot", control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
    ball = pam_mujoco.MujocoItem(
        "ball", control=pam_mujoco.MujocoItem.CONSTANT_CONTROL, color=(1, 0, 0, 1)
    )
    table = pam_mujoco.MujocoTable("table")
    graphics = True
    accelerated_time = False
    handle = pam_mujoco.MujocoHandle(
        mujoco_id,
        table=table,
        robot1=robot,
        balls=(ball,),
        graphics=graphics,
        accelerated_time=accelerated_time,
    )

    return handle.frontends["robot"], handle.frontends["ball"]


class Ball:
    """
    Information regarding the ball at a given step.
    A negative ball_id means the ball was not detected
    during this step
    """
    def __init__(
            self,
            ball_id: int,
            position: Vector3d,
            velocity: Vectory3d
    ):
        self.ball_id = ball_id  
        self.position = position
        self.velocity = velocity

class Robot:
    """
    Information regarding the robot at a given step.
    Position in radian and velocity in radian per second.
    """
    def __init__(
            self,
            position: Vector4d,
            velocity: Vector4d
    ):
        self.position = position
        self.velocity = velocity


Step = typing.Tuple[int, Ball, Robot]
""" represent the observation at a given step, the first value being the timestamp (in nanoseconds) """


def _readline(line: str)->Step:
    """
    Parse a line of the file, and returns the corresponding
    observation.
    """
    entries = eval(line)
    timestamp = entries[1][0]
    ball_id = entries[0][0]
    ball_position = entries[0][2]
    ball_velocity = entries[0][3]
    ball = Ball(ball_id, ball_position, ball_velocity)
    robot_position = entries[1][1]
    robot_velocity = entries[1][2]
    robot = Robot(robot_position, robot_velocity)
    return timestamp, ball, robot
    

def _set_ball_commands(
        ball_frontend: pam_mujoco.BallFrontEnd,
        steps: typing.List[Step]
)->None:
    """
    Buffer all the commands required to replay the ball trajetory
    encoded by the steps. For step during which the ball was not detected
    (negative ball_id): the step is skipped, as the ball interpolates between
    the steps during which it was detected.
    """
    def _get_next(index: int, steps: typing.List[Step] )->typing.Optional[typing.Tuple[int,int]]:
        if index+1 == len(steps):
            return None
        timestamp = steps[index][0]
        for nb,i in enumerate(range(index+1, len(steps))):
            if steps[i].ball_id >= 0:
                next_timestamp = steps[i][0]
                return next_timestamp-timestamp, i
        return None

    def _set_command(duration: int, step: Step)->None:
        ball = step[1]
        ball_frontend.add_command(
            ball.position,
            ball.velocity,
            o80.Duration_us.nanoseconds(duration),
            o80.Mode.QUEUE
        )
    
    index = 0
    while True:
        next_ = _get_next(index, steps)
        if next_ is None:
            return
        duration, index = _next
        _set_command(duration, steps[index])


def _set_robot_commands(
        robot_frontend: pam_mujoco.MirrorRobotFrontEnd,
        steps: typing.List[Step]
)->None:
    """
    Buffer all the commands required to replay the robot's joint trajetories
    encoded by the steps.
    """
    for step1, step2 in zip(steps,steps[1:]):
        robot = step2[2]
        robot_frontend.add_command(
            robot.position,
            robot.velocity,
            o80.Duration_us.nanoseconds(step2[0]-step1[0]),
            o80.Mode.QUEUE
        )

        
def _replay(
        path: Path
        ball_frontend: pam_mujoco.BallFrontEnd,
        robot_frontend: pam_mujoco.MirrorRobotFrontEnd
):
    """
    Parse the file and replay the corresponding ball and robot trajectories.
    """

    # reading the file
    with open(path,"r") as f:
        lines = f.readlines()

    # parsing the content of the file
    steps: typing.List[Step]
    steps = [_readline(line) for line lines]

    # loading trajectories
    _set_ball_commands(ball_frontend,steps)
    _set_robot_commands(robot_frontend,steps)

    # playing
    ball_frontend.pulse()
    robot_frontend.pulse()
    
        
def _configure() -> Path, pam_mujoco.RobotType:
    """
    Configuration dialog
    """

    def _logfile()->Path:
        """
        List the files named 'o80_robot_ball_*' located in the current
        directory and returns the first one (alphabetically)
        """
        current = Path(os.getcwd())
        fileprefix = "o80_robot_ball_"
        files = [f for f in list(current.blog(fileprefix)) if f.is_file()]
        if not files:
            raise FileNotFoundError(
                f"failed to find in the current directory a file which names starts with {fileprefix}"
            )
        return sorted(files)[0]

    # the default file to replay
    default_file = _logfile()
        
    config = lightargs.BrightArgs("o80 robot / tennicam replay")

    # to select the file to replay
    config.add_option(
        "filepath",
        str(default_file),
        "absolute path to the file to replay",
        str,
        integrity_checks = (lightargs.FileExists(),)
    )

    # to select the robot (Pamy1 or Pamy2)
    config.add_option(
        "robot_type",
        "PAMY1",
        "PAM1 or PAMY2",
        str,
        integrity_checks = (lightargs.Set("PAMY1","PAMY2"),))

    # configuring
    change_all = False
    config.dialog(change_all, sys.argv[1:])

    # robot type
    if config.robot_type == "PAMY1":
        robot_type = pam_mujoco.RobotType.PAMY1
    else:
        robot_type = pam_mujoco.RobotType.PAMY2
    
    # returning
    return Path(config.filepath), robot_type


        

    
