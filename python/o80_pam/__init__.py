from o80_pam_wrp import *

from .o80_ball import o80Ball
from .o80_real_ball import o80RealBall
from .o80_hit_point import o80HitPoint
from .o80_goal import o80Goal
from .o80_pressures import o80Pressures
from .o80_robot_mirroring import o80RobotMirroring
from .position_control import PositionController
from .mirroring import start_mirroring, stop_mirroring
from .segment_ids import segment_ids
from .mujoco_id import mujoco_id
from .logger import Logger, read_file, FileManager
from .run_robot import run, run_dummy_robot
from . import robot_ball_parser
from . import observation_convertors

BallFrontEnd = MirrorFreeJointFrontEnd
GoalFrontEnd = MirrorFreeJointFrontEnd
HitPointFrontEnd = MirrorFreeJointFrontEnd

BallObservation = MirrorFreeJointObservation
GoalObservation = MirrorFreeJointObservation
HitPointObservation = MirrorFreeJointObservation

JointFrontEnd = MirrorRobotFrontEnd
JointObservation = MirrorRobotObservation
