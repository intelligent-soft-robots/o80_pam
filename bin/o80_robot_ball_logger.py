#!/usr/bin/env python3

"""
Dump the output of a tennicam_client 
frontend and of a o80_pam frontend into a file,
with format for each line:
 repr( ( (ball_id,ball_time_stamp,ball_position,ball_velocity),
         (robot_time_stamp,robot_positions,robot_velocities) ) )
with:
- ball_id : int
- ball_time_stamp: int (nanoseconds)
- ball_position: 3d tuple
- ball_velocity: 3d tuple
- robot_time_stamp: int (nanoseconds)
- robot_positions: 4d tuple
- robot_velocities: 4d tuple
Dumping is done at tennicam_client running frequency
"""

import sys
import pathlib
import o80
import signal_handler
import tennicam_client
import o80_pam
from lightargs import BrightArgs

TENNICAM_CLIENT_DEFAULT_SEGMENT_ID = "tennicam_client"
O80_PAM_DEFAULT_SEGMENT_ID = "real_robot"
DEFAULT_SAVE_FOLDER = pathlib.Path("/tmp")


def _unique_path(
    directory: pathlib.Path, name_pattern="o80_robot_ball_{:03d}"
) -> pathlib.Path:
    """
    returns the path to a file /directory/tennicam_{x} that does not exists yet
    """

    counter = 0
    while True:
        counter += 1
        path = directory / name_pattern.format(counter)
        if not path.exists():
            return path


def _run(
    tennicam_segment_id: str,
    o80_pam_segment_id: str,
    frequency: float,
    filepath: pathlib.Path,
):
    """
    Creates an tennicam_client frontend and a o80_pam frontend and
    uses them to get all ball observations and pam robot observations;
    and dumping them in a corresponding string in filepath.
    """

    ball_frontend = tennicam_client.FrontEnd(tennicam_segment_id)
    robot_frontend = o80_pam.FrontEnd(o80_pam_segment_id)
    iteration = ball_frontend.latest().get_iteration()

    ball_getters = ("get_ball_id", "get_time_stamp", "get_position", "get_velocity")
    robot_getters = ("get_time_stamp", "get_positions", "get_velocities")

    frequency_manager = o80.FrequencyManager(frequency)

    with filepath.open(mode="w") as f:
        try:
            while not signal_handler.has_received_sigint():
                iteration += 1
                obs_ball = ball_frontend.latest()
                obs_robot = robot_frontend.latest()
                ball_values = tuple(
                    (getattr(obs_ball, getter)() for getter in ball_getters)
                )
                robot_values = tuple(
                    (getattr(obs_robot, getter)() for getter in robot_getters)
                )
                str_ = repr((ball_values, robot_values))
                f.write(str_)
                f.write("\n")
                frequency_manager.wait()
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception as e:
            print("Error:", e)


def _configure() -> BrightArgs:
    """
    Configuration dialog
    """

    global TENNICAM_CLIENT_DEFAULT_SEGMENT_ID
    global DEFAULT_SAVE_FOLDER
    global O80_PAM_DEFAULT_SEGMENT_ID
    config = BrightArgs("o80 robot / tennicam logger")
    config.add_option(
        "ball_segment_id",
        TENNICAM_CLIENT_DEFAULT_SEGMENT_ID,
        "ball_segment_id of the tennicam backend",
        str,
    )
    config.add_option(
        "robot_segment_id",
        O80_PAM_DEFAULT_SEGMENT_ID,
        "robot_segment_id of the o80_pam backend",
        str,
    )
    config.add_option(
        "filepath",
        str(_unique_path(DEFAULT_SAVE_FOLDER)),
        "absolute path of the log file",
        str,
    )
    config.add_option(
        "frequency",
        250.0,
        "frequency at which entries will be written in the file",
        float,
    )
    change_all = False
    config.dialog(change_all, sys.argv[1:])
    print()
    return config


if __name__ == "__main__":
    config = _configure()
    ball_segment_id = config.ball_segment_id
    robot_segment_id = config.robot_segment_id
    frequency = config.frequency
    filepath = pathlib.Path(config.filepath)
    _run(ball_segment_id, robot_segment_id, frequency, filepath)
