import typing
import pathlib

Position = typing.Sequence[float]
Velocity = typing.Sequence[float]
Entry = typing.Tuple[
    typing.Tuple[int, int, Position, Velocity], typing.Tuple[int, Position, Velocity]
]


def parse(filepath: pathlib.Path) -> typing.Generator[Entry, None, bool]:
    """
    Parse a file generated via o80_robot_ball_logger
    and yield the corresponding information about the ball and the robot

    Args:
        filepath: absolute path of the file

    Returns:
        tuple of two tuples, the first is:
        (ball_id: int, ball_time_stamp: int, ball_position: 3d tuple, ball_velocity: 3d tuple);
        and the second is:
        (robot_time_stamp: int, robot_positions: 4d tuple, robot_velocity: 4d tuple);
        time_stamp is in nanoseconds.
        ball_id is -1 if invalid ball (ball not detected by the visual tracking)
    """

    if not filepath.exists():
        raise FileNotFoundError(
            "o80_robot_ball_parser: failed to find: {}".format(filepath)
        )

    with open(filepath) as f:
        for line in f:
            try:
                yield eval(line)
            except:
                return False

    return True
