import multiprocessing
import shared_memory
import o80

from .o80_pressures import o80Pressures
from .o80_robot_mirroring import o80RobotMirroring


def _mirroring(mirror_id, segment_id_mirror_robot, segment_id_pressures, period_ms):
    o80_mirroring = o80RobotMirroring(segment_id_mirror_robot)
    o80_pressure = o80Pressures(segment_id_pressures)

    frequency_manager = o80.FrequencyManager(1.0 / (period_ms / 1000.0))

    running = True
    segment_id = mirror_id + "_running"

    while running:
        _, __, joint_positions, joint_velocities = o80_pressure.read()
        o80_mirroring.set(joint_positions, joint_velocities)
        frequency_manager.wait()

        running = shared_memory.get_bool(segment_id, "running")


def stop_mirroring(mirror_id):
    shared_memory.set_bool(mirror_id + "_running", "running", False)


def start_mirroring(
    mirror_id, segment_id_mirror_robot, segment_id_pressure_robot, period_ms
):
    shared_memory.clear_shared_memory(mirror_id + "_running")
    shared_memory.set_bool(mirror_id + "_running", "running", True)

    process = multiprocessing.Process(
        target=_mirroring,
        args=(
            mirror_id,
            segment_id_mirror_robot,
            segment_id_pressure_robot,
            period_ms,
        ),
    )
    process.start()
    return process
