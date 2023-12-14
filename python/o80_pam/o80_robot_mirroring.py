import o80
import o80_pam

# convenience class for sending mirroring
# (i.e. imposing joint positions and velocities)
# to a robot
class o80RobotMirroring:
    def __init__(self, segment_id, frontend=None, burster=None):

        if frontend is None:
            self._frontend = o80_pam.MirrorRobotFrontEnd(segment_id)
        else:
            self._frontend = frontend
        self._state = o80.State2d(0, 0)

        if burster is None:
            self._burster = self._frontend
        else:
            self._burster = burster

    def reset(self):
        """
        uses o80 frontend to send to the backend in overwrite mode
        a command that request the desired states to be the first states
        the backend experienced, i.e. it resets the robot to its
        original state"""
        self._frontend.add_reinit_command()
        self._frontend.pulse()

    def read(self):

        return self._frontend.latest()

    def burst(self, nb_iterations):

        return self._burster.burst(nb_iterations)

    def get(self):
        states = self._frontend.pulse().get_observed_states()
        positions = [states.get(dof).get(0) for dof in range(4)]
        velocities = [states.get(dof).get(1) for dof in range(4)]
        return positions, velocities


    def get_fk(self):
        states = self._frontend.pulse().get_observed_states()
        positions = [states.get(dof).get(0) for dof in range(4)]
        velocities = [states.get(dof).get(1) for dof in range(4)]
        # extended_states = self._frontend.pulse().get_extended_state()
        racket_pos = self._frontend.pulse().get_cartesian_position()
        racket_vel = self._frontend.pulse().get_cartesian_velocity()
        racket_ori = self._frontend.pulse().get_cartesian_orientation()
        timestamp = self._frontend.pulse().get_time_stamp()
        return positions, velocities, racket_pos, racket_vel, racket_ori, timestamp

    def set(
        self,
        joint_positions,
        joint_velocities,
        duration_ms=None,
        nb_iterations=None,
        wait=False,
        burst=False,
    ):

        if duration_ms is not None:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None

        if nb_iterations is not None:
            iteration = o80.Iteration(nb_iterations, True, True)
        else:
            iteration = None

        if duration:
            self._frontend.add_command(
                joint_positions, joint_velocities, duration, o80.Mode.OVERWRITE
            )
        else:
            self._frontend.add_command(
                joint_positions, joint_velocities, o80.Mode.OVERWRITE
            )
        if wait:
            self._frontend.pulse_and_wait()
        else:
            if burst:
                if type(burst) == type(True):
                    self._burster.burst(1)
                else:
                    self._burster.burst(burst)
            else:
                self._frontend.pulse()
