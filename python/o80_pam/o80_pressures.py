import o80
import o80_pam


class _Data:
    def __init__(self, obs):
        pressures = obs.get_observed_pressures()
        self.pressures_ago = [pressures[dof][0] for dof in range(4)]
        self.pressures_antago = [pressures[dof][1] for dof in range(4)]
        self.robot_joints = obs.get_positions()
        self.robot_joint_velocities = obs.get_velocities()


# convenience class
# for sending pressure commands to a robot
class o80Pressures:
    def __init__(self, segment_id, frontend=None, burster=None):
        if frontend is None:
            self._frontend = o80_pam.FrontEnd(segment_id)
        else:
            self._frontend = frontend

        if burster is None:
            self._burster = self._frontend
        else:
            self._burster = burster

    def reset(self):
        """
        uses o80 frontend to send to the backend in overwrite mode
        a command that request the desired states to be the first states
        the backend experienced, i.e. it resets the robot to its
        original state (as far as muscle pressures are concerned).
        """
        self._frontend.add_reinit_command()
        self._frontend.pulse()

    def get_iteration(self):

        return self._frontend.pulse().get_iteration()

    def burst(self, nb_iterations):

        self._frontend.pulse()
        self._burster.burst(nb_iterations)

    def add_command(self, action, duration_ms=None):

        if duration_ms:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None

        for dof, (ago_pressure, antago_pressure) in enumerate(action):
            if duration:
                self._frontend.add_command(
                    dof, ago_pressure, antago_pressure, duration, o80.Mode.OVERWRITE
                )
            else:
                self._frontend.add_command(
                    dof, ago_pressure, antago_pressure, o80.Mode.OVERWRITE
                )

    def set(self, action, duration_ms=None, wait=False, burst=False):

        self.add_command(action, duration_ms)

        if wait:
            self._frontend.pulse_and_wait()
        else:
            if burst:
                if type(burst) == type(True):
                    self._frontend.pulse()
                    self._burster.burst(1)
                else:
                    self._frontend.pulse()
                    self._burster.burst(burst)
            else:
                self._frontend.pulse()

    def read(self, desired=False):

        obs = self._frontend.latest()
        if desired:
            pressures = obs.get_desired_pressures()
        else:
            pressures = obs.get_observed_pressures()
        pressures_ago = [pressures[dof][0] for dof in range(4)]
        pressures_antago = [pressures[dof][1] for dof in range(4)]

        robot_joints = obs.get_positions()
        robot_joint_velocities = obs.get_velocities()

        return pressures_ago, pressures_antago, robot_joints, robot_joint_velocities

    def get_data(self, start_iteration):

        observations = self._frontend.get_observations_since(start_iteration)
        data = [_Data(obs) for obs in observations]

        return data
