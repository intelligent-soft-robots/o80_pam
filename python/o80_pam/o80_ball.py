import typing
import o80
import o80_pam
import context


class _Data:
    def __init__(self, observation):
        ball_states = observation.get_observed_states()
        self.ball_position = [None] * 3
        self.ball_velocity = [None] * 3
        for dim in range(3):
            self.ball_position[dim] = ball_states.get(2 * dim).get()
            self.ball_velocity[dim] = ball_states.get(2 * dim + 1).get()


# convenience class for shooting virtual balls
# via o80, playing pre-recorded trajectories (hosted in context package)
class o80Ball:
    def __init__(self, segment_id, frontend=None):

        if frontend is None:
            self._frontend = o80_pam.MirrorFreeJointFrontEnd(segment_id)
        else:
            self._frontend = frontend

    def burst(self, nb_iterations):

        self._frontend.burst(nb_iterations)

    def reset(self):
        """
        send via the frontend an overwrite command requesting the backend
        to set the desired states as the first state it ever observed, i.e.
        to reset the object to its initial state.
        """
        self._frontend.add_reinit_command()
        self._frontend.pulse()

    def get_iteration(self):

        return self._frontend.pulse().get_iteration()


    def iterate_trajectory(self, trajectory_iterator: typing.Generator[context.ball_trajectories.DurationPoint,None,None], overwrite=False):

        if overwrite:
            mode = o80.Mode.OVERWRITE
        else:
            mode = o80.Mode.QUEUE

        for duration,state in trajectory_iterator:
            self._frontend.add_command(state.get_position(),
                                       state.get_velocity(),
                                       o80.Duration_us.microseconds(duration),
                                       mode)
            mode = o80.Mode.QUEUE

        self._frontend.pulse()
            
    
    def play_trajectory(self, trajectory: context.ball_trajectories.StampedTrajectory, overwrite=False):

        iterator = context.BallTrajectories.iterate(trajectory)
        self.iterate_trajectory(iterator,overwrite=overwrite)


    def set(self, position, velocity, duration_ms=None, wait=False):

        if duration_ms is not None:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None

        if duration is None:
            self._frontend.add_command(position, velocity, o80.Mode.OVERWRITE)
        else:
            self._frontend.add_command(position, velocity, duration, o80.Mode.OVERWRITE)
        if wait:
            self._frontend.pulse_and_wait()
        else:
            self._frontend.pulse()

    def get(self):

        observation = self._frontend.pulse()
        time_stamp = observation.get_time_stamp()
        ball_states = observation.get_observed_states()
        ball_position = [None] * 3
        ball_velocity = [None] * 3
        for dim in range(3):
            ball_position[dim] = ball_states.get(2 * dim).get()
            ball_velocity[dim] = ball_states.get(2 * dim + 1).get()

        return time_stamp, ball_position, ball_velocity

    def get_data(self, start_iteration):

        observations = self._frontend.get_observations_since(start_iteration)
        data = [_Data(obs) for obs in observations]

        return data
