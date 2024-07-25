import typing
import o80
import o80_pam
import context
from typing import Optional


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
    def __init__(
        self,
        segment_id,
        frontend=None,
        o80_backend_period: Optional[float] = None,
    ):
        if frontend is None:
            self._frontend = o80_pam.MirrorFreeJointFrontEnd(segment_id)
        else:
            self._frontend = frontend
        self._o80_backend_period = o80_backend_period

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

    def iterate_trajectory(
        self,
        trajectory_iterator: typing.Generator[
            context.ball_trajectories.DurationPoint, None, None
        ],
        overwrite=False,
    ):
        if overwrite:
            mode = o80.Mode.OVERWRITE
        else:
            mode = o80.Mode.QUEUE

        if self._o80_backend_period is None:
            for duration, state in trajectory_iterator:
                self._frontend.add_command(
                    state.get_position(),
                    state.get_velocity(),
                    o80.Duration_us.microseconds(duration),
                    mode,
                )
                mode = o80.Mode.QUEUE
        else:
            # what this does:

            # cast the "duration" trajectory to a "iteration" trajectory.
            # i.e. compute for each backend iteration the desired
            # positions and velocities of the ball, performing
            # iterpolation between the "duration" trajectory points.

            # x: time
            # Y: position or velocity vector
            # subscripts A or B: points of the duration
            #   trajectory
            # subscripts 1 or 2: points aligned with
            #   a backend iteration
            # subscripts p and v: position / velocity

            def _interpolate(x1, Y1, xb, Yb, period):
                nb_iterations = int((xb - x1) / period)
                x2 = x1 + nb_iterations * period
                Y2 = [
                    y1 + ((x2 - x1) / (xb - x1) * (yb - y1))
                    for y1, yb in zip(Y1, Yb)
                ]
                return x2, Y2, nb_iterations

            iteration = self._frontend.latest().get_iteration()
            xb = 0
            x1, Yp1, Yv1 = None, None, None
            for duration, state in trajectory_iterator:
                xb += duration * 1e-6
                YpB = state.get_position()
                YvB = state.get_velocity()
                if x1 is None:
                    x1, Yp1, Yv1 = xb, YpB, YvB
                    continue
                x2, Yp2, nb_iterations = _interpolate(
                    x1, Yp1, xb, YpB, self._o80_backend_period
                )
                x2, Yv2, _ = _interpolate(
                    x1, Yv1, xb, YvB, self._o80_backend_period
                )
                iteration += nb_iterations
                self._frontend.add_command(
                    Yp2, Yv2, o80.Iteration(iteration), mode
                )
                x1, Yp1, Yv1 = x2, Yp2, Yv2

        self._frontend.pulse()

    def play_trajectory(
        self,
        trajectory: context.ball_trajectories.StampedTrajectory,
        overwrite=False,
    ):
        iterator = context.BallTrajectories.iterate(trajectory)
        self.iterate_trajectory(iterator, overwrite=overwrite)

    def set(self, position, velocity, duration_ms=None, wait=False):
        if duration_ms is not None:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None

        if duration is None:
            self._frontend.add_command(position, velocity, o80.Mode.OVERWRITE)
        else:
            self._frontend.add_command(
                position, velocity, duration, o80.Mode.OVERWRITE
            )
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
