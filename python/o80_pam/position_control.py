import typing, copy
from typing import Sequence, Tuple
from dataclasses import dataclass
import math
import pam_interface


@dataclass
class PositionControllerStep:
    step: int
    q: float
    dq: float
    error: float
    d_error: float
    q_traj: float
    dq_traj: float
    control_p: float
    control_d: float
    control_i: float
    control: float
    p_ago: int
    p_antago: int
    

class PositionController:
    """
    Computes a trajectory of pressures suitable to reach
    a desired posture (i.e. desired angular positions of
    joints). To compute the pressures: a PID controller is used
    to compute a control signal. The control signal is used to set
    up an ratio of agonist/antagonist pressures to apply.

    Args:
        q_current: current joint positions (in radian)
        q_desired: target joint positions (in radian)
        dq_desired: target joint speed during motion (in radian per second)
        pam_interface_config: configuration of the PAM muscles
        kp,kd,ki : PID gains
        ndp: pressure level gain. The higher the value, the higher the pressures
        time_step: the next function is expected to be called with
                    a period of time_step (seconds)
        extra_steps: the trajectory is extended of some extra steps which
                     have the final position with a velocity of 0 as
                     desired state. Helps the system to stabilize.
    """

    def __init__(
        self,
        q_current: Sequence[float],
        q_desired: Sequence[float],
        dq_desired: Sequence[float],
        pam_interface_config: pam_interface.Configuration,
        kp: Sequence[float],
        kd: Sequence[float],
        ki: Sequence[float],
        ndp: Sequence[float],
        time_step: float,
        extra_steps: int = 100,
    ):

        self._q_current = q_current
        self._q_desired = q_desired
        self._dq_desired = dq_desired

        self._min_agos = pam_interface_config.min_pressures_ago
        self._max_agos = pam_interface_config.max_pressures_ago
        self._min_antagos = pam_interface_config.min_pressures_antago
        self._max_antagos = pam_interface_config.max_pressures_antago
        self._range_agos = [
            max_ - min_ for max_, min_ in zip(self._max_agos, self._min_agos)
        ]
        self._range_antagos = [
            max_ - min_ for max_, min_ in zip(self._max_antagos, self._min_antagos)
        ]

        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._ndp = ndp

        self._time_step = time_step

        q_error = [
            current - desired
            for desired, current in zip(self._q_current, self._q_desired)
        ]

        steps = [
            math.ceil(abs(error) / time_step / dq)
            for error, dq in zip(q_error, self._dq_desired)
        ]

        def _get_q_trajectory(nb_steps, current, desired):
            error = desired - current
            r = [(error / nb_steps) * step + current for step in range(nb_steps)]
            return r
            
        q_trajectories = [
            _get_q_trajectory(nb_steps, current, desired)
            for nb_steps, current, desired in zip(
                steps, self._q_current, self._q_desired
            )
        ]

        dq_trajectories = [
            [d_desired] * nb_steps
            for d_desired, nb_steps in zip(self._dq_desired, steps)
        ]

        def _align_sizes(arrays, fill_values):
            max_size = max([len(a) for a in arrays]) + extra_steps

            def _align_size(array, fill_value):
                array.extend([fill_value] * (max_size - len(array)))
                return array

            return list(map(_align_size, arrays, fill_values))

        self._q_trajectories = _align_sizes(q_trajectories, q_desired)
        self._dq_trajectories = _align_sizes(dq_trajectories, [0] * len(q_desired))
        self._error_sum = [0] * len(q_current)

        self._step = 0
        self._max_step = max(steps) + extra_steps

        self._introspect = [None]*len(steps)
        
    def introspection(self)->typing.Sequence[PositionControllerStep]:
        return copy.deepcopy(self._introspect)
        
    def get_time_step(self)->float:
        return self._time_step
        
    def _next(self, dof: int, q: float, qd: float, step: int) -> Tuple[float, float]:
        error = self._q_trajectories[dof][step] - q
        d_error = self._dq_trajectories[dof][step] -qd
        self._error_sum[dof] += error
        control_p = self._kp[dof] * error
        control_d = self._kd[dof] * d_error
        control_i = self._ki[dof] * self._error_sum[dof]
        control = control_p + control_d + control_i
        control = max(min(control, 1), -1)
        p_ago = self._min_agos[dof] + self._range_agos[dof] * (self._ndp[dof] - control)
        p_antago = self._min_antagos[dof] + self._range_antagos[dof] * (
            self._ndp[dof] + control
        )
        self._introspect[dof]=PositionControllerStep(
            step,q,qd,error,d_error,
            self._q_trajectories[dof][step],self._dq_trajectories[dof][step],
            control_p,control_d,control_i,control,
            int(p_ago),int(p_antago)
        )
        return int(p_ago), int(p_antago)

    def has_next(self) -> bool:
        """
        Returns:
            False if the trajectory is finished, True otherwise
        """
        return self._step < self._max_step

    def next(self, q, qd) -> Sequence[Tuple[float, float]]:
        """
        Returns:
             a list [(pressure ago, pressure antago),...] of pressures to
             apply at the next step in order to follow the computed trajectory
        """
        r = list(map(self._next, range(len(q)), q, qd, [self._step] * len(q)))
        self._step += 1
        return r


class PositionControllerFactory:
    """
    For instanciating instances of PositionController
    having the same configuration, but different starting
    and desired positions
    """
    
    def __init__(
            self,
            dq_desired: Sequence[float],
            pam_interface_config: pam_interface.Configuration,
            kp: Sequence[float],
            kd: Sequence[float],
            ki: Sequence[float],
            ndp: Sequence[float],
            time_step: float,
            extra_steps: int = 100,
    ):

        self.dq_desired = dq_desired
        self.pam_interface_config = pam_interface_config
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ndp = ndp
        self.time_step = time_step
        self.extra_steps = extra_steps

    def get(self,
            q_current: Sequence[float],
            q_desired: Sequence[float])->PositionController:
        return PositionController(q_current,
                                  q_desired,
                                  self.dq_desired,
                                  self.pam_interface_config,
                                  self.kp,
                                  self.kd,
                                  self.ki,
                                  self.ndp,
                                  self.time_step,
                                  self.extra_steps)
