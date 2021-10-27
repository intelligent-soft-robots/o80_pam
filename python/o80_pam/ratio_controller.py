import time
import numpy as np
from .o80_pressures import o80Pressures


class JointPositionController:
    def __init__(
        self,
        o80_pressures: o80Pressures,
        ref_pressures,
        kp,
        iteration_duration_ms,
        min_agos,
        min_antagos,
        max_agos,
        max_antagos,
        burst_mode,
        nb_dofs,
    ):

        self._o80_pressures = o80_pressures
        self._ref_pressures = np.array(ref_pressures)
        self._kp = np.array(kp)
        self._iteration_duration_ms = iteration_duration_ms
        self._min_agos = np.array(min_agos)
        self._max_agos = np.array(max_agos)
        self._min_antagos = np.array(min_antagos)
        self._max_antagos = np.array(max_antagos)
        self._burst_mode = burst_mode
        self._nb_dofs = nb_dofs

    def _get_pressures(self, ratios):
        p2 = 2.0 * self._ref_pressures / (ratios + 1)
        p1 = 2.0 * self._ref_pressures - p2
        p2 = np.array(int(p2 + 0.5), dtype=np.int32)
        p1 = np.array(int(p1 + 0.5), dtype=np.int32)
        return p1, p2

    def go_to(self, q_desired, q_err, timeout_s):

        q_desired = np.array(q_desired)
        q_error = np.array(q_err)

        total_duration = 0
        p_agos, p_antagos, q, _ = self._o80_pressures.read()
        p_agos = np.array(p_agos)
        p_antagos = np.array(p_antagos)
        q = np.array(q)

        ratios = p_agos / p_antagos

        while True:

            # error between current position and desired position
            error = q_desired - q

            # if error is small for all dofs, exiting with success
            if all(abs(error) < q_error):
                yield True, _
                break

            # change in ratios
            ratio_update = abs(kp * error)

            # new ratios
            ratios += ratio_update

            # corresponding pressures, i.e.
            # p1+p2 = 2*ref pressure
            # p1 / p2 = ratio
            if error > 0:
                p_agos, p_antagos = _get_pressures(ratios)
            else:
                p_antagos, p_agos = _get_pressures(ratios)

            # packing pressures to action
            action = [(p_agos[dof], p_antagos[dof]) for dof in range(self._nb_dofs)]

            # send action to robot
            if self._burst_mode:
                self._o80_pressures.set(
                    action, duration_ms=None, wait=False, burst=True
                )

            else:
                self._o80_pressures.set(
                    action,
                    duration_ms=self._iteration_duration_ms,
                    wait=True,
                    burst=False,
                )

            # monitoring timeout
            total_duration += self._iteration_duration_ms
            if (total_duration * 0.001) > timeout_s:
                # False: failed to reach position (exiting because of timeout)
                return False, _

            # reading robot state
            p_agos, p_antagos, q, _ = self._o80_pressures.read()

            # yielding current joint angles (None: not terminated yet)
            yield None, q
