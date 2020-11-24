import time
import numpy as np
from .o80_pressures import o80Pressures


class JointPositionController:


    def __init__(self,
                 o80_pressures:o80Pressures,
                 ref_pressures,
                 kp,kd,
                 iteration_duration_ms,
                 min_agos,min_antagos,
                 max_agos,max_antagos,
                 burst_mode,
                 nb_dofs,
                 mask=[True]*4):
        
        self._o80_pressures = o80_pressures
        self._ref_pressures = np.array(ref_pressures)
        self._kp = np.array(kp)
        self._kd = np.array(kd)
        self._iteration_duration_ms = iteration_duration_ms
        self._min_agos = np.array(min_agos)
        self._max_agos = np.array(max_agos)
        self._min_antagos = np.array(min_antagos)
        self._max_antagos = np.array(max_antagos)
        self._burst_mode = burst_mode
        self._nb_dofs = nb_dofs
        self._mask=np.array(mask)


    def _get_pressures(self,modes,ratios):
        def _solve(ref,ratio):
            p2 = 2.*ref / (ratio+1.)
            a = ref - p2
            p1 = ratio*p2
            return p1,p2
        def _get_pressure(arg):
            ref,mode,ratio = arg
            if mode:
                p1,p2 = _solve(ref,ratio)
            else:
                p2,p1 = _solve(ref,ratio)
            return p1,p2
        pressures = list(map(_get_pressure,zip(self._ref_pressures,
                                                modes,ratios)))
        p_agos = list(map(lambda x:x[0],pressures))
        p_antagos = list(map(lambda x:x[1],pressures))
        return p_agos,p_antagos

       
    def go_to(self,q_desired,q_err,timeout_s):

        q_desired = np.array(q_desired)
        q_error = np.array(q_err)
        
        total_duration = 0
        p_agos,p_antagos,q,q_vel = self._o80_pressures.read()
        p_agos_init = np.array(p_agos)
        p_antagos_init = np.array(p_antagos)
        p_agos = np.array(p_agos)
        p_antagos = np.array(p_antagos)
        q = np.array(q)
        q_vel = np.array(q_vel)

        modes = p_agos > p_antagos
        ratios = np.where(modes,p_agos/p_antagos,p_antagos/p_agos)
        errors = q_desired-q
        
        while True:

            # error between current state and desired state
            errors = q_desired-q
            errors_vel = -q_vel
            
            # if error is small for all dofs, exiting with success
            if all(abs(errors[self._mask])<q_error[self._mask]):
                yield True,None
                break

            # mode "true" : we use ratio p_agos/p_antago
            # mode "false" : we use ratio p_antagos/p_agos
            # also taking care of mode switch between 2 iterations
            new_modes = p_agos > p_antagos
            
            # change in ratios p_ago/p_antagos
            forces = self._kp*errors + self._kd*errors_vel
            ratio_update = np.where(modes,-forces,+forces)

            # new ratios
            ratios += ratio_update

            # if a ratio gets below 1, mode needs to be switched
            new_modes = np.where(ratios<1.,~modes,modes)
            ratios = np.maximum(1,ratios)
            ratios = np.where(modes==new_modes,ratios,1.0/ratios)
            modes = new_modes

            # corresponding pressures, i.e.
            # p1 = ref_pressure + a
            # p2 = ref_pressure - a
            # p1/p2 = ratio
            p_agos,p_antagos = self._get_pressures(modes,ratios)
            
            # capping the pressures
            p_agos = np.minimum(self._max_agos,np.maximum(self._min_agos,p_agos))
            p_antagos = np.minimum(self._max_agos,np.maximum(self._min_antagos,p_antagos))
            
            # applying mask (i.e. some joint should not move)
            p_agos = np.where(self._mask,p_agos,p_agos_init)
            p_antagos = np.where(self._mask,p_antagos,p_antagos_init)
            
            # packing pressures to action (and converting them to int)
            action = [(int(p_agos[dof]+0.5),int(p_antagos[dof]+0.5))
                      for dof in range(self._nb_dofs)]
     
            # send action to robot
            if self._burst_mode:
                self._o80_pressures.set(action,duration_ms=None,wait=False,
                                        burst=True)

            else:
                self._o80_pressures.set(action,duration_ms=self._iteration_duration_ms,
                                        wait=True,burst=False)

            # monitoring timeout
            total_duration += self._iteration_duration_ms
            if (total_duration*0.001) > timeout_s:
                # False: failed to reach position (exiting because of timeout)
                return False,None

            # reading robot state
            p_agos,p_antagos,q,q_vel = self._o80_pressures.read()
            p_agos = np.array(p_agos)
            p_antagos = np.array(p_antagos)
            q = np.array(q)
            q_vel = np.array(q_vel)
            
            # yielding current joint angles (None: not terminated yet)
            yield None,q

        

        

