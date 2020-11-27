import time
import numpy as np
from .o80_pressures import o80Pressures
import o80_pam


class JointPositionControllerConfig:

    __slots__ = ("o80_pressures","ref_pressures","kp","kd","ki",
                 "min_agos","min_antagos","max_agos","max_antagos",
                 "mask","nb_dofs","iteration_duration_ms","target_error")

    def __init__(self,
                 o80_pressures:o80_pam.o80Pressures,
                 pam_interface_config):

        self.o80_pressures = o80_pressures
        
        self.min_agos = pam_interface_config.min_pressures_ago
        self.max_agos = pam_interface_config.max_pressures_ago 
        self.min_antagos = pam_interface_config.min_pressures_antago
        self.max_antagos = pam_interface_config.max_pressures_antago

        self.nb_dofs = len(self.min_agos)
        
        self.ref_pressures = [18000]*self.nb_dofs

        self.kp = [0.01]*self.nb_dofs
        self.kd = [0.005]*self.nb_dofs
        self.ki = [0.0]*self.nb_dofs
        
        self.mask = [True]*self.nb_dofs

        self.iteration_duration_ms = 2


class JointPositionController:


    def __init__(self,
                 config:JointPositionControllerConfig,
                 burst_mode):

        self._o80_pressures = config.o80_pressures
        self._ref_pressures = np.array(config.ref_pressures)
        self._kp = np.array(config.kp)
        self._kd = np.array(config.kd)
        self._ki = np.array(config.ki)
        self._iteration_duration_ms = config.iteration_duration_ms
        self._min_agos = np.array(config.min_agos)
        self._max_agos = np.array(config.max_agos)
        self._min_antagos = np.array(config.min_antagos)
        self._max_antagos = np.array(config.max_antagos)
        self._burst_mode = burst_mode
        self._nb_dofs = config.nb_dofs
        self._mask=np.array(config.mask)


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

       
    def go_to(self,
              q_desired,
              q_err=[0.05]*4,
              timeout_s=5):

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
        integrated_errors = np.zeros(self._nb_dofs)

        modes = p_agos > p_antagos
        ratios = np.where(modes,p_agos/p_antagos,p_antagos/p_agos)
        errors = q_desired-q
        
        while True:

            # error between current state and desired state
            errors = q_desired-q
            errors_vel = -q_vel
            
            integrated_errors += errors*self._iteration_duration_ms*0.001
            
            # if error is small for all dofs, exiting with success
            if all(abs(errors[self._mask])<q_error[self._mask]):
                yield True,None,None,None
                break

            # mode "true" : we use ratio p_agos/p_antago
            # mode "false" : we use ratio p_antagos/p_agos
            # also taking care of mode switch between 2 iterations
            new_modes = p_agos > p_antagos
            
            # change in ratios p_ago/p_antagos
            forces = self._kp*errors + self._kd*errors_vel + self._ki*integrated_errors
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
                return False,None,None,None

            # reading robot state
            p_agos,p_antagos,q,q_vel = self._o80_pressures.read()
            p_agos = np.array(p_agos)
            p_antagos = np.array(p_antagos)
            q = np.array(q)
            q_vel = np.array(q_vel)
            
            # yielding current joint angles (None: not terminated yet)
            yield None,q,q_vel,errors

        

        

