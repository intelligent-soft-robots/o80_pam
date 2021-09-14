import time
import numpy as np
import o80
import o80_pam
from .o80_pressures import o80Pressures


class _Controller:

    def __init__(self,desired,kp,kd,ki):
        self.desired = desired
        self.previous_p = None
        self.integral = 0
        self.kp = kp
        self.kd = kd
        self.ki = ki

    def control(self,p,p_vel,time_diff):
        
        # error
        error = self.desired-p
        
        # integrating
        self.integral += error*time_diff

        # force: by how much pressure we will change
        force = self.kp*error + self.kd*p_vel * self.ki*self.integral

        return force
        

class JointPositionController:


    def __init__(self,
                 pam_interface_config,
                 kp,kd,ki):

        self.min_agos = pam_interface_config.min_pressures_ago
        self.max_agos = pam_interface_config.max_pressures_ago 
        self.min_antagos = pam_interface_config.min_pressures_antago
        self.max_antagos = pam_interface_config.max_pressures_antago
        self.nb_dofs = len(self.min_agos)
        self.controllers = [None]*self.nb_dofs
        self.current_time = None
        self.kp = kp
        self.kd = kd
        self.ki = ki

    def reset(self):
        self.current_time = None
        self.controllers = [None]*self.nb_dofs

    def go_to(self,
              o80_pressures,
              desired_positions,
              current_time):

        p_agos,p_antagos,q,q_vel = o80_pressures.read()

        # computing time diff since previous iteration
        if self.current_time is None:
            self.current_time = current_time
        time_diff = current_time - self.current_time 
        self.current_time = current_time

        # we will return:
        desired_pressures = [None]*self.nb_dofs

        for dof in range(self.nb_dofs):

            # creating controller (if required)
            controller = self.controllers[dof]
            if controller is None:
                controller = _Controller(desired_positions[dof],
                                         self.kp[dof],self.kd[dof],self.ki[dof])
                self.controllers[dof]=controller
            elif controller.desired!=desired_positions[dof]:
                controller = _Controller(desired_positions[dof],
                                         self.kp[dof],self.kd[dof],self.ki[dof])
                self.controllers[dof]=controller
        
            # applying the controller, which returns a "force", i.e.
            # the new desired diff pressure between agonist and antagonist
            force = controller.control(q[dof],q_vel[dof],time_diff)

            force = int(force+0.5)
            p_ago = p_agos[dof] - force
            p_antago = p_antagos[dof] + force

            p_ago = min(max(self.min_agos[dof],p_ago),self.max_agos[dof])
            p_antago = min(max(self.min_antagos[dof],p_antago),self.max_antagos[dof])

            desired_pressures[dof]=(p_ago,p_antago)

        return desired_pressures

