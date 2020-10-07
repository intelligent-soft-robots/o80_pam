import o80
import o80_pam

# convenience class 
# for sending pressure commands to a robot
class o80Pressures:

    def __init__(self,
                 segment_id):
        self._frontend = o80_pam.FrontEnd(segment_id)


    def get_iteration(self):

        return self._frontend.pulse().get_iteration()

    
    def burst(self,nb_iterations):
        
        self._frontend.burst(nb_iterations)

        
    def set(self,action,duration_ms=None,wait=False,burst=False):

        if duration_ms:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None
            
        for dof,(ago_pressure,antago_pressure) in enumerate(action):
            if duration:
                self._frontend.add_command(dof,
                                           ago_pressure,antago_pressure,
                                           duration,
                                           o80.Mode.OVERWRITE)
            else:
                self._frontend.add_command(dof,
                                           ago_pressure,antago_pressure,
                                           o80.Mode.OVERWRITE)
        if wait:
            self._frontend.pulse_and_wait()
        else:
            if burst:
                self._frontend.burst(1)
            else:
                self._frontend.pulse()


    def read(self):

        obs = self._frontend.latest()
        pressures = obs.get_observed_pressures()
        pressures_ago = [pressures[dof][0] for dof in range(4)]
        pressures_antago = [pressures[dof][1] for dof in range(4)]

        robot_joints = obs.get_positions()
        robot_joint_velocities = obs.get_velocities()
        
        return pressures_ago,pressures_antago,robot_joints,robot_joint_velocities
