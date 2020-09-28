import o80
import o80_pam

# convenience class 
# for sending pressure commands to a robot
class o80Pressures:

    def __init__(self,
                 segment_id,
                 period_ms=None):
        
        self._frontend = o80_pam.FrontEnd(segment_id)
        if period_ms:
            self._duration = o80.Duration_us.milliseconds(period_ms)
        else:
            self._duration = None


    def set(self,action):
        
        for dof,(ago_pressure,antago_pressure) in enumerate(action):
            if self._duration:
                self._frontend.add_command(dof,
                                            ago_pressure,antago_pressure,
                                            self._duration,
                                            o80.Mode.OVERWRITE)
            else:
                self._frontend.add_command(dof,
                                            ago_pressure,antago_pressure,
                                            o80.Mode.OVERWRITE)
        self._frontend.pulse()


    def read(self):

        obs = self._frontend.latest()
        pressures = obs.get_observed_pressures()
        pressures_ago = [pressures[dof][0] for dof in range(4)]
        pressures_antago = [pressures[dof][1] for dof in range(4)]

        robot_joints = obs.get_positions()
        robot_joint_velocities = obs.get_velocities()
        
        return pressures_ago,pressures_antago,robot_joints,robot_joint_velocities
