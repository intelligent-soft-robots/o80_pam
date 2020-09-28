import o80
import o80_pam

# convenience class for sending mirroring
# (i.e. imposing joint positions and velocities)
# to a robot
class o80RobotMirroring:


    def __init__(self,segment_id):

        self._frontend = o80_pam.MirrorRobotFrontEnd(segment_id)
        self._state = o80.State2d(0,0)
        
    def set(self,joint_positions,joint_velocities):

        for dof,(position,velocity) in enumerate(zip(joint_positions,
                                                     joint_velocities)):
            self._state.set(0,position)
            self._state.set(1,velocity)
            self._frontend.add_command(dof,self._state,o80.Mode.OVERWRITE)
            
        self._frontend.pulse()
