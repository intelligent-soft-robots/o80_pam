import o80
import o80_pam

# convenience class for sending mirroring
# (i.e. imposing joint positions and velocities)
# to a robot
class o80RobotMirroring:


    def __init__(self,segment_id):

        self._frontend = o80_pam.MirrorRobotFrontEnd(segment_id)
        self._state = o80.State2d(0,0)

        
    def burst(self,nb_iterations):

        self._frontend.burst(nb_iterations)
        

    def get(self):

        states = self._frontend.pulse().get_observed_states()
        positions = [states.get(dof).get(0) for dof in range(4)]
        velocities = [states.get(dof).get(1) for dof in range(4)]
        return positions,velocities
    
        
    def set(self,joint_positions,joint_velocities,
            duration_ms=None,nb_iterations=None,wait=False,burst=False):

        if duration_ms is not None:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None

        if nb_iterations is not None:
            iteration = o80.Iteration(nb_iterations,True,True)
        else:
            iteration = None
            
        for dof,(position,velocity) in enumerate(zip(joint_positions,
                                                     joint_velocities)):
            self._state.set(0,position)
            self._state.set(1,velocity)
            if duration:
                self._frontend.add_command(dof,self._state,duration,o80.Mode.OVERWRITE)
            elif iteration:
                self._frontend.add_command(dof,self._state,iteration,o80.Mode.OVERWRITE)
            else:
                self._frontend.add_command(dof,self._state,o80.Mode.OVERWRITE)

        if wait:
            self._frontend.pulse_and_wait()
        else:
            if burst:
                self._frontend.burst(1)
            else:
                self._frontend.pulse()
