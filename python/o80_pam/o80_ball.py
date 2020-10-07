import o80
import o80_pam
import context

# convenience class for shooting virtual balls
# via o80, playing pre-recorded trajectories (hosted in context package)
class o80Ball:

    
    def __init__(self,segment_id):

        self._frontend = o80_pam.MirrorFreeJointFrontEnd(segment_id)

        
    def burst(self,nb_iterations):

        self._frontend.burst(nb_iterations)

        
    def play_trajectory(self,trajectory_points):

        # sending the full ball trajectory 
        # duration of 10ms : sampling rate of the trajectory
        duration = o80.Duration_us.milliseconds(10)
        for traj_point in trajectory_points:
            # looping over x,y,z
            for dim in range(3):
                # setting position for dimension (x, y or z)
                self._frontend.add_command(2*dim,
                                           o80.State1d(traj_point.position[dim]),
                                           duration,
                                           o80.Mode.QUEUE)
                # setting velocity for dimension (x, y or z)
                self._frontend.add_command(2*dim+1,
                                           o80.State1d(traj_point.velocity[dim]),
                                           duration,
                                           o80.Mode.QUEUE)
        self._frontend.pulse()

        
    def set(self,position,velocity,duration_ms=None,wait=False):

        if duration_ms is not None:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None

        if duration is None:
            for dim in range(3):
                # setting position for dimension (x, y or z)
                self._frontend.add_command(2*dim,
                                           o80.State1d(position[dim]),
                                           o80.Mode.OVERWRITE)
                # setting velocity for dimension (x, y or z)
                self._frontend.add_command(2*dim+1,
                                           o80.State1d(velocity[dim]),
                                           o80.Mode.OVERWRITE)
        else:
            for dim in range(3):
                # setting position for dimension (x, y or z)
                self._frontend.add_command(2*dim,
                                           o80.State1d(position[dim]),
                                           duration,
                                           o80.Mode.OVERWRITE)
                # setting velocity for dimension (x, y or z)
                self._frontend.add_command(2*dim+1,
                                           o80.State1d(velocity[dim]),
                                           duration,
                                           o80.Mode.OVERWRITE)
        if wait:
            self._frontend.pulse_and_wait()
        else:
            self._frontend.pulse()

            
    def get(self):

        ball_states = self._frontend.pulse().get_observed_states()
        ball_position = [None]*3
        ball_velocity = [None]*3
        for dim in range(3):
            ball_position[dim]=ball_states.get(2*dim).get()
            ball_velocity[dim]=ball_states.get(2*dim+1).get()
            
        return ball_position,ball_velocity
