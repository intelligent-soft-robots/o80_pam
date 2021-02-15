import o80
import o80_pam
import context


class _Data:
    def __init__(self,observation):
        ball_states = observation.get_observed_states()
        self.ball_position = [None]*3
        self.ball_velocity = [None]*3
        for dim in range(3):
            self.ball_position[dim]=ball_states.get(2*dim).get()
            self.ball_velocity[dim]=ball_states.get(2*dim+1).get()


# convenience class for shooting virtual balls
# via o80, playing pre-recorded trajectories (hosted in context package)
class o80Ball:

    
    def __init__(self,segment_id):

        self._frontend = o80_pam.MirrorFreeJointFrontEnd(segment_id)

        
    def burst(self,nb_iterations):

        self._frontend.burst(nb_iterations)


    def get_iteration(self):

        return self._frontend.pulse().get_iteration()
        
    def play_trajectory(self,trajectory_points):

        # sending the full ball trajectory 
        # duration of 10ms : sampling rate of the trajectory
        duration = o80.Duration_us.milliseconds(10)
        for traj_point in trajectory_points:
            self._frontend.add_command(traj_point.position,
                                       traj_point.velocity,
                                       duration,
                                       o80.Mode.QUEUE)
        self._frontend.pulse()

        
    def set(self,position,velocity,duration_ms=None,wait=False):

        if duration_ms is not None:
            duration = o80.Duration_us.milliseconds(duration_ms)
        else:
            duration = None

        if duration is None:
            self._frontend.add_command(position,
                                       velocity,
                                       o80.Mode.OVERWRITE)
        else:
            self._frontend.add_command(position,
                                       velocity,
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


    def get_data(self,start_iteration):

        observations = self._frontend.get_observations_since(start_iteration)
        data = [_Data(obs) for obs in observations]

        return data
