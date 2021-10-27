import time
import math
import o80
import o80_pam


# default id when starting pam_robot executable
segment_id = "o80_pam_robot"
frontend = o80_pam.FrontEnd(segment_id)

# getting to 15000 one iteration at a time
for iteration in range(15000):
    target = [iteration] * 4
    frontend.add_command(target, [0] * 4, o80.Mode.OVERWRITE)
    obs = frontend.burst(1)
    new_iteration = obs.get_iteration()
    desired = obs.get_desired_pressures()
    print(new_iteration, target, desired)

# going back to 0 over 10000 iteration,
# played in one burst
current_iteration = frontend.latest().get_iteration()
frontend.add_command(
    [0] * 4, [0] * 4, o80.Iteration(current_iteration + 10000), o80.Mode.QUEUE
)
frontend.burst(10000)
