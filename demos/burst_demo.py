import time
import math
import o80
import o80_pam


# default id when starting pam_robot executable
segment_id = "o80_pam"
frontend = o80_pam.FrontEnd(segment_id)

time.sleep(1)

current_iteration = frontend.latest().get_iteration()
target = 1000
frontend.add_command([target]*4,
                     [target]*4,
                     o80.Iteration(current_iteration+2),
                     o80.Mode.OVERWRITE)
obs = frontend.burst(2)
new_iteration = obs.get_iteration()
desired = obs.get_desired_pressures()
print("start iter:",current_iteration,"end iteration:",new_iteration,
      "target state:",target,"final desired state:",desired)



#time.sleep(2)

# going to 15000 for all muscles, 1 iteration at a time
def comment():
    for iteration in range(15000):
        current_iteration = frontend.latest().get_iteration()
        target = [iteration]*4
        frontend.add_command(target,
                             target,
                             o80.Iteration(current_iteration+1,False),
                             o80.Mode.OVERWRITE)
        obs = frontend.burst(1)
        new_iteration = obs.get_iteration()
        desired = obs.get_desired_pressures()
        print(current_iteration,new_iteration,target,desired)



