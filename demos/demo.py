import time
import math
import o80
import o80_pam


# default id when starting pam_robot executable
segment_id = "o80_pam_robot"
frontend = o80_pam.FrontEnd(segment_id)


# speed command
# on individual musles
print("speed command")
target_pressures = 15000
for actuator in range(8):
    frontend.add_command(
        actuator,
        target_pressures + 1000 * actuator,
        o80.Speed.per_second(4000),  # 4000 unit of pressure per second
        o80.Mode.QUEUE,
    )
frontend.pulse_and_wait()

# duration command on
# each degree of freedom
print("duration command")
ago_pressure = 18000
antago_pressure = 12000
for dof in range(4):
    frontend.add_command(
        dof, ago_pressure, antago_pressure, o80.Duration_us.seconds(3), o80.Mode.QUEUE
    )
frontend.pulse_and_wait()

# duration command over all muscles:
# going back to 15000 in 2 seconds
print("duration command")
frontend.add_command(
    [15000] * 4, [15000] * 4, o80.Duration_us.seconds(2), o80.Mode.QUEUE
)
frontend.pulse_and_wait()

# explicit trajectory specifying
# target pressures for steps of 5000 iterations.
# (iterations_per_step*500)
print("explicit trajectory")
pressure = 15000
v = 0.0
increment = 0.025
amplitude = 3000
first_command = True
current_iteration = 0
iterations_per_steps = 10
for _ in range(500):
    v += increment
    ago = pressure + int(amplitude * math.sin(v))
    antago = pressure - int(amplitude * math.sin(v))
    current_iteration += iterations_per_steps
    for dof in range(4):
        frontend.add_command(
            dof,
            ago,
            antago,
            o80.Iteration(current_iteration, True, first_command),
            o80.Mode.QUEUE,
        )
        first_command = False
frontend.pulse_and_wait()

# sending a command to 1st dof
duration = 5
frontend.add_command(0, 12000, 18000, o80.Duration_us.seconds(duration), o80.Mode.QUEUE)
frontend.pulse()

# having the second 2nd dof mirroring the 1st
print("2nd dof mirroring the 1st")
time_start = time.time()
while time.time() - time_start < duration:
    # getting pressures of the dof at index 0
    observation = frontend.latest()
    observed_pressures = observation.get_observed_pressures()
    p_ago = observed_pressures[0][0]
    p_antago = observed_pressures[0][1]
    # sending direct command for mirroring
    frontend.add_command(1, p_ago, p_antago, o80.Mode.OVERWRITE)
    frontend.pulse()
    # running at around 2000Hz
    time.sleep(0.0005)

# going back to 0
print("end: going back to 0")
duration = 2
frontend.add_command(
    [0] * 4, [0] * 4, o80.Duration_us.seconds(duration), o80.Mode.OVERWRITE
)
frontend.pulse_and_wait()
