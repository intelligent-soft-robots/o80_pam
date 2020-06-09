import time
import o80
import o80_pam
import pam_interface


SEGMENT_ID = "o80_dummy_pam"

def run():

    frontend = o80_pam.FrontEnd(SEGMENT_ID)

    target_low = [10000]*4
    target_high = [25000]*4

    duration = o80.Duration_us.seconds(1)
    
    frontend.add_command(target_low,target_high,duration,o80.Mode.OVERWRITE)
    frontend.add_command(target_high,target_low,duration,o80.Mode.QUEUE)

    frontend.pulse()

if __name__ == "__main__":
    run()



