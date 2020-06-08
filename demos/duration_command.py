import time
import fyplot
import o80
import o80_pam
import pam_interface


SEGMENT_ID = "o80_dummy_pam"


def run():

    frontend = o80_pam.FrontEnd(SEGMENT_ID)

    target_high = [20000]*4
    target_low = [10000]*4

    frontend.add_command(target_high,
                         target_low,
                         o80.Speed.per_second(4000),
                         o80.Mode.OVERWRITE)
     
    frontend.add_command(target_low,
                         target_high,
                         o80.Speed.per_second(4000),
                         o80.Mode.QUEUE)

    print("pulse!")
    frontend.pulse()

if __name__ == "__main__":
    run()
