import time
import fyplot
import o80
import o80_pam
import pam_interface


SEGMENT_ID = "o80_dummy_pam"
FREQUENCY = 200
BURSTING_MODE = False

def run():

    o80.clear_shared_memory(SEGMENT_ID)

    o80_pam.start_standalone(SEGMENT_ID,
                             FREQUENCY,
                             BURSTING_MODE,
                             pam_interface.DefaultConfiguration())

    
    try :
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    o80_pam.stop_standalone(SEGMENT_ID)

if __name__ == "__main__":
    run()



