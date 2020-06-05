import time
import fyplot
import o80
import o80_pam
import pam_interface


SEGMENT_ID = "o80_dummy_pam"
FREQUENCY = 1000
BURSTING_MODE = False
WINDOW = (1200,800)

def _get_plot(frontend):

    plt = fyplot.Plot(SEGMENT_ID,50,WINDOW)

    def get_pressure():
        obs = frontend.latest()
        pressure = obs.get_desired_states().get(0).get()
        return pressure
    
    def get_frequency():
        return frontend.latest().get_frequency()

    pressure_plot = ((get_pressure,(255,0,0)),)
    frequency_plot = ((get_frequency,(255,0,0)),)
    plt.add_subplot((0,25000),1000,pressure_plot)
    plt.add_subplot((0,FREQUENCY+100),1000,frequency_plot)
    
    return plt


def run():

    o80.clear_shared_memory(SEGMENT_ID)

    o80_pam.start_standalone(SEGMENT_ID,
                             FREQUENCY,
                             BURSTING_MODE,
                             pam_interface.DefaultConfiguration())

    
    frontend = o80_pam.FrontEnd(SEGMENT_ID)

    plot = _get_plot(frontend)
    plot.start()

    time.sleep(1)
    
    state = o80_pam.State()
    state.set(20000)
    frontend.add_command(0,state,o80.Duration_us.milliseconds(5000),o80.Mode.OVERWRITE)
    frontend.pulse()
    print("pulse !")
    
    try :
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    plot.stop()
    
    o80_pam.stop_standalone(SEGMENT_ID)

if __name__ == "__main__":
    run()



