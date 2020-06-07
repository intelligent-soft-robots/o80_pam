import sys
import math
import threading
import time
import fyplot
import o80
import o80_pam
import pam_interface
from functools import partial

DEFAULT_FREQUENCY = 2000 # to plot the frequency on the right scale
WINDOW = (1200,800) # plot window size (in pixels)

# creates the fyplot instance with 5 plots:
# - 1 per dof, with the agonist pressure (red),
#   the antagonist pressure (green) and the position (blue).
#   the position is cast from [-pi,pi] to [min_pressure,max_pressure].
# - a plot for frequency
class Data:

    # segment_id : str, for the frontend
    # config : instance of pam_interface.Configuration
    # window : size of the plotting window (pixels)
    # frequency : used to scale the frequency plot
    def __init__(self,segment_id,
                 config,
                 frequency,
                 window=WINDOW):

        # o80 frontend
        self._frontend = o80_pam.FrontEnd(segment_id)

        # data holder. "False" means the value has not been plotted
        # yet
        self._desired = [[None,None,False] for dof in range(4)]
        self._observed = [[None,None,False] for dof in range(4)]
        self._positions = [[None,None] for dof in range(4)]
        self._frequency = [None,False]

        # only data corresponding to a new iteration will be plotted
        self._iteration = None

        # avoiding concurrent read and write of data holder
        self._lock = threading.Lock()

        # a thread will get the data using the frontend
        # (and the fyplot instance will read the data)
        self._running = False
        self._thread = None

        # dynamic plot
        self._plot = fyplot.Plot(segment_id,50,
                                 window)
        
        # min and max pressure according to the configuration
        self._min_pressure  = min(config.min_pressures_ago + config.min_pressures_antago)
        self._max_pressure  = max(config.max_pressures_ago + config.max_pressures_antago)

        # 1 plot per dof (desired and observed pressure, position/angle)
        for dof in range(4):
            dof_plot = ( (partial(self.get_desired,dof,0),(200,0,0)),
                         (partial(self.get_desired,dof,1),(0,200,0)),
                         (partial(self.get_observed,dof,0),(0,255,0)),
                         (partial(self.get_observed,dof,0),(0,255,0)),
                         (partial(self.get_position,dof),(0,255,0)) )

            self._plot.add_subplot((self._min_pressure,self._max_pressure),
                                   1000,dof_plot)

        # 1 plot for the frequency
        freq_plot = ((self.get_frequency,(255,0,0)),)
        max_frequency = frequency + frequency/10.0
        self._plot.add_subplot((0,max_frequency),1000,
                               freq_plot)
        
    def run(self):
        # loops reading the data using the frontend
        self._running = True
        while self._running:
            self._update()
            time.sleep(0.0005)

    def start(self):
        # start the thread for polling the data
        # using the frontend, and start plotting
        self._thread = threading.Thread(target=self.run)
        self._thread.start()
        self._plot.start()

    def stop(self):
        # stop the data polling thread
        # and the plotting
        self._running = False
        self._thread.join()
        self._plot.stop()
            
    def _update(self):
        # read the required data from the
        # frontend
        obs = self._frontend.latest()
        iteration = obs.get_iteration()
        if iteration == self._iteration:
            return
        self._iteration = iteration
        with self._lock:
            desired = obs.get_desired_states()
            observed = obs.get_observed_states()
            positions = obs.get_positions()
            for dof in range(4):
                ago = 2*dof
                antago = ago+1
                self._desired[dof]=[desired.get(ago).get(),
                                    desired.get(antago).get(),
                                    True]
                self._observed[dof]=[observed.get(ago).get(),
                                     observed.get(antago).get(),
                                     True]
                self._positions[dof]=[positions[dof],True]
            self._frequency = [obs.get_frequency(),True]
            print(self._desired)
            print(self._observed)
            print(self._frequency)
            print(self._positions)
            print()
            
    def get_desired(self,dof,ago):
        with self._lock:
            if self._desired[dof][2]:
                # to avoid the same value
                # to be plotted twice
                self._desired[dof][2]=False
                return self._desired[dof][ago]
            return None

    def get_observed(self,dof,ago):
        with self._lock:
            if self._observed[dof][2]:
                self._observed[dof][2]=False
                return self._observed[ago][0]
            return None

    def get_position(self,dof):
        with self._lock:
            if self._positions[dof][1]:
                self._positions[dof][1]=False
                # position is an angle [-pi,pi],
                # casting it to a value between min pressure and
                # max pressure, to be on scale
                return self._normalize_position(self._positions[dof][0])
            return None
        
    def _normalize_position(self,angle):
        v = angle / 2.0 * math.pi
        v = v * (self._max_pressure - self._min_pressure)
        return self._min_pressure + v
        
    def get_frequency(self):
        with self._lock:
            if self._frequency[1]:
                self._frequency[1]=False
                return self._frequency[0]
            return None


# parse the arguments are return
# segment_id (str), frequency (int), path to config file (str)
def parse(args):

    if len(args)<1:
        raise Exception("please specify a segment id")
    segment_id = str(args[0])

    if len(args)==1:
        return segment_id,None,None

    def p(arg,d):
        try:
            freq = int(args[1])
            d["freq"]=freq
        except:
            config = str(args[1])
            d["config"]=config

    d = {"freq":None,"config":None}
    for arg in args[1:]:
        p(arg,d)

    return segment_id,d["freq"],p["config"]

# a config file path was passed as argument,
# creating the corresponding pam_interface.Configuration
# instance
def read_config(config_path):
    if not os.path.isfile(config_path):
        raise Exception("failed to find",config_path)
    try :
        config = pam_interface.JsonConfiguration(config_path)
        return config
    except Exception as e:
        error = "failed to parse json configuration file "+config_path
        raise Exception(error,":",str(e))

def print_usage():
    print("\n")
    print("\tusage: o80_pam_plotting segment_id backend_frequency config_path\n")
    print("\t\tsegment_id: segment_id of the backend")
    print("\t\tbackend_frequency: frequency of the backend (optional, default:2000)")
    print("\t\tconfig_path: json file "+
          "(optional, default: pam_interface.DefaultConfiguration)")
    print("\n")
    
def execute(args):

    try:
    
        # parsing the arguments
        segment_id,freq,config = parse(args)

        # config is used for min and max pressures, for correct
        # scaling
        if config is not None:
            # a (json) config file path was provided,
            # generating config instance
            config = read_config(config)
        else:
            # using default config
            config = pam_interface.DefaultConfiguration()

        # frequency is also used for correct scaling
        if freq is None:
            freq = DEFAULT_FREQUENCY

    except Exception as e:
        print_usage()
        print("\tERROR:",e,"\n")
        return
        
    data = Data(segment_id,config,freq)
    data.start()

    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break

    data.stop()
        
if __name__ == "__main__":

    execute(sys.argv[1:])
    
