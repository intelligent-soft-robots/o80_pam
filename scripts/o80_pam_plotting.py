import threading
import time
import fyplot
import o80
import o80_pam
import pam_interface



class Data:

    def __init__(self,segment_id,
                 config,
                 window=(1200,800),
                 max_frequency = 2000)
        self._frontend = o80_pam.FrontEnd(segment_id)
        self._desired = [(None,None,False) for dof in range(4)]
        self._observed = [(None,None,False) for dof in range(4)]
        self._frequency = (None,False)
        self._previous_iteration = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._plot = fyplot.Plot(segment_id,50,
                                 window)
        min_pressure  = min(config.min_pressures_ago + config.min_pressures_antago)
        max_pressure  = max(config.max_pressures_ago + config.max_pressures_antago)
        for dof in range(4):
            dof_plot = ( (partial(self.get_desired,dof,0),(200,0,0)),
                         (partial(self.get_desired,dof,1),(0,200,0)),
                         (partial(self.get_observed,dof,0),(0,255,0)),
                         (partial(self.get_observed,dof,0),(0,255,0)) )
            self._plot.add_subplot((min_frequency,max_frequency),
                                   1000,dof_plot)
        freq_plot = ((self.frequency,(255,0,0)))
        self._plot.add_subplot((0,max_frequency),1000,
                               freq_plot)
        
        
    def run(self):
        self._running = True
        while self._running:
            self._update()
            time.sleep(0.0005)

    def start(self):
        self._thread = threading.Thread(self.run)
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()

    
    
            
    def _update():
        obs = self._frontend.latest()
        iteration = obs.get_iteration()
        if iteration == self._iteration:
            return
        with self._lock:
            desired = obs.get_desired_states()
            observed = obs.get_observed_states()
            for dof in range(4):
                ago = 2*dof
                antago = ago+1
                self._desired[dof]=(desired.get(ago).get(),
                                    desired.get(antago).get()
                                    ,True)
                self._observed[dof]=(observed.get(ago).get(),
                                     observed.get(antago).get(),
                                     True)
            self._frequency = (obs.get_frequency(),True)

    def get_desired(self,dof,ago):
        with self._lock:
            if self._desired[dof][2]:
                return self._desired[dof][ago]
            return None

    def get_observed(self,dof,ago):
        with self._lock:
            if self._observed[dof][2]:
                return self._observed[ago][0]
            return None

    def get_frequency(self):
        with self._lock:
            if self._frequency[1]:
                return self._frequency[0]
            return None
