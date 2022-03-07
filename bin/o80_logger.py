#!/usr/bin/env python3

import sys
import time
import signal_handler
import o80_pam
from lightargs import BrightArgs,FileExists


def _log(segment_id,file_path,frequency,duration):
    # for exit on ctrl+c
    signal_handler.init()
    time_start = time.time()
    # this context manager creates the file and
    # starts to log
    with o80_pam.Logger(segment_id,
                        file_path,
                        frequency):
        # monitoring for ctrl+c
        while not signal_handler.has_received_sigint():
            time.sleep(0.1)
            # or exiting when the provided duration passed
            if duration > 0:
                if time.time()-time_start>duration:
                    break
    

def _configure():
    config = BrightArgs("o80 PAM logger")
    # note: a backend is expected to be already running on this segment_id
    config.add_option("segment_id",
                      o80_pam.segment_ids.robot,
                      "o80 segment_id of the instance to log",
                      str)
    # the propose path does not exists, and contains the current date and time
    config.add_option("file_path",
                      o80_pam.FileManager().next(),
                      "absolute path to the log file (will overwrite if existing !)",
                      str)
    # all the observations generated between two iterations of the collection process
    # will be written, so (relatively) low frequency should be ok. 500Hz is likely to
    # be overkill, but that's fine
    config.add_option("frequency",
                      500.,
                      "frequency at which entries will be written in the file",
                      float)
    # to have the process automatically stopping
    config.add_option("duration",
                      -1,
                      "duration of record (in sec). if negative, infinite record time",
                      int)
    change_all=False
    finished = config.dialog(change_all,sys.argv[1:])
    if not finished:
        return None
    return config


def execute():
    config = _configure()
    if config is None:
        return
    _log(config.segment_id,
         config.file_path,
         config.frequency,
         config.duration)
    
if __name__ == "__main__":
    execute()
