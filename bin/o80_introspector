#!/usr/bin/env python3

import sys
import time
import o80_pam
import signal_handler
from lightargs import BrightArgs,Set,Range,Positive

def _run(segment_id):
    o80_pam.Introspector.start(segment_id)
    signal_handler.init() # for detecting ctrl+c
    try:
        while not signal_handler.has_received_sigint():
            time.sleep(0.01)
    except (KeyboardInterrupt,SystemExit):
        o80_example.Introspector.stop()

def _configure():
    config = BrightArgs("o80 PAM introspection")
    config.add_option("segment_id",
                      o80_pam.segment_ids.robot,
                      "o80 segment_id of the instance of o80 pam to display",
                      str)
    change_all=False
    finished  = config.dialog(change_all,sys.argv[1:])
    if not finished:
        return None
    return config

def _execute():
    config = _configure()
    if config is None:
        return
    segment_id = config.segment_id
    _run(segment_id)
    
if __name__ == "__main__":
    _execute()
