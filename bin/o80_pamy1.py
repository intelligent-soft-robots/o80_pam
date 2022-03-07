#!/usr/bin/env python3

"""
starts o80 backend process for pamy1
"""

from o80_pam import run_robot

if __name__ == "__main__":

    real_robot = True
    config = run_robot.configure("pamy1")
    if config is not None:
        run_robot.run(config)
