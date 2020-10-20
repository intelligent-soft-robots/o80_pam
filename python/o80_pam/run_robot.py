import logging
import sys
import os
import time
import o80
import o80_pam
import pam_interface
from lightargs import BrightArgs,Set,Range,Positive,FileExists

# code used for starting o80 pam
# in the executable o80_pam/o80_real and
# o80_pam/o80_dummy 


def run(config):

    segment_id = config.segment_id
    frequency = config.frequency
    bursting_mode = config.bursting_mode
    pam_config = pam_interface.JsonConfiguration(config.pam_config_file)
    robot = config.robot

    # starting from a clean state
    o80.clear_shared_memory(segment_id)

    # starting the standalone (hosts the driver to pam
    # and the o80 backend)
    if robot=="real":
        o80_pam.real_start_standalone(segment_id,
                                      frequency,
                                      bursting_mode,
                                      pam_config)
    else:
        o80_pam.dummy_start_standalone(segment_id,
                                       frequency,
                                       bursting_mode,
                                       pam_config)
        
    # setting desired pressures to config minimal pressures
    frontend = o80_pam.FrontEnd(segment_id)
    for dof in range(config.nb_dofs):
        min_ago = pam_config.min_pressure(dof,pam_interface.sign.agonist)
        min_antago = pam_config.min_pressure(dof,pam_interface.sign.antagonist)
        frontend.add_command(dof,min_ago,min_antago,o80.Mode.OVERWRITE)
    frontend.pulse()
    del frontend

    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("\n\nException: {}\n".format(e))
    finally:
        # setting desired pressures to config minimal pressures
        # upon exit
        frontend = o80_pam.FrontEnd(segment_id)
        for dof in range(config.nb_dofs):
            min_ago = pam_config.min_pressure(dof,pam_interface.sign.agonist)
            min_antago = pam_config.min_pressure(dof,pam_interface.sign.antagonist)
            frontend.add_command(dof,min_ago,min_antago,
                                 o80.Speed.per_second(2000),o80.Mode.OVERWRITE)
        frontend.pulse_and_wait()
        del frontend
        o80_pam.stop_standalone(segment_id)

        
def configure(real_robot):
    if real_robot:
        robot = "real"
    else:
        robot = "dummy"
    config = BrightArgs("o80 PAM {} robot".format(robot))
    setattr(config,"robot",robot)
    config.add_option("segment_id",
                      o80_pam.segment_ids.robot,
                      "o80 backend segment_id ",
                      str)
    config.add_operation("bursting_mode",
                         "will the backend be started in bursting mode ?")
    config.add_option("frequency",
                      500,
                      "o80 backend frequency (non bursting mode only)",
                      int,
                      integrity_checks=[Range(1,3000),Positive()])
    config.add_option("nb_dofs",
                      4,
                      "number of degrees of freedom of the robots",
                      int,
                      integrity_checks=[Positive()])
    config.add_option("pam_config_file",
                      pam_interface.DefaultConfiguration.get_path(),
                      "pam configuration file",
                      str,
                      integrity_checks= [FileExists()])
    change_all = False
    config.dialog(change_all)
    return config
