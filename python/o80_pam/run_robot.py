import logging
import sys
import os
import time
import o80
import o80_pam
import pam_interface
import signal_handler
from lightargs import BrightArgs,Set,Range,Positive,FileExists

# code used for starting o80 pam
# in the executable o80_pam/o80_real and
# o80_pam/o80_dummy 

def _set_min_pressures(config,pam_config):

    frontend = o80_pam.FrontEnd(config.segment_id)
    for dof in range(config.nb_dofs):
        min_ago = pam_config.min_pressure(dof,pam_interface.sign.agonist)
        min_antago = pam_config.min_pressure(dof,pam_interface.sign.antagonist)
        frontend.add_command(dof,min_ago,min_antago,o80.Mode.OVERWRITE)
    frontend.pulse()
    del frontend

def run(config):

    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format="[o80 PAM segment_id: {} |  %(levelname)s %(asctime)s] %(message)s".format(config.segment_id),
        level=logging.DEBUG,
        handlers=[log_handler]
    )

    logging.info("read configuration") 
    segment_id = config.segment_id
    frequency = config.frequency
    bursting_mode = config.bursting_mode
    robot = config.robot
    pam_config = pam_interface.JsonConfiguration(config.pam_config_file)

    logging.info("cleaning shared memory") 
    o80.clear_shared_memory(segment_id)

    logging.info("starting standalone") 
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

    logging.info("setting pressures to minimal values")         
    _set_min_pressures(config,pam_config)
    logging.info("init exit signal handler")         
    signal_handler.init()
    try:
        time_c = time.time()-6
        while not signal_handler.has_received_sigint():
            time.sleep(0.01)
            if time.time()-time_c > 5:
                logging.info("running ...")
                time_c = time.time()
    except (KeyboardInterrupt,SystemExit):
        logging.info("exiting ...")
    except Exception as e:
        logging.error(str(e))
    finally:
        logging.info("setting pressures to minimal values")         
        _set_min_pressures(config,pam_config)
        logging.info("stopping standalone")
        o80_pam.stop_standalone(config.segment_id)
        logging.info("cleaning shared memory") 
        o80.clear_shared_memory(segment_id)
        logging.info("exit")
        
        
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
    print()
    return config
