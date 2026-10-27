import logging
import sys
import os
import time
import o80
import o80_pam
import pam_interface
import signal_handler
import shared_memory
from lightargs import BrightArgs, Set, Range, Positive, FileExists

# code used for starting o80 pam
# in the executable o80_pam/o80_real and
# o80_pam/o80_dummy


def _set_min_pressures(config, pam_config):

    frontend = o80_pam.FrontEnd(config.segment_id)
    for dof in range(4):
        min_ago = pam_config.min_pressure(dof, pam_interface.sign.agonist)
        min_antago = pam_config.min_pressure(dof, pam_interface.sign.antagonist)
        frontend.add_command(dof, min_ago, min_antago, o80.Duration_us.seconds(3), o80.Mode.OVERWRITE)
    frontend.pulse()
    del frontend


def _limits_segment_id(segment_id):
    return segment_id + "_pressure_limits"


def _write_pressure_limits(segment_id, pam_config):
    limits_id = _limits_segment_id(segment_id)
    shared_memory.clear_shared_memory(limits_id)
    for dof in range(4):
        for sign, name in (
            (pam_interface.sign.agonist, "ago"),
            (pam_interface.sign.antagonist, "antago"),
        ):
            shared_memory.set_int(
                limits_id, "min_{}_{}".format(name, dof), pam_config.min_pressure(dof, sign)
            )
            shared_memory.set_int(
                limits_id, "max_{}_{}".format(name, dof), pam_config.max_pressure(dof, sign)
            )


def read_pressure_limits(segment_id, nb_dofs=4):
    """
    Reads the pressure limits written by run() for the backend running
    with the specified segment_id.
    Returns (min_agos, max_agos, min_antagos, max_antagos), each a list
    of length nb_dofs.
    """
    limits_id = _limits_segment_id(segment_id)

    def _read(prefix, name):
        return [
            shared_memory.get_int(limits_id, "{}_{}_{}".format(prefix, name, dof))
            for dof in range(nb_dofs)
        ]

    return (
        _read("min", "ago"),
        _read("max", "ago"),
        _read("min", "antago"),
        _read("max", "antago"),
    )


def run(config):
    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format="[o80 PAM segment_id: {} | %(levelname)s %(asctime)s] %(message)s".format(
            config.segment_id
        ),
        level=logging.DEBUG,
        handlers=[log_handler],
    )

    logging.info("read configuration")
    segment_id = config.segment_id
    frequency = config.frequency
    bursting_mode = config.bursting_mode
    robot = config.robot
    pam_config = pam_interface.JsonConfiguration(config.pam_config_file)

    if robot=="pamy2":
        ip = config.ip 
        port = config.port

    logging.info("starting robot: {}".format(robot))
    
    logging.info("cleaning shared memory")
    o80.clear_shared_memory(segment_id)

    logging.info("starting standalone")
    if robot == "pamy1":
        o80_pam.pamy1_start_standalone(segment_id, frequency, bursting_mode, pam_config)
    elif robot == "pamy2":
        o80_pam.pamy2_start_standalone(segment_id, frequency, bursting_mode, pam_config, ip, port)
    else:
        o80_pam.dummy_start_standalone(segment_id, frequency, bursting_mode, pam_config)
    _write_pressure_limits(segment_id, pam_config)

    logging.info("setting pressures to minimal values")
    _set_min_pressures(config, pam_config)
    logging.info("init exit signal handler")
    signal_handler.init()
    try:
        time_c = time.time() - 6
        while not signal_handler.has_received_sigint():
            time.sleep(0.01)
            if time.time() - time_c > 5:
                logging.info("running ...")
                time_c = time.time()
    except (KeyboardInterrupt, SystemExit):
        logging.info("exiting ...")
    except Exception as e:
        logging.error(str(e))
    finally:
        logging.info("setting pressures to minimal values")
        _set_min_pressures(config, pam_config)
        logging.info("stopping standalone")
        o80_pam.stop_standalone(config.segment_id)
        logging.info("cleaning shared memory")
        o80.clear_shared_memory(segment_id)
        shared_memory.clear_shared_memory(_limits_segment_id(segment_id))
        logging.info("exit")


def configure(robot : str):
    config = BrightArgs("o80 PAM {} robot".format(robot))
    setattr(config, "robot", robot)
    config.add_option(
        "segment_id", o80_pam.segment_ids.robot, "o80 backend segment_id ", str
    )
    config.add_operation(
        "bursting_mode", "will the backend be started in bursting mode ?"
    )
    config.add_option(
        "frequency",
        500.,
        "o80 backend frequency (non bursting mode only)",
        float,
        integrity_checks=[Range(1, 3000), Positive()],
    )
    if robot=="pamy2":
        config.add_option(
            "ip",
            "192.168.0.110",
            "IP of the udp socket of the robot",
            str)
        config.add_option(
            "port",
            4700,
            "PORT of the udp socket of the robot",
            int)
        config.add_option(
            "pam_config_file",
            pam_interface.Pamy2DefaultConfiguration.get_path(False),
            "pam configuration file",
            str,
            integrity_checks=[FileExists()],
        )
        config.nb_dofs=4
    else:
        config.add_option(
            "nb_dofs",
            4,
            "number of degrees of freedom of the robots",
            int,
            integrity_checks=[Positive()],
        )
        config.add_option(
            "pam_config_file",
            pam_interface.Pamy1DefaultConfiguration.get_path(False),
            "pam configuration file",
            str,
            integrity_checks=[FileExists()],
        )
    change_all = False
    finished = config.dialog(change_all, sys.argv[1:])
    print()
    if not finished:
        return None
    return config


class run_dummy_robot:
    def __init__(self, segment_id, frequency, bursting_mode, pam_config_path):
        self._segment_id = segment_id
        self._frequency = frequency
        self._bursting_mode = bursting_mode
        self._pam_config_path = pam_config_path
        if not os.path.isfile(pam_config_path):
            raise FileNotFoundError("failed to find: {}".format(pam_config_path))

    def __enter__(self):
        pam_config = pam_interface.JsonConfiguration(self._pam_config_path)
        o80_pam.dummy_start_standalone(
            self._segment_id, self._frequency, self._bursting_mode, pam_config
        )
        return

    def __exit__(self, _, __, ___):
        o80_pam.stop_standalone(self._segment_id)
        o80.clear_shared_memory(self._segment_id)
