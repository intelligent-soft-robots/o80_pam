from typing import Iterable, Tuple
import numpy as np
from functools import reduce, partial
from o80_pam import Observation
from pam_interface import RobotState

_dtype = np.float16

def _pressure_columns(prefix: str) -> tuple[str, ...]:
    """
    return suitable name for columns related to pressure values, e.g.
    'desired_pressure_1_antago' refers to the desired pressure of the 
    antagonist muscle of the second degree of freedom ('desired' is here
    the prefix passed as argument)
    """
    def _mtype(index: int) -> str:
        return "antago" if index % 2 else "ago"

    return tuple([f"{prefix}_pressure_{int(i/2)}_{_mtype(i)}" for i in range(8)])


def _pressures_to_numpy(pressures: list[tuple[int, int]]) -> np.ndarray:
    """
    cast [(1,1),(0,0),(2,1),(1,2)] to [1,1,0,0,2,1,1,2]
    """
    return np.array(reduce(lambda a, b: list(a) + list(b), pressures), dtype=_dtype)


def _np_array(value)->np.array:
    if type(value) in (int, float, bool):
        return np.array([value],dtype=_dtype)
    else:
        return np.array(value,dtype=_dtype)
    

_columns = {
    "get_iteration": (["iteration"], _np_array),
    "get_frequency": (["frequency"], _np_array),
    "get_time_stamp": (["time_stamp"], _np_array),
    "get_positions": (tuple([f"position_{dof}" for dof in range(4)]), _np_array),
    "get_velocities": (tuple([f"velocity_{dof}" for dof in range(4)]), _np_array),
    "get_desired_pressures": (_pressure_columns("desired"), _pressures_to_numpy),
    "get_observed_pressures": (_pressure_columns("observed"), _pressures_to_numpy),
    "get_references_found": (
        tuple([f"reference_found_{dof}" for dof in range(4)]),
        _np_array,
    ),
}
"""
key is the name of a getter function of the class Observation.
Values are a tuple [ list of related column names, function for casting
value returned by the getter function to a numpy array ] 
"""


_ordered_getters = (
    "get_iteration",
    "get_frequency",
    "get_time_stamp",
    "get_positions",
    "get_velocities",
    "get_desired_pressures",
    "get_observed_pressures",
    "get_references_found",
)
"""
ordered Observation getter method (ordered: order of the related
columns in a numpy array)
"""

def observation_to_numpy(observation: Observation) -> np.array:
    """
    Cast an observation to a numpy 1d array. Order of values based
    on the _ordered_getters tuple.
    """
    values = [
        _columns[getter][1](getattr(observation, getter)())
        for getter in _ordered_getters
    ]
    return np.concatenate(values)


def _numpy_to_states(values: np.array)->States:
    states = States()
    for index, state in zip(values, states.values):
        state.set(values[index])
    return states

def _numpy_to_dict(values: np.array)->dict:
    def _int(values):
        if type(values)==float:
            return int(values)
        return [int(v) for v in values]
    def _bool(values):
        if type(values)==float:
            return bool(values)
        return [bool(v) for v in values]
    return {
        "iteration": _int(values[0]),
        "frequency": values[1],
        "time_stamp": _int(values[2]),
        "positions": values[3:7],
        "velocities": values[7:11],
        "desired_pressures": _int(values[11:18]),
        "observed_pressures": _int(values[18:25]),
        "references_found": _bool(values[26])
    }

    
def numpy_to_observation(values: np.array)->Observation:
    d = _numpy_to_dict(values)
    observed_states = _numpy_to_states(d["observed_pressures"])
    desired_states = _numpy_to_states(d["desired_pressures"])
    extended_states = RobotState()
    for dof in range(4):
        agonist = observed_states.values[dof*2].get()
        antagonist = observed_states.values[dof*2+1].get()
        desired_agonist = desired_states.values[dof*2].get()
        desired_antagonist = desired_states.values[dof*2+1].get()
        extended_states.set_joint(
            dof, agonist, antagonist, desired_agonist, desired_antagonist,
            d["positions"], d["velocities"], -1, d["references_found"]
        )
    return Observation(
        observed_states, desired_states, extended_states,
        d["time_stamp"], d["iteration"],d["iteration"], d["frequency"]
    )

def numpy_to_observations(values: np.ndarray)->list[Observation]:
    return [
        numpy_to_observation(v) for v in values.T
    ]


def observations_to_numpy(
    observations: Iterable[Observation],
) -> Tuple[np.ndarray, Tuple[str, ...]]:
    """
    Returns a matrix and the related column names. Each line correspond to an observation.
    """
    observation_matrix = np.row_stack(
        [observation_to_numpy(obs) for obs in observations]
    )
    column_names = reduce(
        lambda a, b: list(a)+list(b),
        [_columns[getter][0] for getter in _ordered_getters],
    )
    return observation_matrix, column_names


def observations_to_panda(
            observations: Iterable[Observation]
)->pd.DataFrame:
    data, columns = observations_to_numpy(observations)
    return pd.DataFrame(data, columns=columns)
    

def convert_to_panda(origin_path: Path)->:
    observations = o80_pam.read_file(origin_path)
    logger.info(f"read {len(observations} from {origin_path}")
    return observations_to_panda(observations)

def pickle_observations(observations: Iterable[Observation], destination_file: Path)->None:
    pass
