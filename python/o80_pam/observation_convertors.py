from typing import Iterable, Tuple, List
from pathlib import Path
import numpy as np
import pandas as pd
from functools import reduce, partial
import o80_pam
from pam_interface import RobotState

_dtype = np.float64


def _pressure_columns(prefix: str) -> Tuple[str, ...]:
    """
    return suitable name for columns related to pressure values, e.g.
    'desired_pressure_1_antago' refers to the desired pressure of the
    antagonist muscle of the second degree of freedom ('desired' is here
    the prefix passed as argument)
    """

    def _mtype(index: int) -> str:
        return "antago" if index % 2 else "ago"

    return tuple([f"{prefix}_pressure_{int(i/2)}_{_mtype(i)}" for i in range(8)])


def _pressures_to_numpy(pressures: List[Tuple[int, int]]) -> np.ndarray:
    """
    cast [(1,1),(0,0),(2,1),(1,2)] to [1,1,0,0,2,1,1,2]
    """
    return np.array(reduce(lambda a, b: list(a) + list(b), pressures), dtype=_dtype)


def _np_array(value) -> np.array:
    if type(value) in (int, float, bool):
        return np.array([value], dtype=_dtype)
    else:
        return np.array(value, dtype=_dtype)


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


def observation_to_numpy(observation: o80_pam.Observation) -> np.array:
    """
    Cast an observation to a numpy 1d array. Order of values: 
    iteration, frequency, time stamp, positions (4d), velocities (4d),
    desired pressures (8d), observed pressures (8d), references found (4d)
    """
    values = [
        _columns[getter][1](getattr(observation, getter)())
        for getter in _ordered_getters
    ]
    return np.concatenate(values)


def _numpy_to_states(values: np.array) -> o80_pam.States:
    states = o80_pam.States()
    for index, state in enumerate(states.values):
        state.set(values[index])
    return states


def numpy_to_dict(values: np.array) -> dict:
    def _int(values):
        if type(values) in (float, _dtype):
            return int(values)
        return [int(v) for v in values]

    def _bool(values):
        if type(values) in (float, _dtype):
            return bool(values)
        return [bool(v) for v in values]
    return {
        "iteration": _int(values[0]),
        "frequency": values[1],
        "time_stamp": _int(values[2]),
        "positions": values[3:7],
        "velocities": values[7:11],
        "desired_pressures": _int(values[11:19]),
        "observed_pressures": _int(values[19:27]),
        "references_found": _bool(values[27:33]),
    }


def dict_to_observation(d: dict)->o80_pam.Observation:
    """
    Cast a dictionary to an instance of Observation.
    The dictionary must have the keys "iteration", "frequency",
    "time_stamp", "positions", "velocities", "desired_pressures",
    "observed_pressures" and "references_found"
    """
    observed_states = _numpy_to_states(d["observed_pressures"])
    desired_states = _numpy_to_states(d["desired_pressures"])
    extended_states = RobotState()
    for dof in range(4):
        agonist = observed_states.values[dof * 2].get()
        antagonist = observed_states.values[dof * 2 + 1].get()
        desired_agonist = desired_states.values[dof * 2].get()
        desired_antagonist = desired_states.values[dof * 2 + 1].get()
        extended_states.set_joint(
            dof,
            agonist,
            antagonist,
            desired_agonist,
            desired_antagonist,
            d["positions"][dof],
            d["velocities"][dof],
            -1,
            d["references_found"][dof],
        )
    return o80_pam.Observation(
        observed_states,
        desired_states,
        extended_states,
        d["time_stamp"],
        d["iteration"],
        d["frequency"],
    )


def numpy_to_observation(values: np.array) -> o80_pam.Observation:
    """
    Cast a numpy array to an instance of Observation.
    Assumes the numpy array has been created via the function
    'observation_to_numpy' (or indirectly via the function 
    'observations_to_pandas').
    """
    d = numpy_to_dict(values)
    return dict_to_observation(d)


def numpy_to_observations(values: np.ndarray) -> List[o80_pam.Observation]:
    """
    Cast a numpy matrix to instances of Observations. This corresponds to the
    function 'numpy_to_observation' called on each row of the matrix.
    """
    return [numpy_to_observation(v) for v in values]


def pandas_to_observations(dataframe: pd.DataFrame) -> List[o80_pam.Observation]:
    """
    Cast a pandas dataframe to instances of Observations.
    This assumes the dataframe has been created via the functions 'observations_to_pandas'
    or 'native_file_to_pandas'
    """
    return numpy_to_observations(dataframe.to_numpy())


def observations_to_numpy(
    observations: Iterable[o80_pam.Observation],
) -> Tuple[np.ndarray, Tuple[str, ...]]:
    """
    Returns a matrix, each line corresponding to an observation (see 'observation_to_numpy').
    Also returns corresponding column names.
    """
    observation_matrix = np.row_stack(
        [observation_to_numpy(obs) for obs in observations]
    )
    column_names = reduce(
        lambda a, b: list(a) + list(b),
        [_columns[getter][0] for getter in _ordered_getters],
    )
    return observation_matrix, column_names


def observations_to_pandas(observations: Iterable[o80_pam.Observation]) -> pd.DataFrame:
    """
    Cast instances of Observation to a pandas dataframe (which has explicit column names)
    """
    data, columns = observations_to_numpy(observations)
    return pd.DataFrame(data, columns=columns)


def native_file_to_pandas(origin_path: Path) -> pd.DataFrame:
    """
    Read the file (which is expected to have been created via 
    the executable 'o80_logger') and returns a corresponding 
    pandas dataframe.
    """
    observations = o80_pam.read_file(origin_path)
    return observations_to_pandas(observations)


def pickle_observations(
    observations: Iterable[o80_pam.Observation], destination_file: Path
) -> None:
    """
    Cast the observations to a pandas dataframe which is then pickled to 
    a file.
    """
    dataframe = observations_to_pandas(observations)
    dataframe.to_pickle(str(destination_file))


def convert_native_file_to_pandas(native_path: Path, destination_file: Path) -> None:
    """
    Read the file (which is expected to have been created via the executable 'o80_logger'),
    converts it to a pandas dataframe, which is then pickled in the destination file.
    """
    observations = o80_pam.read_file(native_path)
    pickle_observations(observations, destination_file)


def read_pandas(path: Path) -> pd.DataFrame:
    """
    Read the file (expected to have been created via 'convert_native_file_to_pandas'
    or 'pickle_observations') and returns the corresponding pandas dataframe.
    """
    return pd.read_pickle(path)
