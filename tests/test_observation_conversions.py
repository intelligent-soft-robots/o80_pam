import unittest
import tempfile
from pathlib import Path
from o80_pam.observation_convertors import (
    dict_to_observation,
    observations_to_pandas,
    pandas_to_observations,
    observations_to_numpy,
    numpy_to_dict,
    convert_native_file_to_pandas,
    read_pandas,
    observation_to_numpy
)
import o80_pam




class O80_PAM_OBSERVATION_CONVERTORS_TESTCASE(unittest.TestCase):

    obs1 = {
        "iteration": 13,
        "frequency": 100,
        "time_stamp": 12,
        "positions": [1.,2.,3.,4.],
        "velocities": [10.,20.,30.,40.],
        "desired_pressures": [10,-10,20,-20,30,-30,40,-40],
        "observed_pressures": [11,-11,21,-21,31,-31,41,-41],
        "references_found": [True,True,False,True]
    }
    obs2 = {
        "iteration": 33,
        "frequency": 120,
        "time_stamp": 18,
        "positions": [1.,2.1,3.4,4.],
        "velocities": [15.,20.,31.,40.],
        "desired_pressures": [10,-16,24,-20,30,-30,45,-40],
        "observed_pressures": [11,-12,21,-21,31,-31,42,-46],
        "references_found": [False,True,True,False]
    }

    def _compare(self, a: dict, b: dict):
        self.assertEqual(set(a.keys()),set(b.keys()))
        value_keys = ("iteration","frequency","time_stamp")
        for vk in value_keys:
            self.assertEqual(a[vk],b[vk])
        for k in [k for k in a.keys() if not k in value_keys]:
            la = list(a[k])
            lb = list(b[k])
            self.assertEqual(len(la),len(lb))
            for v1, v2 in zip(la,lb):
                self.assertEqual(v1,v2)

    
    def test_conversions(self):
        observations_d = [self.obs1,self.obs2]
        observations = [dict_to_observation(obs) for obs in observations_d]
        dataframe = observations_to_pandas(observations)
        observations = pandas_to_observations(dataframe)
        np_observations, cols = observations_to_numpy(observations)
        d1 = numpy_to_dict(np_observations[0])
        d2 = numpy_to_dict(np_observations[1])
        self._compare(self.obs1,d1)
        self._compare(self.obs2,d2)

    def test_native_vs_pandas(self):

        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)
            observations_d = [self.obs1,self.obs2]
            observations = [dict_to_observation(obs) for obs in observations_d]
            native_path = tmp_dir / "native"
            pandas_path = tmp_dir / "pandas"
            serializer = o80_pam.Serializer()
            with open(native_path,"wb+") as f:
                for obs in observations:
                    f.write(serializer.serialize(obs))
                    f.flush()
            observations1 = o80_pam.read_file(native_path)
            convert_native_file_to_pandas(native_path, pandas_path)
            dataframe = read_pandas(pandas_path)
            observations2 = pandas_to_observations(dataframe)
            obs1 = list(observations2)[0]
            obs2 = list(observations2)[1]
            np1 = observation_to_numpy(obs1)
            np2 = observation_to_numpy(obs2)
            d1 = numpy_to_dict(np1)
            d2 = numpy_to_dict(np2)
            self._compare(self.obs1,d1)
            self._compare(self.obs2,d2)


