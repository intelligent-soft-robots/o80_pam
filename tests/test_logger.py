import unittest

import o80_pam
import tempfile

class O80_PAM_LOGGER_TESTCASE(unittest.TestCase):

    def test_exception_on_invalid_path(self):
        self.assertRaises(FileNotFoundError,
                          o80_pam.Logger(os.join("not","existing")))

    def test_serialization(self):
        o1 = o80_pam.Observation()
        serializer = o80_pam.Serializer()
        s = serializer.serialize(o1)
        o2 = serializer.deserialize(s)
        self.assertEqual(o1.get_frequency(),o2.get_frequency())
        self.assertEqual(o1.get_iteration(),o2.get_iteration())
        self.assertEqual(o1.get_time_stamp(),o2.get_time_stamp())
        for dof in range(4):
            for f in ("get_desired_pressures",
                      "get_observed_pressures",
                      "get_references_found",
                      "get_velocities",
                      "get_positions"):
                self.assertEqual(getattr(o1,f),
                                 getattr(o2,f))
            self.assertEqual(o1.get_desired_states().get(dof).get(),
                             o2.get_desired_states().get(dof).get())
            self.assertEqual(o1.get_observed_states().get(dof).get(),
                             o2.get_observerd_states().get(dof).get())
                             
