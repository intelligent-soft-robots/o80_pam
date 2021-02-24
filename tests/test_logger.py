import unittest
import os
import o80_pam
import pam_interface
import tempfile
import time

class O80_PAM_LOGGER_TESTCASE(unittest.TestCase):

    def test_exception_on_invalid_path(self):
        try:
            o80_pam.Logger("foo_id",os.path.join("not","existing"))
            raised=False
        except FileNotFoundError:
            raised=True
        # note: self.assertRaises did not work
        self.assertTrue(raised)

    def test_serialization(self):
        o1 = o80_pam.Observation()
        serializer = o80_pam.Serializer()
        s = serializer.serialize(o1)
        o2 = serializer.deserialize(s)
        for f in ("get_frequency","get_iteration","get_time_stamp"):
            self.assertEqual(getattr(o1,f)(),getattr(o2,f)())
        for dof in range(4):
            for f in ("get_desired_pressures",
                      "get_observed_pressures",
                      "get_references_found",
                      "get_velocities",
                      "get_positions"):
                self.assertEqual(getattr(o1,f)(),
                                 getattr(o2,f)())
            self.assertEqual(o1.get_desired_states().get(dof).get(),
                             o2.get_desired_states().get(dof).get())
            self.assertEqual(o1.get_observed_states().get(dof).get(),
                             o2.get_observed_states().get(dof).get())

    def test_logger_ok(self):
        pam_config = pam_interface.DefaultConfiguration.get_path()
        frequency = 100
        bursting_mode = False
        segment_id = "logger_unit_tests"
        with tempfile.TemporaryDirectory() as tmp:
            log_path = os.path.join(tmp,"logger_ut")
            with o80_pam.run_dummy_robot(segment_id,frequency,bursting_mode,pam_config):
                with o80_pam.Logger(segment_id,log_path) as logger:
                    time.sleep(1.0)
            observations = list(o80_pam.read_file(log_path))
            self.assertGreater(len(observations),5)
        for o1,o2 in zip(observations,observations[1:]):
            self.assertGreater(o2.get_time_stamp(),o1.get_time_stamp())
            self.assertEqual(o1.get_iteration(),o2.get_iteration()-1)

            
                    
        
