import unittest
from inverse_kinematics import inverse_kinematics, forward_kinematics
import config as c

class TestInverseKinematics(unittest.TestCase):
    def test_inverse_kinematics_zeros(self):
        zero_pos = inverse_kinematics(0, 0)
        self.assertEqual(inverse_kinematics(0, 0), zero_pos)
        
    def test_inverse_forward_consistency(self):
        test_positions = [
            (0.0, 0.0),
            (0.5, 0.5),
            (1.0, 1.0),
            (0.25, 0.75),
            (0.75, 0.25)
        ]
        
        for x, y in test_positions:
            with self.subTest(x=x, y=y): # type: ignore
                shoulder, elbow = inverse_kinematics(x, y)
                x_f, y_f = forward_kinematics(shoulder, elbow)
                
                self.assertAlmostEqual(x, x_f, places=2)
                self.assertAlmostEqual(y, y_f, places=2)
        
    def test_out_of_bounds(self):
        out_of_bounds = [
            (100, 100),
            (-1, -1),
        ]
        
        for x, y in out_of_bounds:
            with self.subTest(x=x, y=y): # type: ignore
                with self.assertRaises(ValueError): # type: ignore
                    inverse_kinematics(x, y)

class TestVoltageToAngle(unittest.TestCase):
    def voltage_to_angle(self, v: float, vmin: float, vmax: float, amin: float, amax: float) -> float:
        if v < vmin:
            v = vmin
        if v > vmax:
            v = vmax
        return amin + ((v - vmin) / (vmax - vmin)) * (amax - amin)
    
    def test_shoulder_mid_voltage(self):
        mid_v = (c.MIN_VOLTAGE_SHOULDER + c.MAX_VOLTAGE_SHOULDER) / 2
        expected_angle = c.MIN_ANGLE_SHOULDER + ((mid_v - c.MIN_VOLTAGE_SHOULDER) / (c.MAX_VOLTAGE_SHOULDER - c.MIN_VOLTAGE_SHOULDER)) * (c.MAX_ANGLE_SHOULDER - c.MIN_ANGLE_SHOULDER)
        self.assertAlmostEqual(
            self.voltage_to_angle(mid_v, c.MIN_VOLTAGE_SHOULDER, c.MAX_VOLTAGE_SHOULDER, c.MIN_ANGLE_SHOULDER, c.MAX_ANGLE_SHOULDER),
            expected_angle
        )

    def test_elbow_mid_voltage(self):
        mid_v = (c.MIN_VOLTAGE_ELBOW + c.MAX_VOLTAGE_ELBOW) / 2
        expected_angle = c.MIN_ANGLE_ELBOW + ((mid_v - c.MIN_VOLTAGE_ELBOW) / (c.MAX_VOLTAGE_ELBOW - c.MIN_VOLTAGE_ELBOW)) * (c.MAX_ANGLE_ELBOW - c.MIN_ANGLE_ELBOW)
        self.assertAlmostEqual(
            self.voltage_to_angle(mid_v, c.MIN_VOLTAGE_ELBOW, c.MAX_VOLTAGE_ELBOW, c.MIN_ANGLE_ELBOW, c.MAX_ANGLE_ELBOW),
            expected_angle
        )

    def test_shoulder_min_max_clamp(self):
        self.assertEqual(
            self.voltage_to_angle(c.MIN_VOLTAGE_SHOULDER - 1, c.MIN_VOLTAGE_SHOULDER, c.MAX_VOLTAGE_SHOULDER, c.MIN_ANGLE_SHOULDER, c.MAX_ANGLE_SHOULDER),
            c.MIN_ANGLE_SHOULDER
        )
        self.assertEqual(
            self.voltage_to_angle(c.MAX_VOLTAGE_SHOULDER + 1, c.MIN_VOLTAGE_SHOULDER, c.MAX_VOLTAGE_SHOULDER, c.MIN_ANGLE_SHOULDER, c.MAX_ANGLE_SHOULDER),
            c.MAX_ANGLE_SHOULDER
        )

    def test_elbow_min_max_clamp(self):
        self.assertEqual(
            self.voltage_to_angle(c.MIN_VOLTAGE_ELBOW - 1, c.MIN_VOLTAGE_ELBOW, c.MAX_VOLTAGE_ELBOW, c.MIN_ANGLE_ELBOW, c.MAX_ANGLE_ELBOW),
            c.MIN_ANGLE_ELBOW
        )
        self.assertEqual(
            self.voltage_to_angle(c.MAX_VOLTAGE_ELBOW + 1, c.MIN_VOLTAGE_ELBOW, c.MAX_VOLTAGE_ELBOW, c.MIN_ANGLE_ELBOW, c.MAX_ANGLE_ELBOW),
            c.MAX_ANGLE_ELBOW
        )

if __name__ == '__main__':
    unittest.main()