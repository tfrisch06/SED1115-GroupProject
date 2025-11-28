import unittest
from inverse_kinematics import inverse_kinematics

class TestInverseKinematics(unittest.TestCase):
    def test_inverse_kinematics_zeros(self):
        zero_pos = inverse_kinematics(0, 0)
        self.assertEqual(inverse_kinematics(0, 0), zero_pos)

    def test_straight_out(self):
        mid_point_x = 215
        mid_point_y = 295/2

        straight_out_shouler, straight_out_elbow = inverse_kinematics(mid_point_x, mid_point_y)
        if straight_out_shouler is None or straight_out_elbow is None:
            self.fail
        else:
            self.assertEqual((int(straight_out_shouler), int(straight_out_elbow)), (-31, -62))
if __name__ == '__main__':
    unittest.main()