import unittest
from inverse_kinematics import inverse_kinematics

class TestInverseKinematics(unittest.TestCase):
    def test_inverse_kinematics_zeros(self):
        zero_pos = inverse_kinematics(0, 0)
        self.assertEqual(inverse_kinematics(0, 0), zero_pos)

if __name__ == '__main__':
    unittest.main()