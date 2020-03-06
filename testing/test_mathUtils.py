from unittest import TestCase
from src.utils.math_utils import MathUtils
import numpy as np
import time


class TestMathUtils(TestCase):

    def test_angle_rad_between_3d_vectors(self):
        v1 = [1, 0, 0]
        v2 = [0, 1, 0]

        angle_known = np.deg2rad(90)
        angle_calculated = MathUtils.angle_between_two_3d_vectors(v1, v2)
        self.assertAlmostEqual(angle_known, angle_calculated)

        v1 = [1, 0, 0]
        v2 = [1, 1, 0]

        angle_known = np.deg2rad(45)
        angle_calculated = MathUtils.angle_between_two_3d_vectors(v1, v2)
        self.assertAlmostEqual(angle_known, angle_calculated)

    def test_rotation_matrix_between_3d_vectors(self):
        a = [1, 0, 0]
        b = [0, 1, 0]

        # 90deg rotation about z

        R = MathUtils.rotation_matrix_between_3d_vectors(a, b)

        print(R)

        # self.fail()

    def test_best_fitting_plane(self):
        hm = np.zeros((10, 10))
        print("hm:")
        print(hm)

        start_t = time.time()
        max_cnt = 1000
        for i in range(max_cnt):
            a, b, c, z0 = MathUtils.best_fitting_plane(hm)
        print(f" ran {max_cnt} in {round(time.time() - start_t, 2)} seconds")
        # ran 1000 in 22.07 seconds with double for loop
        # ran 1000 in 5.95 seconds with np.fromfunction(...)

        a, b, c, z0 = MathUtils.best_fitting_plane(hm)
        print("a, b, c, z0: ", a, b, c, z0)

        self.assertAlmostEqual(a, 0)
        self.assertAlmostEqual(b, 0)
        self.assertAlmostEqual(c, 1)
        self.assertAlmostEqual(z0, 0)

        hm = np.ones((5, 5))
        a, b, c, z0 = MathUtils.best_fitting_plane(hm)
        print("\nhm:")
        print(hm)
        print("[ones array] a, b, c, z0: ", a, b, c, z0)

        self.assertAlmostEqual(a, 0)
        self.assertAlmostEqual(b, 0)
        self.assertAlmostEqual(c, 1)
        self.assertAlmostEqual(z0, 1)

        hm = np.array([[0, 1, 2, 3],
                       [1, 2, 3, 4],
                       [2, 3, 4, 5],
                       [3, 4, 5, 6]])

        a, b, c, z0 = MathUtils.best_fitting_plane(hm)
        print("\nhm:")
        print(hm)
        print(" a, b, c, z0: ", a, b, c, z0)

        hm = np.array([[0, 1, 2, 3],
                       [0, 1, 2, 3],
                       [0, 1, 2, 3],
                       [0, 1, 2, 3]])
        a, b, c, z0 = MathUtils.best_fitting_plane(hm)
        print("\nhm:")
        print(hm)
        print(" a, b, c, z0: ", a, b, c, z0)
        self.assertAlmostEqual(a, 0)
        self.assertAlmostEqual(b, -c, places=3)
        self.assertAlmostEqual(z0, 0, places=5)

    def test_max_joint_angle_distance(self):

        pi = np.pi
        eps = .001

        q1 = [0, pi - eps, 0]
        q2 = [0, -pi + eps, 0]
        max_joint_dist = MathUtils.max_joint_angle_distance(q1, q2)
        print("max_joint_dist: ",max_joint_dist)
        print(" 2*eps: ", 2*eps)
        self.assertAlmostEqual(max_joint_dist, 2*eps)

        q1 = [0, pi/4 - eps, 0]
        q2 = [0, -pi/4, 0]
        max_joint_dist = MathUtils.max_joint_angle_distance(q1, q2)
        known_max_joint_dist = pi/2 - eps
        print("known_max_joint_dist: ", known_max_joint_dist)
        print("Calculated: ", max_joint_dist)
        self.assertAlmostEqual(max_joint_dist, known_max_joint_dist)

        q1 = [0, pi - eps, 0]
        q2 = [0, -pi, 0]
        max_joint_dist = MathUtils.max_joint_angle_distance(q1, q2)
        known_max_joint_dist = eps
        print("known_max_joint_dist: ", known_max_joint_dist)
        print("Calculated: ", max_joint_dist)
        self.assertAlmostEqual(max_joint_dist, known_max_joint_dist)
