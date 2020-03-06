from unittest import TestCase
from src.lidar.pcloud_parser import PCloudParser

class TestPCloudParser(TestCase):

    def test_best_fitting_plane(self):

        xyzs = []

        for x in range(0, 10):
            for y in range(0, 10):

                z = 10 - x
                xyz = [x, y, z]
                xyzs.append(xyz)

        a, b, c, x0, y0, z0 = PCloudParser.best_fitting_plane(xyzs)



        print("a, b, c, x0, y0, z0:", a, b, c, x0, y0, z0)

