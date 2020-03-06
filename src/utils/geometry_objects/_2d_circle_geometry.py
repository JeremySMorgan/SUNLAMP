from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LinearRing
import numpy as np
from .geometry_object_superclass import _3dGeometrySuperclass
from klampt.model import trajectory
from klampt import vis
import random

class LegRange(_3dGeometrySuperclass):

    '''
        This class stores a shapely circle and is used to represent the range of end effectors. It only supports 2d circles
        which is not precise but is sufficient for current purposes.
    '''

    def __init__(self, P, r, height_map, name=None):

        self.name = name
        self.x = P[0]
        self.y = P[1]
        self.z = P[2]
        self.center = Point(self.make_2d(P))
        self.R      = r
        self.shapely_poly = self.center.buffer(self.R)
        _3dGeometrySuperclass.__init__(self, height_map, name, self.shapely_poly)

    def visualize(self, arc_step=15):
        milestones = []
        for i in range(0, 361, arc_step):
            x = self.x + self.R*np.cos(np.deg2rad(i))
            y = self.y + self.R*np.sin(np.deg2rad(i))
            try:
                z = self.height_map.height_at_xy(x,y)
            except ValueError:
                z = 0
            milestones.append([x, y, z+.01])
        circle = trajectory.Trajectory(milestones=milestones)
        if not self.name:
            self.name = "circle "+str(random.randint(1,1000))
        vis.add(self.name, circle)
