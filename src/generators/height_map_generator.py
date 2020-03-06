import numpy as np
from klampt.model import collide
from src.utils.logger import Logger
from src.utils.data_objects.height_map import HeightMap


class HeightMapGenerator:

    def __init__(self, ProjectConstants, world, test_suite):

        self.ProjectConstants = ProjectConstants
        self.world = world

        self.geometry_list = []
        self.initialized = False

        self.x_start = test_suite.HM_X_START
        self.x_end = test_suite.HM_X_END
        self.y_start = test_suite.HM_Y_START
        self.y_end = test_suite.HM_Y_END


        self.x_granularity = self.ProjectConstants.HM_X_GRANULARITY
        self.y_granularity = self.ProjectConstants.HM_Y_GRANULARITY

        self.x_indices = (self.x_end - self.x_start) / self.x_granularity
        self.y_indices = (self.y_end - self.y_start) / self.y_granularity

        # save geometry3d objects
        self.geometry_list = [self.world.terrain(j).geometry() for j in range(self.world.numTerrains())]
        for i in range(self.world.numRigidObjects()):
            self.geometry_list.append(self.world.rigidObject(i).geometry())

        # create height map obj
        if np.fabs(self.x_indices-int(self.x_indices)) > .0001:
            Logger.log(" Potential Error: x indices != int(x indices)", "WARNING")

        if np.fabs(self.y_indices-int(self.y_indices)) > .0001:
            Logger.log(" Potential Error: y indices != int(y indices)", "WARNING")

        x_vars = [self.x_start, self.x_end, self.x_granularity]
        y_vars = [self.y_start, self.y_end, self.y_granularity]

        # Height map variables
        self.HeightMap: HeightMap = HeightMap(x_vars, y_vars)

        self.initialized = True

    def height_at_xy(self, x, y):
        source = [x, y, 5]
        collide_rc = collide.ray_cast(self.geometry_list, source, [0, 0, -1])
        try:
            z = collide_rc[1][2]
            return z
        except TypeError:
            print("Error: no collision detected at", x, ",", y)
            return 0

    def build_height_map(self):
        ''' returns runtime
        '''
        return self.HeightMap.build_height_map(self.height_at_xy)

    def return_height_map(self):
        return self.HeightMap

    def visualize_height_map(self):
        self.HeightMap.visualize()
