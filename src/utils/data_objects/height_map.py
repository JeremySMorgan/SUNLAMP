import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import cm
import pickle
from .gradient_map import GradientMap
from .map_superclass import Map
from klampt import vis
from klampt.model import trajectory
from src.utils.data_objects.output_superclass import OutputSuperclass


class HeightMap(Map, OutputSuperclass):

    def __init__(self, x_vars, y_vars):

        # super(self.__class__, self).__init__()
        Map.__init__(self)
        OutputSuperclass.__init__(self, "height map")

        self.x_vars = x_vars
        self.y_vars = y_vars

        self.x_start = x_vars[0]
        self.x_end = x_vars[1]
        self.x_granularity = x_vars[2]

        self.y_start = y_vars[0]
        self.y_end = y_vars[1]
        self.y_granularity = y_vars[2]

        self.x_indices = int((self.x_end - self.x_start) / self.x_granularity)
        self.y_indices = int((self.y_end - self.y_start) / self.y_granularity)

        self.np_arr = np.ndarray(shape=(self.x_indices, self.y_indices))
        self.xy_to_height_dict = {}

    def get_gradient_map_obj(self):
        '''
            returns gradient map, runtime to build gm
        '''
        t_start = time.time()
        gm = GradientMap(self)
        gm.runtime = time.time() - t_start
        gm.failed = False
        return gm

    def get_np_hm_array(self):
        return self.np_arr

    def round(self,n):
        return np.round(n, decimals=self.decimal_round)

    def height_at_xy(self, x, y):
        # stance = (np.round(x, decimals=self.decimal_round), np.round( y, decimals=self.decimal_round))
        # if stance in self.xy_to_height_dict:
        #     return self.xy_to_height_dict[stance]
        h = self.np_arr[self.get_xindex_from_x(x)][ self.get_yindex_from_y(y)]
        # self.xy_to_height_dict[stance] = h
        return h

    def build_height_map(self, height_at_xy_func, debug=False):

        world_x = self.x_start
        world_y = self.y_start

        for x_idx in range(0, self.x_indices):

            world_y = self.y_start
            for y_idx in range(0, self.y_indices):

                self.np_arr.itemset((x_idx, y_idx), height_at_xy_func(world_x, world_y))
                world_y += self.y_granularity

            world_x += self.x_granularity

    def visualize(self):

        import matplotlib.pyplot as plt

        plt.imshow(self.np_arr[::5, ::5])
        plt.show()
        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # X = np.arange(self.x_start, self.x_end, self.x_granularity).transpose()
        # Y = np.arange(self.y_start, self.y_end, self.y_granularity).transpose()
        # X, Y = np.meshgrid(X, Y)
        # Z = self.height_map.transpose()
        # ax.set_title('Height Map')
        # ax.set_zlabel('Z')
        # ax.set_ylabel('Y')
        # ax.set_xlabel('X')
        # surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        # fig.colorbar(surf, shrink=0.5, aspect=5)
        # plt.show()

    def visualize_in_klampt(self, step=5, xmin=4, xmax=12, ymin=8, ymax=12):

        for x_inx in range(self.get_xindex_from_x(xmin), self.get_xindex_from_x(xmax), step):
            for y_inx in range(self.get_yindex_from_y(ymin), self.get_yindex_from_y(ymax), step):

                x_world = self.get_x_from_xindex(x_inx)
                y_world = self.get_y_from_yindex(y_inx)
                z = self.np_arr[x_inx][y_inx]
                if z > 0:
                    name = str(x_world) + ", " + str(y_world)
                    traj = trajectory.Trajectory(milestones=[[x_world, y_world, 0], [x_world, y_world, z]])
                    vis.add(name, traj)
                    vis.hideLabel(name)

    def print_stats(self):
        print("<HeightMap Obj>")
        print(f"      failed:\t\t{self.failed}")
        print(f"      runtime:\t\t{round(self.runtime, 2)}")
        print(f"      x range:\t\t{round(self.x_start, 2)} - {round(self.x_end, 2)}")
        print(f"      y range:\t\t{round(self.y_start, 2)} - {round(self.y_end, 2)}")
        print(f"      np array size:\t[{self.x_indices} {self.y_indices}]\n")