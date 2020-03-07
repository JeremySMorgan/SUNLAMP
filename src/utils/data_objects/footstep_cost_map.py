import numpy as np
from scipy.interpolate import interp1d
import pickle as pickle
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from .map_superclass import Map
from klampt import vis
from klampt.model import trajectory
from src.utils.data_objects.output_superclass import OutputSuperclass


class FootstepCostMap(Map, OutputSuperclass):

    def __init__(self, hm_xvars, hm_yvars):

        super(self.__class__, self).__init__()
        OutputSuperclass.__init__(self, "fs cost map")

        self.x_vars = hm_xvars
        self.y_vars = hm_yvars
        self.x_start = hm_xvars[0]
        self.x_end = hm_xvars[1]
        self.x_granularity = hm_xvars[2]
        self.y_start = hm_yvars[0]
        self.y_end = hm_yvars[1]
        self.y_granularity = hm_yvars[2]

        self.x_indices = int((self.x_end - self.x_start) / self.x_granularity)
        self.y_indices = int((self.y_end - self.y_start) / self.y_granularity)

        self.np_cmap_arr = np.ndarray(shape=(self.x_indices, self.y_indices))
        self.np_cmap_arr.fill(-1)

    def get_max_value(self):
        return np.max(self.np_cmap_arr)

    def min_cost_in_circle_about_xy(self, x, y, r):

        xmin = self.get_xindex_from_x(min(x - r, self.x_start))
        xmax = self.get_xindex_from_x(max(x + r, self.x_end))
        ymin = self.get_yindex_from_y(min(y - r, self.y_start))
        ymax = self.get_yindex_from_y(max(y + r, self.y_end))
        return np.min(self.np_cmap_arr[xmin:xmax][ymin:ymax])

    def cost_at_xy(self, x, y):
        return self.np_cmap_arr[self.get_xindex_from_x(x)][self.get_yindex_from_y(y)]

    def get_height_map(self):
        raise RuntimeError("FootstepCostmap has had get_height_map() removed")

    def get_fs_cost_map_np_arr(self):
        return self.np_cmap_arr

    def save_cost_to_xy_index(self, x_index, y_index, cost):
        self.np_cmap_arr.itemset((x_index, y_index), cost)

    def normalize_cost_arr(self, normalize_to, debug=False):

        max_found_val = np.max(self.np_cmap_arr)

        if debug:
            print("max_found_val:", max_found_val, "\t normalize_to", normalize_to)

        if max_found_val != 0 and max_found_val > normalize_to:

            self.np_cmap_arr *= (normalize_to / max_found_val)

    def save_cost_to_xy(self, x, y, x_radius, y_radius, cost):

        '''
        summary: saves the given cost to the point at x,y, aswell as all points in a rectangular grid around the grid
                defined by the given x and y radius
        :param x: sav
        :param y:
        :param cost:
        :return:
        '''

        x_0, x_f, y_0, y_f = self.get_xy_start_finals(x,y,x_radius,y_radius)

        x_start_index = self.get_xindex_from_x(x_0)
        y_start_index = self.get_yindex_from_y(y_0)
        x_end_index = self.get_xindex_from_x(x_f)
        y_end_index = self.get_yindex_from_y(y_f)

        self.np_cmap_arr[x_start_index:x_end_index, y_start_index:y_end_index] = cost

    def visualize_in_klampt(self, height_map, step=10):
        for x_inx in range(0,self.x_indices-1, step):
            for y_inx in range(0,self.y_indices-1, step):
                x_world = self.get_x_from_xindex(x_inx)
                y_world = self.get_y_from_yindex(y_inx)
                cost = self.np_cmap_arr[x_inx][y_inx]
                name = str(x_world) +","+str(y_world)
                if cost > .001:
                    base_z = height_map.height_at_xy(x_world, y_world)
                    traj = trajectory.Trajectory(milestones=[[x_world, y_world, base_z+.001], [x_world, y_world, base_z+cost]])
                    vis.add(name, traj)
                    vis.hideLabel(name)

    def visualize(self, title=None):

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        if title:
            ax.set_title(title)
        else:
            ax.set_title('Footstep Cost Map')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        X = np.arange(self.x_start, self.x_end, self.x_granularity).transpose()
        Y = np.arange(self.y_start, self.y_end, self.y_granularity).transpose()
        X, Y = np.meshgrid(X,Y)
        Z = self.np_cmap_arr.transpose()
        surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        fig.colorbar(surf, shrink=0.5, aspect=5)
        plt.show()

    def print_stats(self):
        print("<CostMap Obj>")
        print(f"      failed:\t\t{self.failed}")
        print(f"      runtime:\t\t{round(self.runtime, 2)}")
        print(f"      x range:\t\t{round(self.x_start, 2)} - {round(self.x_end, 2)}")
        print(f"      y range:\t\t{round(self.y_start, 2)} - {round(self.y_end, 2)}")
        print(f"      np arr size:\t[{self.x_indices} {self.y_indices}]\n")