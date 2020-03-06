import pickle as pickle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from .map_superclass import Map
from src.utils.math_utils import MathUtils


class GradientMap(Map):

    def __init__(self, height_map_obj):

        super(self.__class__, self).__init__()

        self.x_vars = height_map_obj.x_vars
        self.y_vars = height_map_obj.y_vars

        self.x_start = height_map_obj.x_vars[0]
        self.x_end = height_map_obj.x_vars[1]
        self.x_granularity = height_map_obj.x_vars[2]
        self.y_start = height_map_obj.y_vars[0]
        self.y_end = height_map_obj.y_vars[1]
        self.y_granularity = height_map_obj.y_vars[2]

        self.x_indices = int((self.x_end - self.x_start) / self.x_granularity)
        self.y_indices = int((self.y_end - self.y_start) / self.y_granularity)

        self.hm_x_gradient, self.hm_y_gradient = MathUtils.xy_gradient(height_map_obj.get_np_hm_array())
        self.hm_x_slope, self.hm_y_slope = MathUtils.gradient_to_slope_arr(self.hm_x_gradient), MathUtils.gradient_to_slope_arr(self.hm_y_gradient)

    def get_grad_at_world_xy(self,x,y):
        x_index = self.get_xindex_from_x(x)
        y_index = self.get_yindex_from_y(y)
        return self.hm_x_gradient[x_index][y_index], self.hm_y_gradient[x_index][y_index]

    def get_grad_at_xy_idx(self, x_index, y_index):
        return self.hm_x_gradient[x_index][y_index], self.hm_y_gradient[x_index][y_index]

    def get_np_x_gradient(self):
        return self.hm_x_gradient

    def get_np_y_gradient(self):
        return self.hm_y_gradient

    def get_np_x_slope(self):
        return self.hm_x_slope

    def get_np_y_slope(self):
        return self.hm_y_slope

    def visualize(self, title=None):

        fig1, fig2, fig3, fig4 = plt.figure(), plt.figure(2), plt.figure(3), plt.figure(4)
        ax1, ax2, ax3, ax4 = fig1.gca(projection='3d'),fig2.gca(projection='3d'),fig3.gca(projection='3d'),fig4.gca(projection='3d')

        X = np.arange(self.x_start, self.x_end, self.x_granularity).transpose()
        Y = np.arange(self.y_start, self.y_end, self.y_granularity).transpose()
        X, Y = np.meshgrid(X, Y)

        Z_grad_x, Z_grad_y = self.hm_x_gradient.transpose(), self.hm_y_gradient.transpose()
        Z_slope_x, Z_slope_y = self.hm_x_slope.transpose(), self.hm_y_slope.transpose()

        ax1.set_title('x gradient of height map')
        if title: ax1.set_title(title)
        ax1.set_zlabel('Z')
        ax1.set_ylabel('Y')
        ax1.set_xlabel('X')
        surf = ax1.plot_surface(X, Y, Z_grad_x, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        fig1.colorbar(surf, shrink=1, aspect=5)

        ax2.set_title('y gradient of height map')
        if title: ax2.set_title(title)
        ax2.set_zlabel('Z')
        ax2.set_ylabel('Y')
        ax2.set_xlabel('X')
        surf = ax2.plot_surface(X, Y, Z_grad_y, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        fig2.colorbar(surf, shrink=1, aspect=15)

        ax3.set_title('x (deg) of height map')
        if title: ax3.set_title(title)
        ax3.set_zlabel('Z')
        ax3.set_ylabel('Y')
        ax3.set_xlabel('X')
        surf = ax3.plot_surface(X, Y, Z_slope_x, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        fig3.colorbar(surf, shrink=1, aspect=5)

        ax4.set_title('y slope (deg) of height map')
        if title: ax4.set_title(title)
        ax4.set_zlabel('Z')
        ax4.set_ylabel('Y')
        ax4.set_xlabel('X')
        surf = ax4.plot_surface(X, Y, Z_slope_y, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        fig4.colorbar(surf, shrink=1, aspect=15)

        plt.show()

    def save(self, save_file, print_ = True):

        '''
                - Saves the height map to a numpy file and this object to a .pickle file
        :param save_file:
        :return: None
        '''

        #np.save((save_file+"_x_direction"), self.hm_x_gradient)
        #np.save((save_file+"_y_ydirection"), self.hm_y_gradient)
        pickle.dump(self, open(save_file+".pickle", "wb"))
        if print_:
            print("gradient map saved")
