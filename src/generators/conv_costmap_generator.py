import time
import numpy as np
from scipy.ndimage.filters import convolve
# from src.utils.data_objects.convolution_cost_map import ConvolutionCostMap
from src.utils.data_objects.footstep_cost_map import FootstepCostMap
from src.utils import project_constants


class ConvolutionCostMapGenerator:

    def __init__(self, fs_cost_map_obj):

        self.fs_cost_map_obj: FootstepCostMap = fs_cost_map_obj

        self.decimal_round = 5

        self.x_min = self.fs_cost_map_obj.x_start
        self.x_max = self.fs_cost_map_obj.x_end
        self.y_min = self.fs_cost_map_obj.y_start
        self.y_max = self.fs_cost_map_obj.y_end

        self.x_indices = int((self.fs_cost_map_obj.x_end - self.fs_cost_map_obj.x_start) / self.fs_cost_map_obj.x_granularity)
        self.y_indices = int((self.fs_cost_map_obj.y_end - self.fs_cost_map_obj.y_start) / self.fs_cost_map_obj.y_granularity)

        # self.conv_cost_array_obj = ConvolutionCostMap(self.fs_cost_map_obj.x_vars, self.fs_cost_map_obj.y_vars)
        self.conv_cost_array_obj = FootstepCostMap(self.fs_cost_map_obj.x_vars, self.fs_cost_map_obj.y_vars)

    def build_conv_arr(self, method="min", normalize=True):

        print("calculating "+method+" conv fs costs")
        start_t = time.time()

        np_cost_arr = self.fs_cost_map_obj.get_fs_cost_map_np_arr()

        # print("max(np_cost_arr):", np.max(np_cost_arr))

        kernel_sizex = int((np_cost_arr.shape[0] * 2 * project_constants.CONV_X_WIDTH) / (self.x_max - self.x_min))
        kernel_sizey = int((np_cost_arr.shape[1] * 2 * project_constants.CONV_Y_WIDTH) / (self.y_max - self.y_min))

        # Both odd
        if kernel_sizex % 2 == 0:
            kernel_sizex += 1

        if kernel_sizey % 2 == 0:
            kernel_sizey += 1

        # print("kernel_sizex:",kernel_sizex)
        # print("x,y cell world width:",xcell_world_width,",",ycell_world_width,"\t x delta")
        # print("project_constants.CONV_Y_WIDTH:",project_constants.CONV_Y_WIDTH)
        # print("kernel_sizex:",kernel_sizex)
        kernlen = kernel_sizex

        # create nxn zeros
        # inp = np.zeros((kernlen, kernlen))
        # # set element at the middle to one, a dirac delta
        # inp[kernlen // 2, kernlen // 2] = 1
        # # gaussian-smooth the dirac, resulting in a gaussian filter mask
        # kernal = gaussian_filter(inp, 1)

        ax = np.arange(-kernlen // 2 + 1., kernlen // 2 + 1.)
        xx, yy = np.meshgrid(ax, ax)
        kernel = np.exp(-0.5 * (np.square(xx) + np.square(yy)) / np.square(10))

        # kernal = np.ones((35,35))
        # kernal = np.array([ [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #                     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #                     [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
        #                     [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
        #                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        #                     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        #                     [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        #                     [0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
        #                     [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
        #                     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #                     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
        #                   ])

        self.conv_cost_array_obj.np_cmap_arr = convolve(np_cost_arr, kernel, mode='constant', cval=0.0)

        # for x_idx in range(np_cost_arr.shape[0]):
        #     for y_idx in range(np_cost_arr.shape[1]):
        #

                # # Find x min, max indexs
                # if x_idx-kernel_sizex < 0:
                #     x_min_idx = 0
                # else:
                #     x_min_idx = x_idx - kernel_sizex
                #
                # if x_idx+kernel_sizex >= np_cost_arr.shape[0]:
                #     x_max_idx = np_cost_arr.shape[0]
                # else:
                #     x_max_idx = x_idx + kernel_sizex
                #
                # # Find y min, max indexs
                # if y_idx - kernal_sizey < 0:
                #     y_min_idx = 0
                # else:
                #     y_min_idx = y_idx - kernal_sizey
                #
                # if y_idx+kernal_sizey >= np_cost_arr.shape[1]:
                #     y_max_idx = np_cost_arr.shape[1]
                # else:
                #     y_max_idx = y_idx + kernal_sizey
                #
                # x_max_idx = int(x_max_idx)
                # x_min_idx = int(x_min_idx)
                # y_max_idx = int(y_max_idx)
                # y_min_idx = int(y_min_idx)

                # # create submatrix
                # subC = np_cost_arr[x_min_idx:x_max_idx,y_min_idx:y_max_idx]
                #
                # val = 0
                # if method == "min":
                #     val = np.amin(subC)
                # elif method == "min":
                #     val = np.amax(subC)
                # elif method == "ave":
                #     val = np.average(subC)
                # elif method == "none":
                #     val = np_cost_arr[x_idx][y_idx]
                # else:
                #     logger.log("Error, convolution type unrecognized","FAIL")
                # if val < 0:
                #     val = 0
                # np_minconv_arr.itemset((x_idx, y_idx), val)

        # print "Min conv footstep cost map built in:",time.time()-start_t,"s"

        if normalize:
            self.conv_cost_array_obj.normalize_cost_arr(project_constants.CMAP_NORMALIZED_MAX_VALUE)
        self.conv_cost_array_obj.runtime = time.time() - start_t
        self.conv_cost_array_obj.failed = False
        return self.conv_cost_array_obj

    def return_cost_arr(self):
        return self.conv_cost_array_obj

    # def normalize_cost_arr(self):
    #     self.conv_cost_array_obj.normalize_cost_arr(project_constants.CMAP_NORMALIZED_MAX_VALUE)

    def get_xy_start_finals(self, x, y, xradius, yradius):
        x_0 = x - xradius
        x_f = x + xradius
        y_0 = y - yradius
        y_f = y + yradius
        if x_0 < self.x_min:
            x_0 = self.x_min
        if x_f > self.x_max:
            x_f = self.x_max
        if y_0 < self.y_min:
            y_0 = self.y_min
        if y_f > self.y_max:
            y_f = self.y_max
        return x_0, x_f, y_0, y_f

    def round(self, n):
        return np.round(n, decimals=self.decimal_round)

    def save(self, save_file):
        self.conv_cost_array_obj.save(save_file)
