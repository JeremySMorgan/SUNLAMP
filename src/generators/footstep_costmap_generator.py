import time
import numpy as np
from src.utils.data_objects.footstep_cost_map import FootstepCostMap
from src.utils.data_objects.gradient_map import GradientMap
from src.utils.data_objects.height_map import HeightMap
from src.utils.logger import Logger
from src.utils.math_utils import MathUtils
from src.utils import project_constants

class FootstepCostMapGenerator:

    def __init__(self, heigh_map_obj: HeightMap, gradient_map_obj: GradientMap):

        # height map
        self.hm_obj = heigh_map_obj
        self.gradient_map = gradient_map_obj

        self.decimal_round = 5

        self.x_min = self.hm_obj.x_start
        self.x_max = self.hm_obj.x_end
        self.y_min = self.hm_obj.y_start
        self.y_max = self.hm_obj.y_end

        self.x_indices = int((self.hm_obj.x_end - self.hm_obj.x_start) / self.hm_obj.x_granularity)
        self.y_indices = int((self.hm_obj.y_end - self.hm_obj.y_start) / self.hm_obj.y_granularity)

        self.fs_slope_cost_arr_obj = None
        self.fs_cumm_err_cost_arr_obj = None

        self.fs_costmap = FootstepCostMap(self.hm_obj.x_vars, self.hm_obj.y_vars)

    def get_fs_cost_map(self):
        return self.fs_costmap

    # def normalize_cost_arr(self, debug=False):
    #     self.fs_costmap.normalize_cost_arr(project_constants.CMAP_NORMALIZED_MAX_VALUE, debug=debug)

    def build_costmap(self, debug=False, exlude_slope=False, exlude_roughness=False, exlude_step=False, normalize_cost_arr=True):
        '''
            returns runtime
        '''
        if debug:
            print("calculating footstep location costs")
        start_t = time.time()

        x_step_size_world = project_constants.CMAP_STEP_SIZEX
        y_step_size_world = project_constants.CMAP_STEP_SIZEY

        x_idxs_per_step = x_step_size_world / self.hm_obj.x_granularity
        y_idxs_per_step = y_step_size_world / self.hm_obj.y_granularity
        x_idxs_per_step_on2 = x_idxs_per_step/2.0
        y_idxs_per_step_on2 = x_idxs_per_step/2.0

        if not int(x_idxs_per_step) == x_idxs_per_step or not int(y_idxs_per_step) == y_idxs_per_step:
            Logger.log("Error: CMAP_STEP_SIZE(X/Y) doesnt resolve into discrete indexes. make sure the value is a multiple of (x/y)_granularity ","FAIL")
            return -1

        if not int(x_idxs_per_step_on2) == x_idxs_per_step_on2 or not int(y_idxs_per_step_on2) == y_idxs_per_step_on2:
            Logger.log("Error: CMAP_STEP_SIZE(X/Y)/2 doesnt resolve into discrete indexes. make sure the value is a multiple of 2*(x/y)_granularity","FAIL")
            return -1

        x_idxs_per_step = int(x_idxs_per_step)
        y_idxs_per_step = int(y_idxs_per_step)
        x_idxs_per_step_on2 = int(x_idxs_per_step_on2)
        y_idxs_per_step_on2 = int(y_idxs_per_step_on2)

        x_idx = int(x_idxs_per_step_on2)
        y_idx = int(y_idxs_per_step_on2)

        # discontinuity_degree_threshold - slopes greater than this will be considered discontinuities and ignored by this heuristic.

        x_idxs_in_step_fn_search_area = int(project_constants.CMAP_STEP_COSTFN_SEARCH_SIZE / self.hm_obj.x_granularity)
        y_idxs_in_step_fn_search_area = int(project_constants.CMAP_STEP_COSTFN_SEARCH_SIZE / self.hm_obj.y_granularity)

        hm_np_arr = self.hm_obj.get_np_hm_array()
        # grad_x_np_arr = self.gradient_map.get_np_x_gradient()
        # grad_y_np_arr = self.gradient_map.get_np_y_gradient()

        np.set_printoptions(precision=4)

        # this should cover whole search area
        while x_idx < self.x_indices:
            while y_idx < self.y_indices:

                # world_x = self.hm_obj.get_x_from_xindex(x_idx)
                # world_y = self.hm_obj.get_y_from_yindex(y_idx)

                x0 = x_idx - x_idxs_in_step_fn_search_area
                xf = x_idx + x_idxs_in_step_fn_search_area
                y0 = y_idx - y_idxs_in_step_fn_search_area
                yf = y_idx + y_idxs_in_step_fn_search_area
                if x0 < 0:
                    x0 = 0
                if xf >= self.x_indices:
                    xf = self.x_indices - 1
                if y0 < 0:
                    y0 = 0
                if yf >= self.y_indices:
                    yf = self.y_indices - 1

                hm_sub_arr = hm_np_arr[x0:xf, y0:yf]
                # grad_x_sub_arr = grad_x_np_arr[x0:xf, y0:yf]
                # grad_y_sub_arr = grad_x_np_arr[x0:xf, y0:yf]

                slope_cost = 0
                step_cost = 0
                roughness_cost = 0

                # _____ Slope Cost
                if not exlude_slope:

                    x_grad, y_grad = self.gradient_map.get_grad_at_xy_idx(x_idx, y_idx)
                    abs_x_slope_deg, abs_y_slope_deg = np.abs(MathUtils.gradient_to_slope_deg(x_grad)), np.abs(MathUtils.gradient_to_slope_deg(y_grad))

                    x_cost = abs_x_slope_deg - project_constants.CMAP_MAX_NONPENALIZED_SLOPE
                    y_cost = abs_y_slope_deg - project_constants.CMAP_MAX_NONPENALIZED_SLOPE

                    # Filter height map discontinuities and non penalized slopes
                    if abs_x_slope_deg > project_constants.CMAP_SLOPE_HEURISTIC_MAX_CONSIDERED_DEGREE or \
                            abs_x_slope_deg < project_constants.CMAP_MAX_NONPENALIZED_SLOPE:
                        x_cost = 0

                    if abs_y_slope_deg > project_constants.CMAP_SLOPE_HEURISTIC_MAX_CONSIDERED_DEGREE or \
                            abs_y_slope_deg < project_constants.CMAP_MAX_NONPENALIZED_SLOPE:
                        y_cost = 0

                    slope_cost = project_constants.CMAP_SLOPE_HEURISTIC_COEFF * (x_cost + y_cost) / 2

                    # if c > 0:
                    #     print(f" {round(world_x, 2)},  {round(world_y, 2)}, abs slope x: {round(abs_x_slope_deg,2)}\t y: {round(abs_y_slope_deg,2)}")

                # _____ Step Cost
                if not exlude_step:

                    max_dif = 0
                    for x_i in range(0, xf-x0):
                        y_col = hm_sub_arr[x_i,:]
                        if np.amax(np.abs(np.diff(y_col))) > max_dif:
                            max_dif = np.amax(np.abs(np.diff(y_col)))

                    for y_i in range(0, yf-y0):
                        x_row = hm_sub_arr[:,y_i]
                        if np.amax(np.abs(np.diff(x_row))) > max_dif:
                            max_dif = np.amax(np.abs(np.diff(x_row)))

                    if max_dif > project_constants.CMAP_STEP_COSTFN_MIN_HEIGHT_DIF:
                        step_cost = project_constants.CMAP_STEP_COSTFN_BASELINE_COST + project_constants.CMAP_STEP_COSTFN_DIF_SLOPE_COEFF * max_dif

                    # d = .025
                    # if np.abs(world_x - 5) < d and np.abs(world_y - 1.4) < d:
                    #     print("\n\n______________")
                    #     print("world x:", world_x)
                    #     print("world y:", world_y)
                    #     print("hm array:")
                    #     print(hm_sub_arr)
                    #     print("max_dif:",max_dif)
                    #     print("step_cost:", step_cost)
                    #     print(".CMAP_STEP_COSTFN_DIF_SLOPE_COEFF * max_dif: ", project_constants.CMAP_STEP_COSTFN_DIF_SLOPE_COEFF * max_dif)
                    #     print("CMAP_STEP_COSTFN_BASELINE_COST * max_dif: ", project_constants.CMAP_STEP_COSTFN_BASELINE_COST)

                # _____ Roughness Cost
                # 'The “roughness” of the location. A measure of the deviation of the surface from the fitted plane.
                # Computed by averaging the difference in height of each cell to the plane’s height at the that cell'
                # Quoted from "Navigation Planning for Legged Robots", Joel Chestnutt

                if not exlude_roughness:

                    if not np.max(hm_sub_arr) == np.min(hm_sub_arr) and step_cost == 0:

                        a, b, c, z0 = MathUtils.best_fitting_plane(hm_sub_arr)
                        specified_plane = MathUtils.hm_specified_by_abc_z0(a, b, c, z0, hm_sub_arr.shape)
                        mse = np.sum(np.abs(specified_plane - hm_sub_arr))
                        # normalized_mse = mse / (hm_sub_arr.shape[0] * hm_sub_arr.shape[1])
                        # roughness_cost = project_constants.CMAP_ROUGHNESS_HEURISTIC_COEFF * normalized_mse

                        roughness_cost = project_constants.CMAP_ROUGHNESS_HEURISTIC_COEFF * mse

                        # d = .025
                        # if np.abs(world_x - 5) < d and np.abs(world_y - .9) < d:
                        #     print("\n\n______________")
                        #     print("world x:", world_x)
                        #     print("world y:", world_y)
                        #     print("a:", round(a, 4))
                        #     print("b:", round(b, 4))
                        #     print("c:", round(c, 4))
                        #     print("z0:", round(z0, 4))
                        #     print("hm array:")
                        #     print(hm_sub_arr)
                        #     print("mse:",mse)
                        #     # print("normalized mse:", normalized_mse)
                        #     print("cost: ", roughness_cost)
                        #
                        # if np.abs(world_x - 4.25) < 2*d and np.abs(world_y - 1) < 2*d:
                        #     print("\n\n______________")
                        #     print("world x:", world_x)
                        #     print("world y:", world_y)
                        #     print("a:", round(a,4))
                        #     print("b:", round(b,4))
                        #     print("v:", round(c,4))
                        #     print("z0:", round(z0,4))
                        #     print("hm array:")
                        #     print(hm_sub_arr)
                        #     print("mse:",mse)
                        #     # print("normalized mse:", normalized_mse)
                        #     print("cost: ", roughness_cost)

                cost = slope_cost + step_cost + roughness_cost
                self.fs_costmap.np_cmap_arr[x_idx - x_idxs_per_step_on2:x_idx + x_idxs_per_step_on2,
                                                    y_idx-y_idxs_per_step_on2:y_idx+y_idxs_per_step_on2] = cost
                y_idx += y_idxs_per_step
            y_idx = y_idxs_per_step_on2
            x_idx += x_idxs_per_step

        if normalize_cost_arr:
            self.fs_costmap.normalize_cost_arr(project_constants.CMAP_NORMALIZED_MAX_VALUE, debug=debug)

        self.fs_costmap.runtime = time.time() - start_t
        self.fs_costmap.failed = False

        if debug:
            print("footstep cost map built in:",time.time()-start_t,"s")

        return self.fs_costmap

    def round(self, n):
        return np.round(n, decimals=self.decimal_round)

    def visualize_cost_array(self):
        self.fs_costmap.visualize()

    def return_fs_cost_map(self):
        return self.fs_costmap
