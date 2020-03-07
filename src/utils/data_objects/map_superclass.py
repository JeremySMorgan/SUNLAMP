from scipy.interpolate import interp1d
import time
import numpy as np

class Map:

    def __init__(self):

        self.x_start = None
        self.x_end = None
        self.x_granularity = None
        self.y_start = None
        self.y_end = None
        self.x_granularity = None

        self.x_indices = None
        self.y_indices = None

        self.x_vars = None
        self.y_vars = None
        self.decimal_round = 6

        self.np_arr = None

    def get_x_from_xindex(self, x_index):
        m = interp1d([0, self.x_indices], [self.x_start,self.x_end])
        return float(m(x_index))

    def get_y_from_yindex(self, y_index):
        m = interp1d([0, self.y_indices], [self.y_start, self.y_end])
        return float(m(y_index))

    def get_xindex_from_x(self,x):
        m = interp1d( [self.x_start, self.x_end], [0, self.x_indices])
        return int(m(x))

    def get_yindex_from_y(self, y):
        m = interp1d([self.y_start, self.y_end], [0, self.y_indices])
        return int(m(y))

    def xy_inbound(self, x, y):
        return self.x_start <= x <= self.x_end and self.y_start <= y <= self.y_end

    def max_in_radius_r_centered_at_xy(self, x_world: float, y_world: float, r_world: float):

        x_idx = self.get_xindex_from_x(x_world)
        y_idx = self.get_yindex_from_y(y_world)
        r_idxs = int(r_world / self.x_granularity)

        subarr = self.np_arr[x_idx - r_idxs:x_idx + r_idxs, y_idx - r_idxs:y_idx + r_idxs]
        max_ = -np.inf

        for x in range(subarr.shape[0]):
            for y in range(subarr.shape[1]):
                if np.sqrt((x - r_idxs) ** 2 + (y - r_idxs) ** 2) < r_idxs:
                    if subarr[x, y] > max_:
                        max_ = subarr[x, y]

        return max_

    def get_xy_start_finals(self, x, y, xradius, yradius):

        x_0 = x - xradius
        x_f = x + xradius
        y_0 = y - yradius
        y_f = y + yradius

        if x_0 < self.x_start:
            x_0 = self.x_start
        if x_f > self.x_end:
            x_f = self.x_end
        if y_0 < self.y_start:
            y_0 = self.y_start
        if y_f > self.y_end:
            y_f = self.y_end
        return x_0, x_f, y_0, y_f

    def visualize(self):
        pass

    def __str__(self):
        print_str = "<Map Object>\n"
        print_str += f"   x range: {round(self.x_start, 2)} - {round(self.x_end, 2)}\n"
        print_str += f"   y range: {round(self.y_start, 2)} - {round(self.y_end, 2)}\n"
        print_str += f"   numpy array size: [{self.x_indices} {self.y_indices}]\n"
        return print_str
