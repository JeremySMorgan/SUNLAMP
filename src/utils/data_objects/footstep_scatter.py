import pickle as pickle
import numpy as np
from klampt.model import trajectory
from src.utils.logger import Logger
from src.utils.data_objects.output_superclass import OutputSuperclass


class FootstepScatter(OutputSuperclass):

    def __init__(self):

        OutputSuperclass.__init__(self, "footstep scatter")

        self.scatter = []
        self.idx_tree = {}

        self.min_x = np.inf
        self.max_x = -np.inf
        self.min_y = np.inf
        self.max_y = -np.inf
        self.precision = 3

    def nb_points(self):
        return len(self.scatter)

    def get_idxs_near_xy(self, x: float, y: float, r: float):

        xmin = round(max(x-r, self.min_x), self.precision)
        xmax = round(min(x+r, self.max_x), self.precision)
        ymin = round(max(y-r, self.min_y), self.precision)
        ymax = round(min(y+r, self.max_y), self.precision)

        inc = 1/(10 ** self.precision)
        xmin_query_val = round(xmin, self.precision)
        if xmin_query_val not in self.idx_tree:
            x_i = 0
            while 1:
                xmin_query_val = round(xmin + x_i * inc, self.precision)
                if xmin_query_val in self.idx_tree:
                    break
                x_i += 1
                if x_i > 10**self.precision:
                    print("could not find xmin querry value")
                    return []

        xmax_query_val = round(xmax, self.precision)
        if xmax_query_val not in self.idx_tree:
            x_i = 0
            while 1:
                xmax_query_val = round(xmax - x_i * inc, self.precision)
                if xmax_query_val in self.idx_tree:
                    break
                x_i += 1
                if x_i > 10**self.precision:
                    print("could not find xmax querry value")
                    return []

        # Get y query vals
        ymin_query_val = round(ymin, self.precision)
        if ymin_query_val not in self.idx_tree[xmin_query_val]:
            x_i = 0
            while 1:
                ymin_query_val = round(ymin + x_i * inc, self.precision)
                if ymin_query_val in self.idx_tree[xmin_query_val]:
                    break
                x_i += 1
                if x_i > 10**self.precision:
                    print("could not find ymin querry value")
                    return []

        ymax_query_val = round(ymax, self.precision)
        if ymax_query_val not in self.idx_tree[xmin_query_val]:
            x_i = 0
            while 1:
                ymax_query_val = round(ymax + x_i * inc, self.precision)
                if ymax_query_val in self.idx_tree[xmin_query_val]:
                    break
                x_i += 1
                if x_i > 10**self.precision:
                    print("could not find ymax querry value")
                    return []

        xmin_ymin_idx = self.idx_tree[xmin_query_val][ymin_query_val]
        xmin_ymax_idx = self.idx_tree[xmin_query_val][ymax_query_val]
        xmax_ymin_idx = self.idx_tree[xmax_query_val][ymin_query_val]
        xmax_ymax_idx = self.idx_tree[xmax_query_val][ymax_query_val]

        min_idx = min(xmin_ymin_idx, xmin_ymax_idx, xmax_ymin_idx, xmax_ymax_idx)
        max_idx = max(xmin_ymin_idx, xmin_ymax_idx, xmax_ymin_idx, xmax_ymax_idx)

        return [i for i in range(min_idx, max_idx)]

    def append_to_scatter(self, xyz_c, add_to_idx_tree=True):

        self.scatter.append(xyz_c)

        if add_to_idx_tree:
            world_x = round(xyz_c[0], self.precision)
            world_y = round(xyz_c[1], self.precision)

            self.min_x = min(world_x, self.min_x)
            self.max_x = max(world_x, self.max_x)
            self.min_y = min(world_y, self.min_y)
            self.max_y = max(world_y, self.max_y)

            if world_x not in self.idx_tree:
                self.idx_tree[world_x] = {}

            self.idx_tree[world_x][world_y] = len(self.scatter) - 1

    def visualize_in_klampt(self):

        from klampt import vis

        height = .25

        for xyz in self.scatter:
            name = f"{xyz[0]}{xyz[1]}{xyz[2]}"
            traj = trajectory.Trajectory(milestones=[[xyz[0], xyz[1], xyz[2] + .001], [xyz[0], xyz[1], xyz[2] + height]])
            vis.add(name, traj)
            vis.hideLabel(name)

        for i in range(len(self.scatter)-4, len(self.scatter)):
            xyz = self.scatter[i]
            name = f"{xyz[0]}{xyz[1]}{xyz[2]}_2"
            traj = trajectory.Trajectory(milestones=[[xyz[0], xyz[1], xyz[2] + .001], [xyz[0], xyz[1], xyz[2] + 1]])
            vis.add(name, traj)
            vis.setColor(name, 1, 0, 0, 1)

    def get_scatter_list(self):
        return self.scatter

    def get_scatter_tree(self):
        return self.idx_tree

    def print_stats(self):
        print("<FS_Scatter Obj>")
        print(f"      failed:\t\t{self.failed}")
        print(f"      runtime:\t\t{round(self.runtime,2)}")
        print(f"      min_x:\t\t{round(self.min_x, 2)}")
        print(f"      max_x:\t\t{round(self.max_x, 2)}")
        print(f"      min_y:\t\t{round(self.min_y, 2)}")
        print(f"      max_y:\t\t{round(self.max_y, 2)}\n")
