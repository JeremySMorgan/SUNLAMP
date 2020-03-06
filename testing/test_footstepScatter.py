from unittest import TestCase
import time
from src.generators.scatter_list_generator import ScatterListGenerator
from src.utils.data_objects.footstep_scatter import FootstepScatter


class TestFootstepScatter(TestCase):

    def test_get_idxs_near_xy(self):

        scatter = FootstepScatter()

        xmin = 2
        xmax = 5
        xdelta = .1
        x = xmin

        ymin = 1
        ymax = 3
        ydelta = .1
        y = ymin

        c = z = 0

        while x <= xmax:

            while y <= ymax+ydelta:

                scatter.append_to_scatter((x, y, c, z))
                y += ydelta

            y = ymin
            x += xdelta

        x0 = 3.05
        y0 = 2.025
        r = .225

        idxs_near_xy_r = scatter.get_idxs_near_xy(x0, y0, r)
        print()
        print(len(idxs_near_xy_r),"idxs")
        print(idxs_near_xy_r)
        print()

        scatter.print_scatter(only_bounds=True)
        # scatter.print_scatter(only_bounds=False)
        tree = scatter.idx_tree
        for x_key in tree:
            for y_key in tree[x_key]:

                if x0 - r < x_key < x0 + r and y0 - r < y_key < y0 + r:
                    idx = tree[x_key][y_key]
                    in_idxs = idx in idxs_near_xy_r
                    print(f" (inbound) idx[{x_key}, {y_key}]: {tree[x_key][y_key]}\tin idxs: {in_idxs}")




        time.sleep(.1)
        self.assertEqual(0, 0)

    def test_append_to_scatter(self):
        self.fail()
