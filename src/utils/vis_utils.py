from klampt import vis
from klampt.model import trajectory
import numpy as np


class VisUtils:

    BLUE = "b"
    ORANGE = "o"
    RED = "r"
    WHITE = "w"
    GREEN = "g"
    TURQ = "t"

    @staticmethod
    def visualize_dashed_line(xyz1, xyz2, name, dashes, color=None):
        delta_x = xyz2[0] - xyz1[0]
        delta_y = xyz2[1] - xyz1[1]
        delta_z = xyz2[2] - xyz1[2]

        dx = delta_x / (2.0 * dashes)
        dy = delta_y / (2.0 * dashes)
        dz = delta_z / (2.0 * dashes)

        for i in range(dashes):
            h = i * 2
            p1 = [xyz1[0] + h * dx, xyz1[1] + h * dy, xyz1[2] + h * dz]
            p2 = [xyz1[0] + (h + 1) * dx, xyz1[1] + (h + 1) * dy, xyz1[2] + (h + 1) * dz]
            traj = trajectory.Trajectory(milestones=[p1, p2])
            vis.add((name + str(h)), traj)
            if color:
                VisUtils.set_color(name + str(h), color)
            else:
                VisUtils.set_color(name + str(h), VisUtils.WHITE)

    @staticmethod
    def clear_visitems(names):
        for name in list(set(names)):
            vis.remove(name)

    @staticmethod
    def visualize_line(p1, p2, name="default_line", add_z=0, color=None, hm=None):
        if len(p1) < 3:
            p1 = [p1[0],p1[1],0]
        if len(p2) < 3:
            p2 = [p2[0], p2[1], 0]
        p1 = [p1[0], p1[1], p1[2] + add_z]
        p2 = [p2[0], p2[1], p2[2] + add_z]
        traj = trajectory.Trajectory(milestones=[p1[0:3], p2[0:3]])
        vis.add(name, traj)
        if color:
            VisUtils.set_color(name, color)

    @staticmethod
    def visualize_xyz_list(xyz, name="defaultname", height_map=None,loop=False, color=None):
        milestones = []
        for xy_y in xyz:
            x = xy_y[0]
            y = xy_y[1]
            if height_map:
                milestones.append([x, y, height_map.height_at_xy(x, y) + .05 ])
            else:
                milestones.append([x, y, 0])
        if loop:
            if height_map:
                milestones.append([xyz[0][0]+.0001, xyz[0][1]+.0001, height_map.height_at_xy(xyz[0][0], xyz[0][1]) + .05 ])
            else:
                milestones.append([xyz[0][0]+.0001, xyz[0][1]+.0001, .001])

        path = trajectory.Trajectory(milestones=milestones)
        vis.add(name, path)
        if color:
            VisUtils.set_color(name, color)

    @staticmethod
    def set_color(name, color, alpha=1):
        if color == VisUtils.BLUE:
            vis.setColor(name, 0, 0, 1, a=alpha)
        if color == VisUtils.RED:
            vis.setColor(name, 1, 0, 0, a=alpha)
        if color == VisUtils.WHITE:
            vis.setColor(name, 1, 1, 1, a=alpha)
        if color == VisUtils.GREEN:
            vis.setColor(name, 0, 1, 0, a=alpha)
        if color == VisUtils.ORANGE:
            vis.setColor(name, 1, 2/3, 0, a=alpha)
        if color == VisUtils.TURQ:
            vis.setColor(name, 0, .7, .07, a=alpha)

    @staticmethod
    def visualize_shapely_line(L, name="default_line", add_z=0, hm=None, color=None):
        '''
            L.xy Separate arrays of X and Y coordinate values
        :param L:
        :param name:
        :param add_z:
        :param hm:
        :param color:
        :return:
        '''
        x, y = L.xy
        p1 = [x[0], y[0], 0]
        p2 = [x[1], y[1], 0]
        VisUtils.visualize_line(p1, p2, name, color=color, add_z=add_z)

    @staticmethod
    def visualize_shapely_poly(poly, name="shapely_poly", add_z=0, color=None):
        x, y = poly.exterior.coords.xy
        xyz = []
        for i in range(len(x)):
            xyz.append([x[i], y[i], add_z])
        VisUtils.visualize_xyz_list(xyz, name=name, color=color)

    @staticmethod
    def visualize_cost(x,y,cost,name, with_z=0, hm=None, color=None, hide_label=False):
        traj = trajectory.Trajectory(milestones=[[x, y, with_z], [x, y, with_z+cost]])
        vis.add(name, traj)
        if color:
            VisUtils.set_color(name,color)
        if hide_label:
            vis.hideLabel(name)

    @staticmethod
    def visualize_xy_point_dashed(xy, name, height=.25, color=None, with_z=0, hm=None):
        VisUtils.visualize_dashed_line(xy+[0], xy+[height], name, 6, color=color)

    @staticmethod
    def visualize_xy_point(xy, name, height=.75, color=None, with_z=0, hm=None):
        VisUtils.visualize_cost(xy[0],xy[1], height, name, with_z=with_z, hm=hm)
        if color:
            VisUtils.set_color(name, color)
        else:
            VisUtils.set_color(name, VisUtils.WHITE)
            # vis.setColor(name, color[0], color[1], color[2], a=1.0)

    @staticmethod
    def visualize_q_holds(q, scatter_list):
        fl = scatter_list[q[0]]
        fr = scatter_list[q[1]]
        br = scatter_list[q[2]]
        bl = scatter_list[q[3]]
        VisUtils.visualize_cost(fl[0],fl[1],.45,name="front left",with_z=fl[2])
        VisUtils.visualize_cost(fr[0],fr[1],.45,name="front right",with_z=fr[2])
        VisUtils.visualize_cost(br[0],br[1],.45,name="back right",with_z=br[2])
        VisUtils.visualize_cost(bl[0],bl[1],.45,name="back left",with_z=bl[2])

    @staticmethod
    def visualize_circle(x0,y0,r, name="default_circle", hm=None, color=None, arc_step=15):
        milestones = []
        for i in range(0, 361, arc_step):
            x = x0 + r*np.cos(np.deg2rad(i))
            y = y0 + r*np.sin(np.deg2rad(i))
            if hm:
                z = hm.height_at_xy(x, y)
            else:
                z = 0
            milestones.append([x,y,z+.01])
        circle = trajectory.Trajectory(milestones=milestones)
        vis.add(name, circle)
        if color:
            VisUtils.set_color(name, color)
