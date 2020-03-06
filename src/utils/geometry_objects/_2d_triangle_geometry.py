from klampt.model import trajectory
from shapely.geometry.polygon import Polygon
import random
from klampt import vis
from .geometry_object_superclass import _3dGeometrySuperclass
from src.utils.math_utils import MathUtils
from shapely.geometry import LineString


class SupportTriangle(_3dGeometrySuperclass):

    # diag_point_idxs: [False, True, True]

    def __init__(self, P, height_map, diag_point_idxs:list, name=None, save_incenter=True):

        self.unchanged_points = P
        self.diag_point_idxs = diag_point_idxs
        self.points = []

        for p in P:
            self.points.append(self.make_2d(p))

        self.shapely_poly = Polygon(self.points)
        _3dGeometrySuperclass.__init__(self, height_map, name, self.shapely_poly)

        self.incenterx = None
        self.incentery = None
        self.incenterr = None

        self.P_diag1, self.P_diag2 = None, None
        self.update_diag_points()

        if save_incenter:
            self.save_incenter_xyr()

    def update_diag_points(self):

        if self.diag_point_idxs[0] and self.diag_point_idxs[1]:
            self.P_diag1, self.P_diag2 = self.points[0], self.points[1]

        elif self.diag_point_idxs[0] and self.diag_point_idxs[2]:
            self.P_diag1, self.P_diag2 = self.points[0], self.points[2]

        elif self.diag_point_idxs[1] and self.diag_point_idxs[2]:
            self.P_diag1, self.P_diag2 = self.points[1], self.points[2]
        else:
            print(f"ERROR: diag_point_idxs={self.diag_point_idxs} not valid")

    def get_diag_linestring(self) -> LineString:
        return LineString([self.P_diag1, self.P_diag2])

    def save_incenter_xyr(self):
        self.incenterx, self.incentery, self.incenterr = MathUtils.incenter_circle_xy_R(
            self.points[0],self.points[1],self.points[2])

    def enforce_safety_margin(self, scaling_factor):

        points = list(self.shapely_poly.exterior.coords)
        P1 = points[0]
        P2 = points[1]
        P3 = points[2]  # 0, 10
        V1 = MathUtils.get_vector_between_points(P1, P2)
        V1_neg = MathUtils.flip_vector(V1)
        V2 = MathUtils.get_vector_between_points(P2, P3)
        V2_neg = MathUtils.flip_vector(V2)
        V3 = MathUtils.get_vector_between_points(P3, P1)
        V3_neg = MathUtils.flip_vector(V3)
        v1_prime = MathUtils.scalar_multiply_vector( MathUtils.vector_adder(V1_neg, V3), scaling_factor)
        v2_prime = MathUtils.scalar_multiply_vector( MathUtils.vector_adder(V1, V2_neg), scaling_factor)
        v3_prime = MathUtils.scalar_multiply_vector( MathUtils.vector_adder(V2, V3_neg), scaling_factor)
        P_ret1 = MathUtils.add_scaled_vector_to_pt(P1, v1_prime, 1)
        P_ret2 = MathUtils.add_scaled_vector_to_pt(P2, v2_prime, 1)
        P_ret3 = MathUtils.add_scaled_vector_to_pt(P3, v3_prime, 1)
        self.points = [P_ret1, P_ret2, P_ret3]
        self.shapely_poly = Polygon(self.points)
        self.shapely_poly.exterior.coords =  self.points
        self.update_diag_points()
        self.save_incenter_xyr()

    def visualize(self, style="none", hide_label=True):

        if style == "none":
            milestones = []
            for p in list(self.shapely_poly.exterior.coords):
                z = self.height_map.height_at_xy(p[0],p[1]) + .01
                milestones.append([p[0],p[1],z])
            path = trajectory.Trajectory(milestones=milestones)
            if not self.name:
                self.name = "Support Triangle " + str(random.randint(1, 1000))
            vis.add(self.name, path)
            if hide_label:
                vis.hideLabel(self.name)

        elif style == "dashed":
            dashes = 10.0
            xyz_points = list(self.shapely_poly.exterior.coords)
            for i in range(len(xyz_points)):
                x, y = xyz_points[i][0], xyz_points[i][1]
                xyz_points[i] = [x, y, self.height_map.height_at_xy(x,y)+.01]

            self.visualize_dashed_line(xyz_points[0], xyz_points[1], "1-2", dashes)
            self.visualize_dashed_line(xyz_points[0], xyz_points[2], "1-3", dashes)
            self.visualize_dashed_line(xyz_points[1], xyz_points[2], "2-3", dashes)


    def visualize_dashed_line(self, xyz1, xyz2, name, dashes):

        delta_x = xyz2[0] - xyz1[0]
        delta_y = xyz2[1] - xyz1[1]
        delta_z = xyz2[2] - xyz1[2]

        dx = delta_x / (2.0*float(dashes))
        dy = delta_y / (2.0*float(dashes))
        dz = delta_z / (2.0*float(dashes))

        for i in range(int(dashes)):
            h=i*2
            p1 = [xyz1[0] + h*dx,     xyz1[1] + h*dy,     xyz1[2] + h*dz]
            p2 = [xyz1[0] + (h+1)*dx, xyz1[1] + (h+1)*dy, xyz1[2] + (h+1)*dz]
            traj = trajectory.Trajectory(milestones=[p1, p2])
            vis.add((name + str(h)), traj)