import time
from scipy import optimize
from klampt import vis
from shapely.geometry import LineString
import numpy as np
from klampt.math import so3
from klampt.model.collide import WorldCollider
from src.utils.logger import Logger
from src.utils.geometry_objects._2d_circle_geometry import LegRange
from src.utils.geometry_objects._2d_triangle_geometry import SupportTriangle
from shapely.geometry import Point
from src.utils.math_utils import MathUtils
from src.utils.vis_utils import VisUtils
from src.utils.data_objects.height_map import HeightMap
from src.utils import config

class RobotPose:

    def __init__(self, fl, fr, br, bl, torso_xyz_yawdeg):
        self.fl, self.fr, self.br, self.bl, self.torso_xyz_yawdeg = fl, fr, br, bl, torso_xyz_yawdeg

    def update_end_effectors(self,fl, fr, br, bl):
        self.fl, self.fr, self.br, self.bl = fl, fr, br, bl

    def get_moving_leg_xyzR(self,moving_leg):
        if moving_leg == 1: return self.fl
        if moving_leg == 2: return self.fr
        if moving_leg == 3: return self.br
        if moving_leg == 4: return self.bl

    def update_moving_legs_xyz(self, moving_leg, xyz):
        if moving_leg == 1: self.fl = xyz
        if moving_leg == 2: self.fr = xyz
        if moving_leg == 3: self.br = xyz
        if moving_leg == 4: self.bl = xyz

    def update_torso_xyz_yaw_deg(self,torso_xyz_yawdeg):
        self.torso_xyz_yawdeg = torso_xyz_yawdeg

    def debug(self):
        print("\nRobotPose debug")
        VisUtils.visualize_cost(self.fl[0],self.fl[1],.25,"front left")
        VisUtils.visualize_cost(self.fr[0],self.fr[1],.25,"front right")
        VisUtils.visualize_cost(self.br[0],self.br[1],.25,"back right")
        VisUtils.visualize_cost(self.bl[0],self.bl[1],.25,"back left")
        VisUtils.visualize_cost(self.torso_xyz_yawdeg[0], self.torso_xyz_yawdeg[1], .25, "torso",with_z=self.torso_xyz_yawdeg[2])
        print("  Front Left:",Logger.pp_list(self.fl), "len:",len(self.fl))
        print("  Front Right:",Logger.pp_list(self.fr), "len:",len(self.fr))
        print("  Back Right:",Logger.pp_list(self.br), "len:",len(self.br))
        print("  Back Left:",Logger.pp_list(self.bl), "len:",len(self.bl))
        print("  Torso:",Logger.pp_list(self.torso_xyz_yawdeg),"\n")


class Constraints:

    def __init__(self, support_triangles, range_circles):
        self.support_triangles = support_triangles
        self.range_circles = range_circles

    def get_xy_centroid(self):
        if len(self.support_triangles) >= 1:
            if len(self.support_triangles) >=2:
                o_support_tris = self.support_triangles[1:]
                o_3d_objs = o_support_tris + self.range_circles
                return self.support_triangles[0].xy_centroid_from_o_3dgeoms(o_3d_objs)
            else:
                return self.support_triangles[0].xy_centroid_from_o_3dgeoms(self.range_circles)

    def xy_in_support_area(self, x, y, ignore_range_tris=False):
        if ignore_range_tris:
            if len(self.support_triangles) == 1:
                return self.support_triangles[0].point_is_inside([x, y])
            else:
                Logger.log("Multiple support triangle point inside detection unimplemented, exiting", "FAIL")
                return "this should throw an error"
        support_objs = self.get_all_objects()
        return support_objs[0].point_is_inside_o_3d_geoms([x,y,.25], support_objs)

    def number_constraints(self):
        return len(self.support_triangles) + len(self.range_circles)

    def debug(self):
        print("\n",len(self.support_triangles),"support triangles,",len(self.range_circles),"range circles")
        for i in range(len(self.support_triangles)):
            print(" support tri",i,":")
        for i in range(len(self.range_circles)):
            print(" range_circles", i," has centroid at:",Logger.pp_list([self.range_circles[i].x,self.range_circles[i].y]))

    def get_all_objects(self):
        ret = []
        for tri in self.support_triangles:
            ret.append(tri)
        for rcircle in self.range_circles:
            ret.append(rcircle)
        return ret

    def get_support_triangles(self):
        return self.support_triangles

    def get_range_circles(self):
        return self.range_circles

    def visualize_support_tri(self):
        for support_tri in self.support_triangles:
            support_tri.visualize()

    def remove_support_tri_visualization(self):
        for support_tri in self.support_triangles:
            support_tri.remove_visualization()

    def visualize_range_circles(self):
        for circle in self.range_circles:
            circle.visualize()

    def remove_range_circle_visualizations(self):
        for circle in self.range_circles:
            circle.remove_visualization()

    def visualize_all(self, hm=None):
        self.visualize_range_circles()
        self.visualize_support_tri()

    def remove_all_visualizations(self):
        self.remove_range_circle_visualizations()
        self.remove_support_tri_visualization()


class MotionUtils:

    def __init__( self, world, height_map: HeightMap, scatter_list, state_path, gradient_map, u_input=None,
            include_ik_solver=False, lidar_mode=False):
        
        self.u_input = u_input
        self.gradient_map = gradient_map
        self.scatter_list = scatter_list
        self.stance_path = state_path
        self.world = world
        self.robosimian = world.robot(0)
        self.bl_end_effector = self.robosimian.link(config.BL_ACTIVE_DOFS[len(config.BL_ACTIVE_DOFS) - 1])
        self.br_end_effector = self.robosimian.link(config.BR_ACTIVE_DOFS[len(config.BR_ACTIVE_DOFS) - 1])
        self.fl_end_effector = self.robosimian.link(config.FL_ACTIVE_DOFS[len(config.FL_ACTIVE_DOFS) - 1])
        self.fr_end_effector = self.robosimian.link(config.FR_ACTIVE_DOFS[len(config.FR_ACTIVE_DOFS) - 1])
        self.torso = self.robosimian.link(config.TORSO_LINK_INDEX)
        self.height_map = height_map
        if include_ik_solver:
            from .ik_solver_utils import IKSolverUtils
            self.collider = WorldCollider(self.world)
            self.IKSolverUtil = IKSolverUtils(world, height_map, self.scatter_list, self.stance_path, gradient_map)

    def countdown(self,t):
        for i in range(t):
            i = t-i
            for j in range(i):
                print(".", end=' ')
            print()
            time.sleep(1)

    def debug_visualize_stance(self, stance_idx, constraint_obj: Constraints, visualize=False, debug=False):

        stance_state = self.stance_path[stance_idx]
        try:
            stance_state_next = self.stance_path[stance_idx + 1]

        except IndexError:
            # Logger.log("error inbound","FAIL")
            stance_state_next = None

        fl_xyz, fr_xyz, br_xyz, bl_xyz = self.get_end_affector_xyzs_from_curr_stance(stance_state)
        try:
            fl_xyz_next, fr_xyz_next, br_xyz_next, bl_xyz_next = self.get_end_affector_xyzs_from_curr_stance(stance_state_next)
        except TypeError:
            return

        moving_leg = self.get_moving_leg_from_stance_idx(stance_idx)

        if debug:
            print(f"stance idx:{stance_idx}/{len(self.stance_path)} stance_state:", self.stance_path[stance_idx], f"\t\ttorso xy: {Logger.pp_list(self.robosimian.getConfig()[0:3])}")
            print()

        # msg = "Step distance for fl end effector: " + Logger.pp_double(
        #     MathUtils._3d_euclidian_distance(self.fl_end_effector.getWorldPosition([0, 0, 0]), xyzR_next[0:3]))
        # msg = "Step distance for fr end effector: " + Logger.pp_double(MathUtils._3d_euclidian_distance(
        #     self.fr_end_effector.getWorldPosition([0, 0, 0]), xyzR_next[0:3]))
        # msg = "Step distance for br end effector: " + Logger.pp_double(
        #     MathUtils._3d_euclidian_distance(self.br_end_effector.getWorldPosition([0, 0, 0]), xyzR_next[0:3]))
        # msg = "Step distance for bl end effector: " + Logger.pp_double(MathUtils._3d_euclidian_distance(
        #     self.bl_end_effector.getWorldPosition([0, 0, 0]), xyzR_next[0:3]))

        # debugging
        diag_dist_2d = -1
        diag_dist_3d = -1
        # tri = #[fl_xyz, fr_xyz, br_xyz, bl_xyz]
        tri = constraint_obj.get_support_triangles()[0]
        if moving_leg == 1:
            diag_dist_2d = MathUtils._2d_euclidian_distance(fl_xyz, br_xyz)
            diag_dist_3d = MathUtils._3d_euclidian_distance(fl_xyz, br_xyz)
            # tri.remove(fl_xyz)
        elif moving_leg == 2:
            # tri.remove(fr_xyz)
            diag_dist_2d = MathUtils._2d_euclidian_distance(fr_xyz, bl_xyz)
            diag_dist_3d = MathUtils._3d_euclidian_distance(fr_xyz, bl_xyz)
        elif moving_leg == 3:
            # tri.remove(br_xyz)
            diag_dist_2d = MathUtils._2d_euclidian_distance(br_xyz, fl_xyz)
            diag_dist_3d = MathUtils._3d_euclidian_distance(br_xyz, fl_xyz)
        elif moving_leg == 4:
            # tri.remove(bl_xyz)
            diag_dist_2d = MathUtils._2d_euclidian_distance(bl_xyz, fr_xyz)
            diag_dist_3d = MathUtils._3d_euclidian_distance(bl_xyz, fr_xyz)

        # inscribed_circX, inscribed_circY, inscribed_circR = MathUtils.incenter_circle_xy_R_fromT(tri)
        inscribed_circX, inscribed_circY, inscribed_circR = tri.incenterx, tri.incentery, tri.incenterr
        inscribed_circZ = self.height_map.height_at_xy(inscribed_circX, inscribed_circY)

        moving_leg_to_com_dist_curr_2d = -1
        moving_leg_to_com_dist_next_2d = -1
        moving_leg_to_com_dist_curr_3d = -1
        moving_leg_to_com_dist_next_3d = -1

        com_diag_line = tri.get_diag_linestring()

        if moving_leg == 1:

            # com_diag_line = LineString([bl_xyz[0:2], fr_xyz[0:2]])
            moving_leg_to_incenter_line = LineString([fl_xyz[0:2], [inscribed_circX, inscribed_circY]])

            intersection_pt = com_diag_line.intersection(moving_leg_to_incenter_line)
            intersection_pt_xyz = [intersection_pt.x, intersection_pt.y, self.height_map.height_at_xy(intersection_pt.x, intersection_pt.y)]

            moving_leg_to_com_dist_curr_2d = MathUtils._2d_euclidian_distance(fl_xyz, [intersection_pt.x, intersection_pt.y])
            moving_leg_to_com_dist_curr_3d = MathUtils._3d_euclidian_distance(fl_xyz, intersection_pt_xyz)

            moving_leg_to_incenter_line_next = LineString([fl_xyz_next[0:2], [inscribed_circX, inscribed_circY]])
            intersection_pt_next = com_diag_line.intersection(moving_leg_to_incenter_line_next)
            intersection_pt_next_xyz =  [intersection_pt_next.x, intersection_pt_next.y,
                                         self.height_map.height_at_xy(intersection_pt_next.x, intersection_pt_next.y)]

            moving_leg_to_com_dist_next_2d = MathUtils._2d_euclidian_distance(fl_xyz_next, [intersection_pt_next.x, intersection_pt_next.y])
            moving_leg_to_com_dist_next_3d = MathUtils._3d_euclidian_distance(fl_xyz_next, intersection_pt_next_xyz)

            if visualize:
                VisUtils.visualize_line(fl_xyz, [intersection_pt.x, intersection_pt.y], name="daig_com_dist")
                VisUtils.visualize_line(fl_xyz_next, [intersection_pt_next.x, intersection_pt_next.y], name="daig_com_dist_next")
                VisUtils.visualize_line(fl_xyz_next, intersection_pt_next_xyz, name="D1_3d")

        elif moving_leg == 2:
            # com_diag_line = LineString([fl_xyz[0:2], br_xyz[0:2]])
            moving_leg_to_incenter_line = LineString([fr_xyz[0:2], [inscribed_circX, inscribed_circY]])

            intersection_pt = com_diag_line.intersection(moving_leg_to_incenter_line)
            intersection_ptxy = [intersection_pt.x, intersection_pt.y]
            intersection_pt_xyz = [intersection_pt.x, intersection_pt.y, self.height_map.height_at_xy(intersection_pt.x, intersection_pt.y)]

            moving_leg_to_com_dist_curr_2d = MathUtils._2d_euclidian_distance(fr_xyz, intersection_ptxy)
            moving_leg_to_com_dist_curr_3d = MathUtils._3d_euclidian_distance(fr_xyz, intersection_pt_xyz)

            moving_leg_to_incenter_line_next = LineString([fr_xyz_next[0:2], [inscribed_circX, inscribed_circY]])
            intersection_pt_next = com_diag_line.intersection(moving_leg_to_incenter_line_next)
            intersection_pt_next_xyz = [intersection_pt_next.x, intersection_pt_next.y,
                                        self.height_map.height_at_xy(intersection_pt_next.x, intersection_pt_next.y)]

            moving_leg_to_com_dist_next_2d = MathUtils._2d_euclidian_distance(fr_xyz_next,  [intersection_pt_next.x, intersection_pt_next.y])
            moving_leg_to_com_dist_next_3d = MathUtils._3d_euclidian_distance(fr_xyz_next,  intersection_pt_next_xyz)

            if visualize:
                VisUtils.visualize_line(fr_xyz, intersection_ptxy, name="daig_com_dist")
                VisUtils.visualize_line(fr_xyz_next, [intersection_pt_next.x, intersection_pt_next.y], name="daig_com_dist_next")
                VisUtils.visualize_line(fr_xyz_next, intersection_pt_next_xyz, name="D1_3d")

        elif moving_leg == 3:

            # com_diag_line = LineString([bl_xyz[0:2], fr_xyz[0:2]])
            moving_leg_to_incenter_line = LineString([br_xyz[0:2], [inscribed_circX, inscribed_circY]])
            intersection_pt = com_diag_line.intersection(moving_leg_to_incenter_line)
            intersection_pt_xyz = [intersection_pt.x, intersection_pt.y, self.height_map.height_at_xy(intersection_pt.x, intersection_pt.y)]

            moving_leg_to_com_dist_curr_2d = MathUtils._2d_euclidian_distance(br_xyz, [intersection_pt.x, intersection_pt.y])
            moving_leg_to_com_dist_curr_3d = MathUtils._3d_euclidian_distance(br_xyz, intersection_pt_xyz)

            moving_leg_to_incenter_line_next = LineString([br_xyz_next[0:2], [inscribed_circX, inscribed_circY]])
            intersection_pt_next = com_diag_line.intersection(moving_leg_to_incenter_line_next)
            intersection_pt_next_xyz = [intersection_pt_next.x, intersection_pt_next.y,
                                        self.height_map.height_at_xy(intersection_pt_next.x, intersection_pt_next.y)]

            moving_leg_to_com_dist_next_2d = MathUtils._2d_euclidian_distance(br_xyz_next, [intersection_pt_next.x, intersection_pt_next.y])
            moving_leg_to_com_dist_next_3d = MathUtils._3d_euclidian_distance(br_xyz_next, intersection_pt_next_xyz)

            if visualize:
                VisUtils.visualize_line(br_xyz, [intersection_pt.x, intersection_pt.y], name="daig_com_dist")
                VisUtils.visualize_line(br_xyz_next, [intersection_pt_next.x, intersection_pt_next.y], name="daig_com_dist_next")
                VisUtils.visualize_line(br_xyz_next, intersection_pt_next_xyz, name="D1_3d")

        elif moving_leg == 4:
            # com_diag_line = LineString([fl_xyz[0:2], br_xyz[0:2]])
            moving_leg_to_incenter_line = LineString([bl_xyz[0:2], [inscribed_circX, inscribed_circY]])
            intersection_pt = com_diag_line.intersection(moving_leg_to_incenter_line)
            intersection_pt_xyz = [intersection_pt.x, intersection_pt.y, self.height_map.height_at_xy(intersection_pt.x, intersection_pt.y)]

            moving_leg_to_com_dist_curr_2d = MathUtils._2d_euclidian_distance(bl_xyz, [intersection_pt.x, intersection_pt.y])
            moving_leg_to_com_dist_curr_3d = MathUtils._3d_euclidian_distance(bl_xyz, intersection_pt_xyz)

            moving_leg_to_incenter_line_next = LineString([bl_xyz_next[0:2], [inscribed_circX, inscribed_circY]])
            intersection_pt_next = com_diag_line.intersection(moving_leg_to_incenter_line_next)
            intersection_pt_next_xyz = [intersection_pt_next.x, intersection_pt_next.y,
                                        self.height_map.height_at_xy(intersection_pt_next.x, intersection_pt_next.y)]

            moving_leg_to_com_dist_next_2d = MathUtils._2d_euclidian_distance(br_xyz_next, [intersection_pt_next.x, intersection_pt_next.y])
            moving_leg_to_com_dist_next_3d = MathUtils._3d_euclidian_distance(br_xyz_next, intersection_pt_next_xyz)

            if visualize:
                VisUtils.visualize_line(bl_xyz, [intersection_pt.x, intersection_pt.y], name="daig_com_dist")
                VisUtils.visualize_line(bl_xyz_next, [intersection_pt_next.x, intersection_pt_next.y], name="daig_com_dist_next")
                VisUtils.visualize_line(bl_xyz_next, intersection_pt_next_xyz, name="D1_3d")

        if visualize:
            vis.setColor("daig_com_dist", 1, 0, 0, a=1.0)
            VisUtils.visualize_circle(inscribed_circX, inscribed_circY, inscribed_circR, "inscribed_circle", hm=self.height_map)
            vis.setColor("D1_3d", 1, 0, 1, a=1.0)

        if debug:
            print(
                f"moving leg: {moving_leg} \tincenter r: {round(inscribed_circR, 3)}"
                # f"  2d_diag dist: {round(diag_dist_2d, 3)} \n"
                f"  3d_diag dist: {round(diag_dist_3d, 3)} \n"
                # f"  moving_leg_to_com_dist_curr_2d: {round(moving_leg_to_com_dist_curr_2d, 3)}\n"
                f"  moving_leg_to_com_dist_curr_3d: {round(moving_leg_to_com_dist_curr_3d, 3)}\n"
                # f"  moving_leg_to_com_dist_next_2d: {round(moving_leg_to_com_dist_next_2d, 3)}\n"
                f"  moving_leg_to_com_dist_next_3d: {round(moving_leg_to_com_dist_next_3d, 3)}")
            #
            # if moving_leg_to_com_dist_curr_3d > moving_leg_to_com_dist_curr_2d:
            #     print(f"moving_leg_to_com_dist_curr_3d > moving_leg_to_com_dist_curr_2d: {round(moving_leg_to_com_dist_curr_3d,4)} > {round(moving_leg_to_com_dist_curr_2d,4)}")
            #
            # if moving_leg_to_com_dist_next_3d > moving_leg_to_com_dist_next_2d:
            #     print(f"moving_leg_to_com_dist_next_3d > moving_leg_to_com_dist_next_2d: {round(moving_leg_to_com_dist_next_3d,4)} > {round(moving_leg_to_com_dist_next_2d,4)}")

    def skip_to_stance(self, stance_idx):
        self.j = 1
        self.i = 1
        self.torso_inside_new_support_tri = False
        self.curr_stance_idx = stance_idx
        self.curr_stance = self.stance_path[self.curr_stance_idx]
        self.robot_pose = self.get_robot_pose_from_stance(self.curr_stance, with_end_effector_Rs=True)
        self.IKSolverUtil.set_pose_w_R(self.robot_pose)

    def manual_torso_control(self):
        c = .0025
        while not self.u_input.exit():
            self.test_for_collisions(self.collider)
            dx, dy = self.u_input.get_direction(c)
            self.update_torso_com_line()
            torso_xyz_yaw_deg = self.robot_pose.torso_xyz_yawdeg
            torso_xyz_yaw_deg[0] += dx
            torso_xyz_yaw_deg[1] += dy
            self.robot_pose.update_torso_xyz_yaw_deg(torso_xyz_yaw_deg)
            self.IKSolverUtil.set_pose_w_R(self.robot_pose)
            time.sleep(.01)
        return

    def test_for_collisions(self,collider):
        for i, j in collider.collisionTests():
            if i[1].collides(j[1]):
                print("Object", i[0].getName(), "collides with", j[0].getName())

    def get_end_effector_from_end_effector_number(self, end_effector):
        if end_effector == 4:
            return self.bl_end_effector
        elif end_effector == 3:
            return self.br_end_effector
        elif end_effector == 1:
            return self.fl_end_effector
        elif end_effector == 2:
            return self.fr_end_effector
        else:
            msg = "Error finding: "+str(end_effector)
            Logger.log(msg, "FAIL")
            return False

    def get_end_effector_current_xyzs(self):
        fl_xyz = self.fl_end_effector.getWorldPosition([0, 0, 0])
        fr_xyz = self.fr_end_effector.getWorldPosition([0, 0, 0])
        br_xyz = self.br_end_effector.getWorldPosition([0, 0, 0])
        bl_xyz = self.bl_end_effector.getWorldPosition([0, 0, 0])
        return fl_xyz, fr_xyz, br_xyz, bl_xyz

    def get_end_effector_current_xyzRs(self):
        fl_xyz, fr_xyz, br_xyz, bl_xyz = self.get_end_effector_current_xyzs()
        fl_R = self.fl_end_effector.getTransform()[0]
        fr_R = self.fr_end_effector.getTransform()[0]
        br_R = self.br_end_effector.getTransform()[0]
        bl_R = self.bl_end_effector.getTransform()[0]
        fl_xyzR, fr_xyzR, br_xyzR, bl_xyzR = fl_xyz+[fl_R], fr_xyz+[fr_R], br_xyz+[br_R], bl_xyz+[bl_R]
        return fl_xyzR, fr_xyzR, br_xyzR, bl_xyzR

    def estimate_torso_xy_yaw_rads_from_stance(self, stance):

        if self.scatter_list == None:
            Logger.log("Error: self.scatter_list has not been initialized", "FAIL")

        fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc = self.get_end_affector_xyzs_from_curr_stance(stance)
        x_ave = (fl_xyzc[0] + fr_xyzc[0] + bl_xyzc[0] + br_xyzc[0]) / 4.0
        y_ave = (fl_xyzc[1] + fr_xyzc[1] + bl_xyzc[1] + br_xyzc[1]) / 4.0
        right_side_v = np.array([fr_xyzc[0] - br_xyzc[0], fr_xyzc[1] - br_xyzc[1]])
        left_side_v = np.array([fl_xyzc[0] - bl_xyzc[0], fl_xyzc[1] - bl_xyzc[1]])
        right_side_normalized_v = right_side_v / np.linalg.norm(right_side_v)
        left_side_normalized_v = left_side_v / np.linalg.norm(left_side_v)
        ave_v = left_side_normalized_v + right_side_normalized_v
        yaw_rads = np.arctan2(ave_v[1], ave_v[0])
        return x_ave, y_ave, yaw_rads

    def get_torso_xy_yawdeg_from_stance(self, stance):
        x_ave, y_ave, yaw_rads = self.estimate_torso_xy_yaw_rads_from_stance(stance)
        return [x_ave, y_ave, np.rad2deg(yaw_rads)]

    def get_end_affector_xyzs_from_curr_stance(self, stance):

        if self.scatter_list == None:
            Logger.log("Error: self.scatter_list has not been initialized", "FAIL")
        fl_xyz = self.scatter_list[int(stance[0])][0:3]
        fr_xyz = self.scatter_list[int(stance[1])][0:3]
        br_xyz = self.scatter_list[int(stance[2])][0:3]
        bl_xyz = self.scatter_list[int(stance[3])][0:3]
        return fl_xyz, fr_xyz, br_xyz, bl_xyz

    def get_active_dofs_from_end_effector(self, end_effector):
        if end_effector == 1:
            return config.FL_ACTIVE_DOFS
        elif end_effector == 2:
            return config.FR_ACTIVE_DOFS
        elif end_effector == 3:
            return config.BR_ACTIVE_DOFS
        elif end_effector == 4:
            return config.BL_ACTIVE_DOFS
        else:
            msg = "Error finding: "+str(end_effector)
            Logger.log(msg, "FAIL")
            return False

    def get_shoulder_from_end_effector(self, end_effector):
        if not end_effector in [1, 2, 3, 4]:
            print_str = "Error: "+str(end_effector)+" unrecognized"
            Logger.log(print_str, "FAIL")
            return
        if end_effector == 4:
            end_effector = self.robosimian.link(config.BL_ACTIVE_DOFS[0])
        elif end_effector == 3:
            end_effector = self.robosimian.link(config.BR_ACTIVE_DOFS[0])
        elif end_effector == 1:
            end_effector = self.robosimian.link(config.FL_ACTIVE_DOFS[0])
        else:
            end_effector = self.robosimian.link(config.FR_ACTIVE_DOFS[0])
        return end_effector

    def update_torso_com_line(self):
        torso_xyz = self.robosimian.getConfig()[0:3]
        ground_xyz = [torso_xyz[0], torso_xyz[1], self.height_map.height_at_xy(torso_xyz[0],torso_xyz[1])]
        VisUtils.visualize_line(torso_xyz, ground_xyz, "torso com")

    def print_robot_config(self):
        print(Logger.pp_list(self.robosimian.getConfig()))

    def get_constraint_obj(self, stance_idx, moving_leg=None, visualize=True):
        current_support_tri = self.get_support_triangle_from_stance_idx(stance_idx)
        if moving_leg:
            range_circles = self.get_end_effector_range_circles(stance_idx, visualize=False)
        else:
            range_circles = self.get_end_effector_range_circles(stance_idx, only_include_current_holds=True, visualize=False)
        constraint_obj = Constraints([current_support_tri], range_circles)
        if visualize:
            constraint_obj.visualize_all()

        return constraint_obj

    def get_moving_leg_xyz_0(self, moving_leg, stance_idx):
        return self.adjust_endeff_z(self.scatter_list[self.stance_path[stance_idx][moving_leg - 1]])

    def get_moving_leg_xyz_f(self, moving_leg, stance_idx):
        return self.adjust_endeff_z(self.scatter_list[self.stance_path[stance_idx + 1][moving_leg - 1]])

    def get_robot_pose_from_current_stance(self,with_end_eff_R=False):
        current_torso_xyz_yaw = self.get_current_torso_xyz_yaw_deg()
        fl, fr, br, bl = self.get_end_effector_current_xyzs()
        if with_end_eff_R:
            fl += [self.fl_end_effector.getTransform()[0]]
            fr += [self.br_end_effector.getTransform()[0]]
            br += [self.br_end_effector.getTransform()[0]]
            bl += [self.fl_end_effector.getTransform()[0]]
        r_pose = RobotPose(fl, fr, br, bl, current_torso_xyz_yaw)
        return r_pose

    def update_rpose_from_current_stance(self, rpose):
        fl_xyzR, fr_xyzR, br_xyzR, bl_xyzR = self.get_end_effector_current_xyzRs()
        torso_xyz_yawdeg = self.get_current_torso_xyz_yaw_deg()
        rpose.update_end_effectors(fl_xyzR, fr_xyzR, br_xyzR, bl_xyzR)
        rpose.update_torso_xyz_yaw_deg(torso_xyz_yawdeg)

    def get_robot_pose_from_stance(self, stance, with_end_effector_Rs=False, debug=False, visualize_normal=True):
        fl, fr, br, bl = self.get_end_affector_xyzs_from_curr_stance(stance)
        fl, fr, br, bl = self.adjust_all_end_effector_zs(fl, fr, br, bl)
        estimated_torso_xy_yaw = self.estimate_torso_xy_yaw_rads_from_stance(stance)
        torso_z = self.get_torso_z_des_from_xy(estimated_torso_xy_yaw, stance, debug=debug)
        estimated_torso_xyz_yaw = [estimated_torso_xy_yaw[0], estimated_torso_xy_yaw[1], torso_z, np.rad2deg(estimated_torso_xy_yaw[2]) ]
        if with_end_effector_Rs:
            fl_xyzR = fl + [self.get_end_effector_rotation_matrix( 1, xyz=fl, visualize_normal=visualize_normal, debug=debug)]
            fr_xyzR = fr + [self.get_end_effector_rotation_matrix( 2, xyz=fr, visualize_normal=visualize_normal, debug=debug)]
            br_xyzR = br + [self.get_end_effector_rotation_matrix( 3, xyz=br, visualize_normal=visualize_normal, debug=debug)]
            bl_xyzR = bl + [self.get_end_effector_rotation_matrix( 4, xyz=bl, visualize_normal=visualize_normal, debug=debug)]
            r_pose = RobotPose(fl_xyzR, fr_xyzR, br_xyzR, bl_xyzR, estimated_torso_xyz_yaw)
        else:
            r_pose = RobotPose(fl, fr, br, bl, estimated_torso_xyz_yaw)
        return r_pose

    def adjust_all_end_effector_zs(self,fl, fr, br, bl):
        return self.adjust_endeff_z(fl), self.adjust_endeff_z(fr),self.adjust_endeff_z(br),self.adjust_endeff_z(bl)

    def get_moving_leg_xyzR_f(self, moving_leg: int, stance_idx: int, visualize_normal=False):
        return self.get_moving_leg_xyzR_0(moving_leg, stance_idx + 1, visualize_normal=visualize_normal)

    def get_moving_leg_xyzR_0(self, moving_leg: int, stance_idx: int, visualize_normal=False):
        ''' Assumes class has a gradient map'''
        xyz = self.get_moving_leg_xyz_0(moving_leg, stance_idx)
        R = self.get_end_effector_rotation_matrix(moving_leg, xyz=xyz, visualize_normal=visualize_normal)
        return xyz + [R]

    def get_end_effector_rotation_matrix(self, end_effector, xyz=None, yaw_rad=None, visualize_normal=False, debug=False):
        upright = [0, 0, -1, 0, 1, 0, 1, 0, 0]
        if end_effector in [1, 4]:
            upright = [0, 0, -1, 0, -1, 0, -1, 0, 0]
        if xyz:
            xyz[2] -= config.END_EFFECTOR_HEIGHT
            x_grad, y_grad = self.gradient_map.get_grad_at_world_xy(xyz[0], xyz[1])
            x_grad /= 2 * config.HM_X_GRANULARITY; y_grad /= 2 * config.HM_Y_GRANULARITY
            normal = [x_grad, y_grad, 1 ]
            normal_magnitude = MathUtils._3d_vector_magnitude(normal)
            if normal_magnitude > .0001:
                normal /= MathUtils._3d_vector_magnitude(normal)
            else:
                normal = [0,0,1]
            end_effector_vector = [0,0,1]
            axis = MathUtils._3d_vector_cross_product(normal, end_effector_vector)
            angle = -MathUtils.angle_between_two_3d_vectors(normal, end_effector_vector)
            axis_magnitude = MathUtils._3d_vector_magnitude(axis)
            if axis_magnitude > .0001:
                axis /= axis_magnitude
            rotation_R = so3.from_axis_angle((axis,angle))
            R = so3.mul(rotation_R, upright)
            if visualize_normal:
                VisUtils.visualize_line(
                    xyz, [xyz[0] + normal[0], xyz[1] + normal[1], xyz[2] + normal[2]], "unit normal for {end_effector}")
            if not so3.is_rotation(R): Logger.log("Failure - calculated matrix is NOT a rotation matrix", "FAIL")
            xyz[2] += config.END_EFFECTOR_HEIGHT
            if debug:print("for end effector:",end_effector,"at xyz:",Logger.pp_list(xyz)," normal:",normal,"R:",Logger.pp_list(R))
            return R
        else:
            if yaw_rad is None:
                yaw_rad = self.get_current_torso_yaw_rads()
            yaw_rotation_aa = ([0, 0, 1], yaw_rad)
            yaw_rotation_R = so3.from_axis_angle(yaw_rotation_aa)
            R = so3.mul(yaw_rotation_R, upright)
            return R

    def get_linear_mid_motion_xyzR(self, xyzR0, xyzRf, i, imax):

        xyz0 = xyzR0[0:3]
        xyzf = xyzRf[0:3]
        xyz = self.get_linear_3d_mid_motion_vals(xyz0, xyzf, i, imax)
        R = MathUtils.mid_motion_rotation_matrix(xyzR0[3], xyzRf[3], i, imax)
        return xyz+[R]

    def get_parabolic_mid_motion_xyzR(self, xyzR0, xyzRf, i, imax, step_height):

        xyz0 = xyzR0[0:3]
        xyzf = xyzRf[0:3]
        xyz = self.get_parabolic_mid_motion_xyz(xyz0, xyzf, i, imax, step_height)
        R = MathUtils.mid_motion_rotation_matrix(xyzR0[3], xyzRf[3], i, imax)
        return xyz+[R]

    # --------------- End of New Code

    def get_support_triangle_from_stance_idx(self, stance_idx, name=None):
        stance = self.stance_path[stance_idx]
        fl_xyz, fr_xyz, br_xyz, bl_xyz = self.get_end_affector_xyzs_from_curr_stance(stance)

        support_triangle_points = [fl_xyz, fr_xyz, br_xyz, bl_xyz]
        support_triangle_points.pop(stance[4] - 1)

        moving_leg = self.get_moving_leg(stance)
        if moving_leg == 1:
            diag_points = [True, False, True]
        elif moving_leg == 2:
            diag_points = [True, True, False]
        elif moving_leg == 3:
            diag_points = [False, True, True]
        elif moving_leg == 4:
            diag_points = [True, False, True]
        else:
            Logger.log("moving leg unrecognized", "FAIL")
            diag_points = []
            exit()

        next_support_tri = self.get_support_triangle_from_points(support_triangle_points, diag_points, name=name)
        return next_support_tri

    def get_moving_leg(self, stance):
        return stance[4]

    def get_moving_leg_from_stance_idx(self, stance_idx):
        stance = self.stance_path[stance_idx]
        return self.get_moving_leg(stance)

    def get_end_effector_range_circles(self, stance_idx, visualize=False, ignore_leg=None, only_include_current_holds=False):
        stance = self.stance_path[stance_idx]
        fl_xyz_current, fr_xyz_current, br_xyz_current, bl_xyz_current = self.get_end_affector_xyzs_from_curr_stance(stance)
        current_fs_holds = [fl_xyz_current, fr_xyz_current, br_xyz_current, bl_xyz_current]
        current_end_effectors = [1, 2, 3, 4]
        if not ignore_leg:
            if not only_include_current_holds:
                stance_next = self.stance_path[stance_idx + 1]
                fl_xyz_next, fr_xyz_next, br_xyz_next, bl_xyz_next = self.get_end_affector_xyzs_from_curr_stance(stance_next)
                next_fs_holds = [fl_xyz_next, fr_xyz_next, br_xyz_next, bl_xyz_next]
                next_end_effectors = [1, 2, 3, 4]
                new_hold = [next_fs_holds[stance[4] - 1]]
                next_end_effector = [next_end_effectors[stance[4] - 1]]
                end_effectors = current_end_effectors + next_end_effector
                all_holds = current_fs_holds + new_hold
            else:
                end_effectors = current_end_effectors
                all_holds = current_fs_holds
        else:
            end_effectors = current_end_effectors
            end_effectors.pop(ignore_leg-1)
            current_fs_holds.pop(ignore_leg-1)
            all_holds = current_fs_holds
        ranges = []
        for i in range(len(all_holds)):
            end_effector = end_effectors[i]
            point = all_holds[i]
            circle_ = self.get_end_effector_2D_range_circle(end_effector, at_point=point)
            ranges.append(circle_)
            if visualize:
                circle_.visualize()
        return ranges

    def get_torso_z_des_from_xy(self, torso_xy, curr_state, debug=False):

        # TODO: This requires more consideration

        fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc = self.get_end_affector_xyzs_from_curr_stance(curr_state)
        end_eff_zs = np.array([fl_xyzc[2],fr_xyzc[2],bl_xyzc[2], br_xyzc[2]])
        z_endeff_ave = np.average(end_eff_zs)
        z_env = self.height_map.height_at_xy(torso_xy[0], torso_xy[1])
        min_torso_clearance = config.MIN_TORSO_CLEARANCE
        torso_z_des = config.TORSO_Z_DESIRED

        # if debug: print "z_env:", z_env, "\tz_endeff_ave:", z_endeff_ave, "\tz_endeff_stddev:", z_endeff_stddev, "\t z_endeff_ave + min_torso_clearance:",logger.pp_double(z_endeff_ave + min_torso_clearance)
        # note: THis will cause torso z 'jumps' if the torso goes over an obstacle which goes above the torso clearance
        if z_env > z_endeff_ave + min_torso_clearance:
            z_ret = z_env + min_torso_clearance
            # if debug: print "z_env > z_endeff_ave + min_torso_clearance \tz:",logger.pp_double(z_ret)
        else:
            z_ret = z_endeff_ave + torso_z_des
            # if debug: print "z_ret = z_endeff_ave + torso_z_des :\t",logger.pp_double(z_ret)
        return z_ret

    def adjust_endeff_z(self, xyz):
        offset = config.END_EFFECTOR_HEIGHT
        # z = self.height_map.height_at_xy(xyz[0], xyz[1])
        xyz_new = [xyz[0], xyz[1], xyz[2] + offset]
        return xyz_new

    def print_state_list(self):
        for stance in self.stance_path:
            print("\n  ", stance)
            print("     ", Logger.pp_list(self.scatter_list[stance[0]]), ", ", Logger.pp_list(
                self.scatter_list[stance[1]]), ", ", Logger.pp_list(self.scatter_list[stance[2]]), ", ", Logger.pp_list(
                self.scatter_list[stance[3]]))

    def get_optimized_torso_xy_yaw_deg(self, current_support_tri, static_range_circles, moving_end_eff_xyz, current_average_xy_yaw_deg, ignore_moving_end_eff=False, debug=False):

        outbound_limiter = 100
        outbound_scaler = 5.0
        inbound_scalar = 1.0

        def cost(xy_yaw_deg):
            torso_x = xy_yaw_deg[0]
            torso_y = xy_yaw_deg[1]
            torso_xy = [torso_x, torso_y]
            torso_yaw_rad = np.deg2rad(xy_yaw_deg[2])
            torso_xy_shapely_point = Point(torso_x, torso_y)

            if not ignore_moving_end_eff:
                moving_end_eff_range_circle_radius = self.get_torso_range_from_end_effector(4, at_point=moving_end_eff_xyz, yaw_rads=torso_yaw_rad)
                moving_end_eff_range_circle = LegRange(moving_end_eff_xyz, moving_end_eff_range_circle_radius, self.height_map)
                range_circles = static_range_circles + [moving_end_eff_range_circle]
            else:
                range_circles = static_range_circles

            if not current_support_tri.point_is_inside(torso_xy):
                dist = current_support_tri.get_shapely_poly().boundary.distance(torso_xy_shapely_point)
                cost = outbound_scaler * dist + outbound_limiter
                #print " - outbound valus:", logger.pp_list(xy_yaw_deg), "has cost:", cost
                return cost

            dist = 0
            for range_circle in range_circles:
                dist +=  range_circle.get_shapely_poly().distance(torso_xy_shapely_point)
            cost = inbound_scalar * dist
            #print " -  inbound valus:", logger.pp_list(xy_yaw_deg), "has cost:", cost
            return cost

        ret_obj = optimize.minimize(cost, current_average_xy_yaw_deg)
        ret_val = np.ndarray.tolist(ret_obj.x)

        if cost(ret_obj.x) > outbound_limiter - .01:
            if debug:
                Logger.log("Error, could not find xy inside of support triangle", "FAIL")
                print(ret_obj)
            return False
        else:
            if ret_obj.success:
                #print "get_optimized_torso_xy_yaw_deg(): success! returning:",ret_val
                return ret_val
            else:
                Logger.log("Error, could not find xy inside of support triangles, range circles", "FAIL")
                return False

    def get_end_effector_2D_range_circle(self, end_effector, circle_name = None, at_point=None, yaw_rads=None):

        '''
        @summary returns a _2DLegRadius object given the end effectors name
        @param link_name: string specifiing the link name
        @param circle_name: name of circle
        @return: _2DLegRadius object
        '''

        r = self.get_torso_range_from_end_effector(end_effector, at_point=at_point, yaw_rads=yaw_rads)
        global_xyz = self.get_end_effector_from_end_effector_number(end_effector).getWorldPosition([0,0,0])
        if at_point:
            global_xyz = at_point
        return LegRange(global_xyz, r, self.height_map, name=circle_name)

    def get_support_triangle_from_points(self, P, diag_points, name=None):
        '''
        @summary returns a SupportTriangle created by the parameterized array of Points
        @param P: list of xyz coordinates
        @return: SupportTriangle object
        '''
        support_tri = SupportTriangle(P, self.height_map, diag_points, name=name)
        support_tri.enforce_safety_margin(config.SUPPORT_TRIANGLE_SAFETY_MARGIN)
        return support_tri

    def get_centroid_from_multiple_poly_intersections(self, support_triangles, add_z=None, closest_to=None):
        if len(support_triangles) < 2:
            Logger.log("Error: support_triangles have less than two objects", "FAIL")
            return False
        first_obj = support_triangles[0]
        rest = support_triangles[1:]
        ret = first_obj.xy_centroid_from_o_3dgeoms(rest, closest_to=closest_to)
        if add_z:
            ret = [ret[0], ret[1], add_z]
        return ret

    def get_current_torso_xy_yaw_deg(self):
        q = self.robosimian.getConfig()
        xyz_yaw = [q[0], q[1], np.rad2deg(q[3])]
        return xyz_yaw

    def get_current_torso_xyz_yaw_deg(self):
        q = self.robosimian.getConfig()
        xyz_yaw = [q[0], q[1], q[2], np.rad2deg(q[3])]
        return xyz_yaw

    def get_linear_3d_mid_motion_vals(self, start, end, _i, _i_max):
        x_delta = end[0] - start[0]
        y_delta = end[1] - start[1]
        z_delta = end[2] - start[2]
        try:
            x = start[0] + (_i / _i_max) * x_delta
        except ZeroDivisionError:
            x = start[0]
        try:
            y = start[1] + (_i / _i_max) * y_delta
        except ZeroDivisionError:
            y = start[1]
        try:
            z = start[2] + (_i / _i_max) * z_delta
        except ZeroDivisionError:
            z = start[2]
        return [x, y, z]

    def get_parabolic_mid_motion_xyz(self, startXYZ, endXYZ, i, i_max, step_height):

        # prevents division by 0
        if i == 0: i = 1

        x_start = float(startXYZ[0])
        y_start = float(startXYZ[1])
        z_start = float(startXYZ[2])

        x_end = float(endXYZ[0])
        y_end = float(endXYZ[1])
        z_end = float(endXYZ[2])

        x_delta = x_end - x_start
        y_delta = y_end - y_start
        z_delta = z_end - z_start
        delta_xy_total = MathUtils._3d_euclidian_distance( [x_start,y_start,0], [x_end, y_end, 0] )

        x = x_start + (float(i) / float(i_max) * x_delta)
        y = y_start + (float(i) / float(i_max) * y_delta)
        z = z_start + (float(i) / float(i_max) * z_delta)
        delta_xy = MathUtils._3d_euclidian_distance([x,y,0],[x_start,y_start,0])

        height_offset = 0
        if delta_xy_total > .0001:
            height_offset = (-4*step_height*delta_xy*(delta_xy-delta_xy_total)) / delta_xy_total
        # see https://www.desmos.com/calculator/v8wb6o83jh
        return [ x, y, z + height_offset ]

    def get_torso_rotation_matrix_from_yaw_deg(self, yaw_deg):
        curr_yaw_deg = self.get_current_torso_yaw_deg()
        offset = yaw_deg - curr_yaw_deg
        return self.get_torso_rotation_matrix_from_yaw_deg_offset(offset)

    def get_current_torso_yaw_deg(self):
        q = self.robosimian.getConfig()
        deg = np.rad2deg(q[3])
        return deg

    def get_current_torso_yaw_rads(self):
        return self.robosimian.getConfig()[3]

    def get_torso_rotation_matrix_from_yaw_deg_offset(self, yaw_offset_deg):
        current_torso_yaw_rad = self.get_current_torso_yaw_rads()
        desired_torso_yaw_rad = current_torso_yaw_rad + np.deg2rad(yaw_offset_deg)
        axis_angle = ([0,0,1], desired_torso_yaw_rad)
        desired_r = so3.from_axis_angle(axis_angle)
        return desired_r

    #TODO: Update to include non level/flat end effectors
    def get_torso_range_from_end_effector(self, end_effector, at_point=None, yaw_rads=None):
        R = config.END_AFFECTOR_RADIUS_TO_SHOULDER
        S = config.SHOULDER_TORSO_XY_EUCLIDEAN_DIF
        yaw = self.get_current_torso_yaw_rads()
        if yaw_rads: yaw = yaw_rads
        psi = config.SHOULDER_TORSO_PSI_RADS
        shoulder_link = self.get_shoulder_from_end_effector(end_effector)
        shoulder_world_xyz = shoulder_link.getWorldPosition([0, 0, 0])
        if end_effector == 1:
            link_global_xyz = self.fl_end_effector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point
            leg_dx = shoulder_world_xyz[0] - link_global_xyz[0]
            leg_dy = shoulder_world_xyz[1] - link_global_xyz[1]
            theta = np.arctan2(leg_dy, leg_dx)
            delta_y_max = R * np.sin(theta) + (- S * np.cos((np.pi / 2) - (yaw + psi)))
            delta_x_max = R * np.cos(theta) + (- S * np.sin((np.pi / 2) - (yaw + psi)))
        elif end_effector == 4:
            link_global_xyz = self.bl_end_effector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point
            leg_dx = shoulder_world_xyz[0] - link_global_xyz[0]
            leg_dy = shoulder_world_xyz[1] - link_global_xyz[1]
            theta = np.arctan2(leg_dy, leg_dx)
            delta_y_max = R * np.sin(theta) + (- S * np.sin(psi - yaw))
            delta_x_max = R * np.cos(theta) + (S * np.cos(psi - yaw))
        elif end_effector == 2:
            link_global_xyz = self.fr_end_effector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point
            leg_dx = shoulder_world_xyz[0] - link_global_xyz[0]
            leg_dy = shoulder_world_xyz[1] - link_global_xyz[1]
            theta = np.arctan2(leg_dy, leg_dx)
            delta_y_max = R * np.sin(theta) + S * np.sin(yaw - psi - np.pi)
            delta_x_max = R * np.cos(theta) + S * np.cos(yaw - psi - np.pi)
        else:
            link_global_xyz = self.br_end_effector.getWorldPosition([0, 0, 0])
            if at_point:
                link_global_xyz = at_point
            leg_dx = (shoulder_world_xyz[0] - link_global_xyz[0])
            leg_dy = (shoulder_world_xyz[1] - link_global_xyz[1])
            theta = np.arctan2(leg_dy, leg_dx)
            delta_y_max = R * np.sin(theta) + S * np.sin(yaw + psi)
            delta_x_max = R * np.cos(theta) + S * np.cos(yaw + psi)
        r = np.sqrt(delta_x_max ** 2 + delta_y_max ** 2)
        return config.END_RANGE_MULTIPLIER * r

    def get_torso_R_from_yaw_rad(self, yaw_rad):
        axis_angle = ([0, 0, 1], yaw_rad)
        desired_r = so3.from_axis_angle(axis_angle)
        return desired_r
