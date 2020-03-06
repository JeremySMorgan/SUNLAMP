from klampt import vis
from src.utils.logger import Logger
from src.utils.math_utils import MathUtils
from src.utils.vis_utils import VisUtils
from shapely.geometry import GeometryCollection
from shapely.geometry import LinearRing
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.ops import nearest_points

class _3dGeometrySuperclass:

    def __init__(self, height_map, name, shapely):
        self.height_map = height_map
        self.name = name
        self.shapely_poly = shapely

    @staticmethod
    def get_intersection(obj, o_objects):
        ret = obj.intersection(o_objects[0])
        if len(o_objects) >= 2:
            for o_obj in o_objects:
                ret = ret.intersection(o_obj)
        return ret

    def get_2d_centroid(self):
        '''
        @summary returns the xy center point of a support polygon
        @return: tuple with x, y coordinates
        '''
        return self.shapely_poly.centroid.coords[0]

    def make_2d(self,_3dP):
        if len(_3dP) > 2:
            return (_3dP[0],_3dP[1])

    def remove_visualization(self):
        if self.name:
            try:
                vis.hide(self.name)
            except AttributeError:
                msg = "Error removing SupportTriangle ("+self.name+") from klamp't vis"
                Logger.log(msg, "FAIL")

    def point_is_inside(self, P):
        point = Point(P[0], P[1])
        return self.shapely_poly.contains(point)

    def get_centroid(self):
        return self.shapely_poly.centroid.coords[0]

    def get_shapely_poly(self):
        return self.shapely_poly

    def point_is_inside_o_3d_geoms(self, point_xyz, o_2D_objects):
        intersection = self.get_shapely_intersection_from_o_3dgeoms(o_2D_objects)
        point = Point(point_xyz[0], point_xyz[1])
        return intersection.contains(point)

    def get_shapely_intersection_from_o_3dgeoms(self, o_2DGeometry_objs):
        obj = self.shapely_poly
        o_objs = []
        for i in o_2DGeometry_objs:
            o_objs.append(i.shapely_poly)
        intersection = _3dGeometrySuperclass.get_intersection(obj, o_objs)
        return intersection

    def xy_centroid_from_o_3dgeoms(self, o_2DGeometry_objs, closest_to=None):
        '''
        @summary returns the centroid of the intersection area of numrerous _2D_xxx support ibjects
        @param o_2D_objects:
        @return:
        '''
        intersection_poly = self.get_shapely_intersection_from_o_3dgeoms(o_2DGeometry_objs)
        # VisUtils.visualize_shapely_poly(intersection_poly, add_z=.25, color=VisUtils.TURQ)

        intersection_centroid = intersection_poly.centroid.coords
        try:
            try:
                intersection_poly = intersection_poly.geoms[1]
            except IndexError:
                Logger.log("2d objects do not intersect", "FAIL")
                return None
        except AttributeError:
            pass

        if closest_to:

            d = .001
            intersection_poly_centroid = intersection_poly.centroid.coords[0]
            v = MathUtils.normalize([closest_to[0] - intersection_poly_centroid[0], closest_to[1] - intersection_poly_centroid[1]])
            xy = intersection_poly_centroid
            while intersection_poly.contains(Point(xy)):
                xy = MathUtils.add_scaled_vector_to_pt(xy, v, d)
                # print(f"xy: {xy}\t inside: {intersection_poly.contains(Point(xy))}")

            xy = MathUtils.add_scaled_vector_to_pt(xy, v, -d)
            # print(f"Done. xy: {xy}\t inside: {intersection_poly.contains(Point(xy))}")
            return xy


            # point = Point(closest_to[0], closest_to[1])
            # pol_ext = LinearRing(intersection_poly.exterior.coords)

            # d = pol_ext.project(point)
            # p = pol_ext.interpolate(d)
            # closest_point_coords = list(p.coords)[0]
            # print(type(closest_point_coords))
            # print(closest_point_coords)
            # return closest_point_coords

            # p1, p2 = nearest_points(intersection_poly, point)
            # print("intersection_poly.contains(p1): ",intersection_poly.contains(p1))
            # print("intersection_poly.contains(p2): ",intersection_poly.contains(p2))
            # return p1.x, p1.y

            # if type(intersection_poly) == type(GeometryCollection):
            #     for i in intersection_poly.geoms:
            #         if type(i) == type(Polygon):
            #             intersection_poly = i
            # pol_ext = LinearRing(intersection_poly.exterior.coords)
            # closest_to_point = Point(closest_to[0], closest_to[1])
            # d = pol_ext.project(closest_to_point)
            # p = pol_ext.interpolate(d)
            # closest_point_coords = list(p.coords)[0]
            # return closest_point_coords


        else:
            try:
                x = intersection_centroid.xy[0][0]
                y = intersection_centroid.xy[1][0]
                return [x,y]
            except AttributeError:
                Logger.log("2d objects do not intersect", "FAIL")
                return None

    def point_is_inside_intersection_of_multiple_2DGeometry_objects(self, point_xyz, o_2D_objects):
        intersection = self.get_shapely_intersection_from_o_3dgeoms(o_2D_objects)
        point = Point(point_xyz[0], point_xyz[1])
        return intersection.contains(point)

    def support_poly_is_in_intersection_with_other_2d_support_poly(self, support_triangle, P) :
        if not type(support_triangle) == type(self):
            Logger.log(
                "Error in support_poly_is_in_intersection_with_other_2d_support_poly(): o__2DSupportPolygon is not of type __2DSupportPolygon",
                "FAIL")
            return False
        o_poly = support_triangle.get_shapely_poly()
        intersection_poly = self.shapely_poly.intersection(o_poly)
        point = Point(P[0], P[1])
        return intersection_poly.contains(point)

    def xy_centroid_of_intersection(self, support_triangle):
        if not type(support_triangle) == type(self):
            Logger.log("Error in centroid_of_intersection(): o__2DSupportPolygon is not of type __2DSupportPolygon",
                       "FAIL")
            return False
        o_poly = support_triangle.get_shapely_poly()
        intersection_poly = self.shapely_poly.intersection(o_poly)
        return intersection_poly.centroid.coords[0]

