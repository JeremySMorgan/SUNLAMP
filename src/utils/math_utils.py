import random
import numpy as np
from klampt.math import so3
from scipy import interpolate
from scipy.optimize import minimize
from src.utils.logger import Logger as Lg


class MathUtils:

    I_3d = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])

    @staticmethod
    def rotation_matrix_between_3d_vectors(a, b):

        # Returns the rotation matrix which rotates unit vector a onto unit vector b
        a = np.array(a)
        b = np.array(b)
        a = a / MathUtils._3d_vector_magnitude(a)
        b = a / MathUtils._3d_vector_magnitude(b)

        v = np.cross(a, b)
        s = MathUtils._3d_vector_magnitude(v)
        c = a * b
        if np.abs(s) < .00001:
            return MathUtils.I_3d

        v_skew_sym = np.array([ [0,    -v[2],   v[1]],
                                [v[2],    0,   -v[0]],
                                [-v[1],  v[0],    0 ]])

        R = MathUtils.I_3d + v_skew_sym + (v_skew_sym ** 2 * ((1 - c) / s ** 2))
        return R

    @staticmethod
    def gradient_to_slope_deg(grad_):
        #  Numerical approximation for gradient -> degree. R^2 = 0.9999
        # see https://www.desmos.com/calculator/aykma8veq1
        a = 57.3521
        b = 99.36
        deg = a * np.arctan(b*grad_)
        return deg

    @staticmethod
    def slope_deg_to_grad(deg):
        # see https://www.desmos.com/calculator/dhmfrigq1n
        a = -0.00563426
        b = -0.0299936
        c = 5.7986*(10**-8)
        d = -0.171289
        grad = a*np.sinh(b*deg) + c*np.cosh(d*deg)
        return grad

    @staticmethod
    def gradient_to_slope_arr(grad_arr):
        slope_arr = np.ndarray(shape=grad_arr.shape)
        slope_arr.fill(0)
        for x in range(grad_arr.shape[0]):
            for y in range(grad_arr.shape[1]):
                slope_arr[x][y] = MathUtils.gradient_to_slope_deg(grad_arr[x][y])
        return slope_arr

    @staticmethod
    def hm_specified_by_abc_z0(a, b, c, z0, shape: tuple):
        return np.fromfunction(lambda x, y: (c*z0 - a*x - b*y) / c, shape, dtype=float)

    @staticmethod
    def best_fitting_plane(hm):

        # Find: z0, a, b, c

        # nb_x_idxs, nb_y_idxs = hm.shape[0], hm.shape[1]
        # xs = np.array(range(nb_x_idxs))
        # ys = np.array(range(nb_y_idxs))

        # print(f"xs: {xs}")
        # print(f"ys: {ys}")

        # hm_specified = np.zeros(shape=hm.shape)

        def mse_(p):

            a = p[0]
            b = p[1]
            c = p[2]
            z0 = p[3]

            # hm_specified = (c*z0 - a*xs - b*ys) / c
            hm_specified = np.fromfunction(lambda x, y: (c*z0 - a*x - b*y) / c, hm.shape, dtype=float)

            # for x in range(nb_x_idxs):
            #     for y in range(nb_y_idxs):
            #         hm_specified[x][y] = (c*z0 - a*x - b*y) / c

            error = np.sum(np.abs(hm_specified - hm))**2

            return error

        bounds = ((-1, 1), (-1, 1), (-1, 1), (-10, 10))
        p0 = np.array([0, 0, 1, np.average(hm)])

        res = minimize(mse_, p0, bounds=bounds)
        p_opt = res.x
        a, b, c, z0 = p_opt[0], p_opt[1], p_opt[2], p_opt[3]
        return a, b, c, z0

    @staticmethod
    def xy_gradient(height_map_arr):
        '''
        @summary: returns the x and y gradients from a given height map
        :param height_map_arr:
        :return:
        '''
        x_grad = np.ndarray(shape=height_map_arr.shape)
        y_grad = np.ndarray(shape=height_map_arr.shape)
        x_grad.fill(0)
        y_grad.fill(0)
        for y in range(height_map_arr.shape[1]-1):
            for x in range(height_map_arr.shape[0]-1):

                if x-1 < 0:
                    x_minus = 0
                else:
                    x_minus = height_map_arr[x - 1][y]

                if x + 1 == height_map_arr.shape[0]:
                    x_plus = 0
                else:
                    x_plus = height_map_arr[x + 1][y]

                if y - 1 < 0:
                    y_minus = 0
                else:
                    y_minus = height_map_arr[x][y - 1]

                if y + 1 == height_map_arr.shape[1]:
                    y_plus = 0
                else:
                    y_plus = height_map_arr[x][y + 1]

                x_grad[x][y] = x_minus - x_plus
                y_grad[x][y] = y_minus - y_plus

        return x_grad, y_grad

    @staticmethod
    def mid_motion_rotation_matrix(R0, Rf, i, i_max):

        if not so3.is_rotation(R0):
#             Lg.log("R0 is not a rotation", "FAIL")
            return

        if not so3.is_rotation(Rf):
#             Lg.log("Rf is not a rotation", "FAIL")
            return

        h = float(i) / float(i_max)
        R = MathUtils.spherical_linear_interpolation_fromR(R0, Rf, h)
        if not so3.is_rotation(R):
            Lg.pp_list(R)
            Lg.log("R(mid motion) is not a rotation", "FAIL")
            return R0
        return R

    @staticmethod
    def incenter_circle_xy_R_fromT(T):
        return MathUtils.incenter_circle_xy_R(T[0],T[1],T[2])

    @staticmethod
    def incenter_circle_xy_R(P1,P2,P3):
        a = MathUtils._2d_euclidian_distance(P1,P2)
        b = MathUtils._2d_euclidian_distance(P2,P3)
        c = MathUtils._2d_euclidian_distance(P3,P1)
        p = a + b + c
        x = (b*P1[0]+c*P2[0]+a*P3[0])/p
        y = (b*P1[1]+c*P2[1]+a*P3[1])/p
        k = .25*np.sqrt(p*(a-b+c)*(b-c+a)*(c-a+b))
        r = (2.0*k)/p
        return x,y,r


    @staticmethod
    def sigmoid_fn(x, a, b):
        num = np.power(np.e, ((x/a)-b))
        den = np.power(np.e, ((x/a)-b)) + 1
        return num / den

    @staticmethod
    def normalized_3d_vector(a):
        return np.array(a) / MathUtils._3d_vector_magnitude(a)

    @staticmethod
    def _2d_rotation_transformation(x, y, theta_rad):
        _x = np.cos(theta_rad) * x - np.sin(theta_rad) * y
        _y = (np.sin(theta_rad) * x + np.cos(theta_rad) * y)
        return [_x, _y]

    @staticmethod
    def _4d_euclidian_distance(xyz1, xyz2):
        return np.sqrt( (xyz2[0]-xyz1[0])**2 + (xyz2[1]-xyz1[1])**2 + (xyz2[2]-xyz1[2])**2  + (xyz2[3]-xyz1[3])**2 )

    @staticmethod
    def _3d_euclidian_distance(xyz1, xyz2):
        return ((xyz2[0] - xyz1[0])** 2 + (xyz2[1] - xyz1[1])** 2 + (xyz2[2] - xyz1[2])** 2) ** .5

    @staticmethod
    def _2d_euclidian_distance(xyz1, xyz2):
        return np.sqrt((xyz2[0] - xyz1[0])** 2 + (xyz2[1] - xyz1[1])** 2)

    @staticmethod
    def _3d_vector_magnitude(V):
        return np.sqrt(V[0]**2 + V[1]**2 + V[2]**2)

    @staticmethod
    def _2d_magnitude(x,y):
        return np.sqrt(x**2 + y**2)

    @staticmethod
    def _3d_vector_cross_product(v1,v2):
        x = v1[1]*v2[2] - v2[1]*v1[2]
        y = v1[2]*v2[0] - v2[2]*v1[0]
        z = v1[0]*v2[1] - v2[0]*v1[1]
        return [x,y,z]

    @staticmethod
    def angle_between_two_3d_vectors(v1, v2):
        v1_mag = MathUtils._3d_vector_magnitude(v1)
        v2_mag = MathUtils._3d_vector_magnitude(v2)
        a = MathUtils._3d_dot_product(v1, v2) / (v1_mag * v2_mag)
        angle = np.arccos(a)
        return angle

    @staticmethod
    def _3d_dot_product(v1,v2):
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

    @staticmethod
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    @staticmethod
    def intersect(A, B, C, D):
        return MathUtils.ccw(A, C, D) != MathUtils.ccw(B, C, D) and MathUtils.ccw(A, B, C) != MathUtils.ccw(A, B, D)

    @staticmethod
    def quantarion_angle(q):
        return 2*np.arccos(q[3])

    @staticmethod
    def multiply_quantarnions(q0,qf):
        w0, x0, y0, z0 = q0
        w1, x1, y1, z1 = qf
        ret = [-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
               x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
               -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
               x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0]
        return ret

    @staticmethod
    def spherical_linear_interpolation_fromR(R0, Rf, h):

        # See https://en.wikipedia.org/wiki/Slerp

        q0 = np.array(so3.quaternion(R0))
        qf = np.array(so3.quaternion(Rf))
        v0 = np.array(MathUtils.normalize(q0))
        vf = np.array(MathUtils.normalize(qf))
        dot = np.dot(v0, vf)
        if dot < 0.0:
            vf = -vf
            dot = -dot

        # TODO: Is this causing non rotation matrixes?
        dot_threshold = .995
        if dot > dot_threshold:

            # print(f"WARNING: dot={dot} > threshold={dot_threshold}\t linearly interprolating")

            # If results are close, calculate new result linearly
            result = v0 + h*(vf-v0)
            result_as_list = result.tolist()
            q = MathUtils.normalize(result_as_list)

        else:
            theta_0 = np.arccos(dot)
            theta = theta_0 * h
            sin_theta = np.sin(theta)
            sin_theta_0 = np.sin(theta_0)
            s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0
            q = (s0 * v0) + (s1 * vf)
        return so3.from_quaternion(q)

    @staticmethod
    def normalize(v, tolerance=0.00001):
        mag2 = sum(n * n for n in v)
        if abs(mag2 - 1.0) > tolerance:
            mag = np.sqrt(mag2)
            v = tuple(n / mag for n in v)
        return v

    @staticmethod
    def dot_product(v0,v1):
        ret = 0
        for i in range(v0):
            ret += v0[i]*v1[i]
        return ret

    @staticmethod
    def clamp(input_,min_, max_):
        return max(min(input_, max_), min_)

    @staticmethod
    def points_on_same_side_of_line( X1, X2, P1, P2):
        '''
        :param X1: [x1,y1], first point used to define line
        :param X2: [x2,y2], second point used to define line
        :param P1: [xp1, yp1], first point
        :param P2: [xp2, yp2], second point
        :return: Bool
        see https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
        note if both points are on the line this fn will return False
        '''
        d_1 = (P1[0]-X1[0])*(X2[1]-X1[1]) - (P1[1]-X1[1])*(X2[0]-X1[0])
        d_2 = (P2[0]-X1[0])*(X2[1]-X1[1]) - (P2[1]-X1[1])*(X2[0]-X1[0])

        if d_1 < 0 and d_2 < 0:
            return True
        if d_1 > 0 and d_2 > 0:
            return True
        return False

    @staticmethod
    def joint_angle_distances(q1, q2):
        '''
                Assumes q1,q2 in [-pi, pi]
        :param q1:
        :param q2:
        :return:
        '''
        q1_np = np.array(q1)
        q2_np = np.array(q2)
        _2pi_arr = 2 * np.pi * np.ones(q1_np.shape)
        d1 = np.abs(q1_np - q2_np)
        d2 = np.abs(q1_np - _2pi_arr - q2_np)
        d3 = np.abs(q1_np + _2pi_arr - q2_np)
        d = np.minimum(d1, d2, d3)
        return d

    @staticmethod
    def max_joint_angle_distance(q1, q2):
        return np.max(MathUtils.joint_angle_distances(q1, q2))

    @staticmethod
    def _2dpoint_inbounds(X, min_maxX, min_maxY):
        if X[0] < min_maxX[0] or X[0] > min_maxX[1]:
            return False
        if X[1] < min_maxY[0] or X[1] > min_maxY[1]:
            return False
        return True

    @staticmethod
    def _3d_pointlist_cubic_interprolation(xyz_vals):
        '''
        @summary: accepts 3d list and returns 3d list generated by interprolating a cubic smoothed line
        :param _3d_list:
        :return:
        '''
        x_vals, y_vals, z_vals = [], [], []
        for xyz in xyz_vals:
            x_vals.append(xyz[0])
            y_vals.append(xyz[1])
            z_vals.append(xyz[2])
        len_ = len(x_vals)
        list_len_multiplier = 3
        s = 2
        X = [x_vals,y_vals,z_vals]
        tck, u = interpolate.splprep(X, s=s)
        u_fine = np.linspace(0, 1, len_*list_len_multiplier)
        x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

        ret_ = []
        for i in range(len(x_fine)):
            ret_.append([x_fine[i],y_fine[i],z_fine[i]])
        return ret_

    @staticmethod
    def get_k_random_from_list(list,k):

        '''
        :summary: returns k random elements from the given list. returns the list if k < len(list)
        '''
        if k > len(list):
            return list
        ret_list = []
        list_cpy = list[:]
        while len(ret_list) < k:
            random_list_idx = random.randint(0, len(list_cpy)-1)
            list_item = list_cpy.pop(random_list_idx)
            ret_list.append(list_item)
        return ret_list

    @staticmethod
    def list_extender(input_list, num_midpoints):
        ret_list = []
        for i in range(len(input_list) - 1):
            l1 = input_list[i]
            l2 = input_list[i + 1]
            dx = float(l2[0] - l1[0]) / float(num_midpoints)
            dy = float(l2[1] - l1[1]) / float(num_midpoints)
            dz = float(l2[2] - l1[2]) / float(num_midpoints)
            ret_list.append(l1)
            for j in range(num_midpoints - 1):
                j += 1
                x = l1[0] + (j) * (dx)
                y = l1[1] + (j) * (dy)
                z = l1[2] + (j) * (dz)
                to_add = [x, y, z]
                ret_list.append(to_add)
        return ret_list

    @staticmethod
    def angle_between_three_points(p1, p2, p3):
        # see https://stackoverflow.com/questions/35176451/python-code-to-calcualte-angle-between-three-point-using-thier-3d-coordinates
        a = np.array(p1)
        b = np.array(p2)
        c = np.array(p3)
        ba = a - b
        bc = c - b
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        return np.degrees(np.arccos(cosine_angle))

    @staticmethod
    def add_lists(l1,l2):
        res = []
        for i in range(len(l1)):
            res.append(l1[i] + l2[i])
        return res

    @staticmethod
    def always_true_func():
        return True

    @staticmethod
    def add_scaled_vector_to_pt(P, Vect, c):
        P_ret = []
        V = MathUtils.scalar_multiply_vector(Vect, c)
        for i in range(len(V)):
            P_ret.append(V[i] + P[i])
        return P_ret

    @staticmethod
    def flip_vector(Vect):
        '''
            Rotates vector 180 degrees
        '''
        return MathUtils.scalar_multiply_vector(Vect, -1)

    @staticmethod
    def vector_adder(v1, v2):
        v3 = []
        for i in range(len(v1)):
            v3.append(v1[i] + v2[i])
        return v3

    @staticmethod
    def get_vector_between_points(P1, P2):
        V = []
        for i in range(len(P1)):
            V.append(P1[i] - P2[i])
        return V

    @staticmethod
    def get_averaged_normalized_vector(V1, V2):
        V = MathUtils.vector_adder(V1, V2)
        mag = 0
        for i in V:
            mag += i ** 2
        mag = np.sqrt(mag)
        V = MathUtils.scalar_multiply_vector(V, (1 / mag))
        return V

    @staticmethod
    def scalar_multiply_vector(Vect, c):
        V = []
        for i in Vect:
            V.append(i * c)
        return V

