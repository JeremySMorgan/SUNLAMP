# !/usr/bin/env python
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud
from klampt import vis
import klampt
from klampt import robotsim
from klampt.robotsim import Geometry3D
from klampt.model import coordinates

# Plot distribution
import scipy.stats as stats
from matplotlib.ticker import NullFormatter


class PCloudSubscriber:

    '''
    Notes: to run in pycharm, launch pycharm from terminal: 'charm'
    '''

    def __init__(self,disable_ros=False):
        if not disable_ros:
            rospy.init_node('pcloud_listener')
            rospy.Subscriber("/pcloud", PointCloud, self.pcloud_callback)
        self.pcloud = klampt.PointCloud()
        self.np_cloud = None

    def pcloud_callback(self, reading):
        for point32 in reading.points:
            self.pcloud.addPoint([point32.x, point32.y, point32.z])

    def listen_for(self,d):
        print("starting listener")
        start_t = time.time()
        done = False
        while not done and not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)
            if time.time() > start_t + d:
                done = True

        num_poins = len(self.pcloud.vertices) / 3
        self.np_cloud = np.zeros(shape=(num_poins,3))
        for i in range(0,len(self.pcloud.vertices)-3,3):
            point = i / 3
            self.np_cloud[point,0] = self.pcloud.vertices[i]
            self.np_cloud[point,1] = self.pcloud.vertices[i+1]
            self.np_cloud[point,2] = self.pcloud.vertices[i+2]
        print("done")

    def save_pcloud(self,save_file):
        print(self.np_cloud.shape,": size of numpy cloud")
        self.np_cloud.tofile(save_file)
        print("pcloud saved")

    def visualize(self):
        world = robotsim.WorldModel()
        vis.add("world", world)
        vis.add("coordinates", coordinates.manager())
        vis.setWindowTitle("PointCloud World")
        vp = vis.getViewport()
        vp.w, vp.h = 800, 800
        vis.setViewport(vp)
        #vis.autoFitCamera()
        vis.show()

        vis.lock()
        vis.unlock()

        while vis.shown():
            time.sleep(.01)
        vis.kill()

    def stats(self):
        return str(self.pcloud.numPoints())+": number of points in pcloud"

    def load_pcloud(self,save_file):

        '''
        Converts a geometry to another type, if a conversion is available. The
        interpretation of param depends on the type of conversion, with 0
        being a reasonable default.
            Available conversions are:
                PointCloud -> TriangleMesh, if the point cloud is structured. param is the threshold
                                for splitting triangles by depth discontinuity, by default infinity.
        '''

        long_np_cloud = np.fromfile(save_file)
        print(long_np_cloud.shape,": shape of long numpy cloud")

        num_points = long_np_cloud.shape[0] / 3
        np_cloud = np.zeros(shape=(num_points, 3))
        pcloud = klampt.PointCloud()
        scaling_factor = .1
        points = []
        xs = []
        ys = []
        zs = []

        for x in range(num_points):
            i = x*3
            x_val = long_np_cloud[i] * scaling_factor
            y_val = long_np_cloud[i+1] * scaling_factor
            z_val = long_np_cloud[i+2] * scaling_factor
            np_cloud[x][0] = x_val
            np_cloud[x][1] = y_val
            np_cloud[x][2] = z_val
            xs.append(x_val)
            ys.append(y_val)
            zs.append(z_val)
            points.append(np_cloud[x])

        points.sort(key=lambda tup: tup[2])

        x_sorted = sorted(xs)  # sorted
        y_sorted = sorted(ys)  # sorted
        z_sorted = sorted(zs)  # sorted

        xfit = stats.norm.pdf(x_sorted, np.mean(x_sorted), np.std(x_sorted))
        yfit = stats.norm.pdf(y_sorted, np.mean(y_sorted), np.std(y_sorted))
        zfit = stats.norm.pdf(z_sorted, np.mean(z_sorted), np.std(z_sorted))

        # plot with various axes scales
        plt.figure(1)

        # linear
        plt.subplot(221)
        plt.plot(x_sorted, xfit)
        plt.hist(x_sorted, normed=True)
        plt.title('X values')
        plt.grid(True)

        plt.subplot(222)
        plt.plot(y_sorted, yfit)
        plt.hist(y_sorted, normed=True)
        plt.title('Y values')
        plt.grid(True)

        plt.subplot(223)
        plt.plot(z_sorted, zfit)
        plt.hist(z_sorted, normed=True)
        plt.title('Z values')
        plt.grid(True)

        # Format the minor tick labels of the y-axis into empty strings with
        # `NullFormatter`, to avoid cumbering the axis with too many labels.
        plt.gca().yaxis.set_minor_formatter(NullFormatter())
        # Adjust the subplot layout, because the logit one may take more space
        # than usual, due to y-tick labels like "1 - 10^{-3}"
        plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25,
                            wspace=0.35)

        #plt.show()


        median_z = np.median(zs)
        threshold = .25 * scaling_factor
        for point in points:
            if np.fabs(point[2] - median_z) < threshold:
                pcloud.addPoint(point)

        print(pcloud.numPoints(),": num points")

        # Visualize
        pcloud.setSetting("width", "3")
        pcloud.setSetting("height",str(len(points)/3))
        g3d_pcloud = Geometry3D(pcloud)
        mesh = g3d_pcloud.convert("TriangleMesh", 0)

        world = robotsim.WorldModel()
        vis.add("world", world)
        vis.add("coordinates", coordinates.manager())
        vis.setWindowTitle("PointCloud World")
        vp = vis.getViewport()
        vp.w, vp.h = 800, 800
        vis.setViewport(vp)
        vis.autoFitCamera()
        vis.show()

        vis.lock()
        vis.add("mesh",mesh)
        vis.unlock()

        while vis.shown():
            time.sleep(.01)
        vis.kill()

        print("done")



if __name__ == '__main__':

    listen_d = 1
    save_file = "test.pcl"

    pcloud_subscriber = PCloudSubscriber(disable_ros=True)
    #pcloud_subscriber.listen_for(10)
    #pcloud_subscriber.save_pcloud(save_file)
    #pcloud_subscriber.visualize()
    pcloud_subscriber.load_pcloud(save_file)



    '''
    Attributes: vertices: a list of vertices, given as a list [x1, y1, z1, x2, y2, ... zn]

    properties: a list of vertex properties, given as a list [p11, p21, ..., pk1, p12, p22, ..., pk2, ... , pn1, pn2, ..., pn2] 
    where each vertex has k properties. The name of each property is given by the propertyNames member.
    '''

    '''
        The point cloud must be structured, which means it has properties "width" and "height". 
        Additionally, there must be width x height points in scan line order, with missing data denoted as 0 0 0.
    '''
    #
    # points = [
    #     [1.47947514057, -1.4282643795, 0.373247712851],
    #     [1.49710404873, -1.3478281498, 0.35222736001],
    #     [1.50019907951, -1.25909936428, 0.329039901495],
    #     [1.42877399921, -1.11711573601, 0.291935384274],
    #     [1.46256947517, -1.06416726112, 0.278098374605],
    #     [0.398368835449, -0.269339948893, 0.0703864991665],
    #     [0.417903065681, -0.262051969767, 0.0684819370508],
    #     [0.445102781057, -0.258244156837, 0.0674868300557],
    #     [0.478639513254, -0.256185561419, 0.0669488683343],
    #     [0.518779039383, -0.25523224473, 0.0666997358203],
    #     [0.576619923115, -0.259619534016, 0.0678462684155],
    #     [0.616011798382, -0.252463132143, 0.0659760907292],
    #     [0.665824234486, -0.246756494045, 0.0644847676158],
    #     [0.717816054821, -0.23860630393, 0.0623548813164],
    #     [1.04449081421, -0.308243483305, 0.0805531442165],
    #     [0.713936448097, -0.184628769755, 0.0482489615679],
    #     [1.03881776333, -0.231422007084, 0.0604774169624],
    #     [1.0376906395, -0.194580689073, 0.0508496910334],
    #     [1.02925539017, -0.15719370544, 0.041079364717],
    #     [1.02833890915, -0.121672213078, 0.0317965485156],
    #     [1.33893847466, -0.112745374441, 0.0294637028128],
    #     [0.666100502014, -0.0335031636059, 0.00875536724925],
    #     [0.61890989542, -0.0102198421955, 0.00267074699514],
    #     [0.621901333332, 0.0107165733352, -0.00280055752955],
    #     [1.37808537483, 0.0703078657389, -0.0183735247701],
    #     [1.33287453651, 0.11320053786, -0.0295826476067],
    #     [1.28133630753, 0.152541905642, -0.0398637130857],
    #     [1.26913392544, 0.194764778018, -0.050897795707],
    #     [1.27410399914, 0.239862054586, -0.0626830533147],
    #     [1.32801687717, 0.296853929758, -0.0775767117739],
    #     [1.89894425869, 0.492543101311, -0.128716081381],
    #     [1.8953397274, 0.560830652714, -0.146561637521],
    #     [1.84750199318, 0.615606307983, -0.160876125097],
    #     [1.96329498291, 0.729222476482, -0.190567404032],
    #     [1.93766248226, 0.795765638351, -0.207957088947],
    #     [1.86067759991, 0.839386820793, -0.219356611371],
    #     [1.94780421257, 0.960056722164, -0.250891238451],
    #     [1.87266111374, 1.0040769577, -0.262394994497],
    #     [1.91677451134, 1.11396813393, -0.291112810373],
    #     [2.02644014359, 1.272777915, -0.332614511251],
    #     [1.50826311111, 1.02136230469, -0.266912192106],
    #     [1.50089132786, 1.09374082088, -0.285826832056],
    #     [1.99146401882, 1.55943500996, -0.407526493073],
    #     [1.16934013367, 0.982887983322, -0.256857693195],
    #     [1.12516033649, 1.01448154449, -0.26511400938],
    #     [1.11833250523, 1.08122837543, -0.282556951046],
    #     [1.93222129345, 2.00317382812, -0.523488521576],
    #     [1.9067710638, 2.12042212486, -0.554129004478],
    #     [1.89845943451, 2.26613473892, -0.59220802784],
    #     [2.13058924675, 2.73276782036, -0.714153051376],
    #     [1.99062979221, 2.74752259254, -0.718008875847],
    #     [1.95690572262, 2.91196250916, -0.760981917381],
    #     [1.83994793892, 2.95879721642, -0.773221254349],
    #     [1.68609166145, 2.93870568275, -0.767970740795],
    #     [1.53734087944, 2.91451501846, -0.761649012566],
    #     [1.60489761829, 3.3240032196, -0.868660330772],
    #     [1.51778995991, 3.45269036293, -0.90229010582],
    #     [1.19818639755, 3.01327514648, -0.787457942963],
    #     [1.02588903904, 2.87535452843, -0.751415193081],
    #     [0.897604942322, 2.83236336708, -0.740180373192],
    #     [0.780187129974, 2.8076646328, -0.733725786209],
    #     [0.674916863441, 2.81719613075, -0.736216664314],
    #     [0.563011705875, 2.78887343407, -0.72881513834],
    #     [0.458544254303, 2.78419518471, -0.727592527866],
    #     [0.355840057135, 2.78164792061, -0.726926863194],
    #     [0.254144072533, 2.7784678936, -0.726095855236],
    #     [0.152524471283, 2.76119875908, -0.721582889557],
    #     [0.0527010001242, 2.75305700302, -0.719455242157],
    #     [-0.0462354086339, 2.73478055, -0.714679062366],
    #     [-0.144558921456, 2.7276930809, -0.71282684803],
    #     [-0.242428079247, 2.71730232239, -0.710111439228],
    #     [-0.339965850115, 2.70555090904, -0.70704048872],
    #     [-0.438504040241, 2.7000837326, -0.70561170578],
    #     [-0.537818312645, 2.69504857063, -0.704295933247],
    #     [-0.333749353886, 1.40694344044, -0.367675930262],
    #     [-0.384765148163, 1.39669883251, -0.364998728037],
    #     [-0.632796466351, 2.0122885704, -0.525870501995],
    #     [-0.798640668392, 2.25422048569, -0.589094519615],
    #     [-0.892960548401, 2.26025009155, -0.590670228004],
    #     [-0.780344307423, 1.78585362434, -0.466696381569],
    #     [-0.797467589378, 1.66104459763, -0.434080064297],
    #     [-0.899844288826, 1.71509110928, -0.448204040527],
    #     [-0.94225114584, 1.65066242218, -0.431366920471],
    #     [-1.15783786774, 1.87105417252, -0.488961786032],
    #     [-2.32525777817, 3.47651767731, -0.908516943455],
    #     [-0.924718618393, 1.28221392632, -0.335080444813],
    #     [-0.912119090557, 1.17519426346, -0.307113021612],
    #     [-2.12762904167, 2.55094742775, -0.666638016701],
    #     [-2.26344394684, 2.52808141708, -0.660662472248],
    #     [-1.29196596146, 1.34522926807, -0.351548194885],
    #     [-1.35942316055, 1.32001805305, -0.344959765673],
    #     [-1.26233828068, 1.14311146736, -0.298728853464],
    #     [-2.90014338493, 2.44838309288, -0.639834940434],
    #     [-3.10688686371, 2.44365739822, -0.638599932194],
    #     [-3.2919485569, 2.40974831581, -0.629738509655],
    #     [-3.45286607742, 2.34898614883, -0.613859534264],
    #     [-3.6811580658, 2.32304811478, -0.60708117485],
    #     [-3.94584751129, 2.30445194244, -0.602221488953],
    #     [-4.20995569229, 2.26880908012, -0.5929068923],
    #     [-4.37549448013, 2.16819119453, -0.566612482071],
    #     [-4.37220144272, 1.98353278637, -0.5183557868],
    #     [-4.42539167404, 1.82837402821, -0.477808207273],
    #     [-0.000932791444939, 0.000348706293153, -9.11272727535e-05],
    #     [-0.000944790954236, 0.000317027181154, -8.28485790407e-05],
    #     [-0.000955641560722, 0.00028496250161, -7.44691351429e-05],
    #     [-0.000965329993051, 0.00025255131186, -6.59991274006e-05],
    #     [-0.000973844551481, 0.000219832974835, -5.74488585698e-05],
    #     [-0.000981174758635, 0.000186847304576, -4.88287296321e-05],
    #     [-9.32022285461, 1.45030879974, -0.379008591175],
    #     [-9.29141139984, 1.12587761879, -0.294225126505],
    #     [-9.26159763336, 0.806118667126, -0.210662648082],
    #     [-9.22311115265, 0.48990637064, -0.128027021885],
    #     [-9.21816158295, 0.178144484758, -0.0465544201434],
    #     [-9.17998027802, -0.132370099425, 0.0345922224224],
    #     [-6.79257631302, -0.327397793531, 0.0855587273836],
    #     [-9.15310668945, -0.751441776752, 0.196373954415],
    #     [-2.55563521385, -0.296953976154, 0.0776028558612],
    #     [-2.23509144783, -0.336563527584, 0.0879540070891],
    #     [-2.20785307884, -0.409209728241, 0.106938607991],
    #     [-2.29803681374, -0.506882309914, 0.132463335991],
    #     [-2.34893727303, -0.602187216282, 0.157369330525],
    #     [-7.15448904037, -2.09503293037, 0.547494053841],
    #     [-7.79961061478, -2.57440137863, 0.672767221928],
    #     [-7.15721559525, -2.63532447815, 0.688688278198],
    #     [-7.24774837494, -2.95250797272, 0.771577656269],
    #     [-3.09654569626, -1.38632488251, 0.362287700176],
    #     [-5.02304506302, -2.45805478096, 0.642362415791],
    #     [-2.8069331646, -1.49471187592, 0.390612453222],
    #     [-2.85526585579, -1.64848303795, 0.430797368288],
    #     [-3.63866019249, -2.27087473869, 0.593446791172],
    #     [-3.40848469734, -2.29390025139, 0.599463999271],
    #     [-3.1100063324, -2.2526717186, 0.588689744473],
    #     [-2.8352663517, -2.20701909065, 0.576759397984],
    #     [-2.67832517624, -2.23808264732, 0.584877192974],
    #     [-2.47720503807, -2.22055006027, 0.58029538393],
    # ]
    #
    # pcloud = klampt.PointCloud()
    # for i in range(len(points)):
    #     pcloud.addPoint(points[i])
    # pcloud.setSetting("width", "3")
    # pcloud.setSetting("height", "45")
    # g3d_pcloud = Geometry3D(pcloud)
    # mesh = g3d_pcloud.convert("TriangleMesh",0)
    #
    # world = robotsim.WorldModel()
    # vis.add("world", world)
    #
    #
    # vis.add("coordinates", coordinates.manager())
    # vis.setWindowTitle("PointCloud World")
    # vp = vis.getViewport()
    # vp.w, vp.h = 800, 800
    # vis.setViewport(vp)
    # # vis.autoFitCamera()
    # vis.show()
    #
    # vis.lock()
    # vis.add("mesh",mesh)
    # vis.unlock()
    #
    # while vis.shown():
    #     time.sleep(.01)
    # vis.kill()

























