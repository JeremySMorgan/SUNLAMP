from yattag import Doc, indent
import numpy as np
from klampt import WorldModel
from klampt import vis

doc, tag, text = Doc().tagtext()


# convienece (<- missspeclled <- (also misspelled <- ... (recursion XD))) classes
class Terrain(object):
    default_file = "terrains/plane.tri"
    default_translation = "0 0 0"
    def __init__(self, terrain_file=default_file, translation=default_translation):
        self.file = terrain_file
        self.translation = translation

    def get_tag(self):
        with tag('terrain',file=self.file, translation=self.translation):
            pass


class DefaultTags(object):

    @staticmethod
    def default_physics_tag():
        return tag("physics", mass="1.0", automass="1", kRestitution="0.0", kFriction="0.0", kStiffness="10000.00", kDamping="100.00")


class Cube(object):
    default_mesh = "objects/cube.off"
    def __init__(self, x, y, z, scale_x, scale_y, scale_z, rotatex=0,rotatey=0,rotatez=0, mesh_file=default_mesh):
        self.x = x
        self.y = y
        self.z = z
        self.rotatex = rotatex
        self.rotatey = rotatey
        self.rotatez = rotatez
        self.scale_x = scale_x
        self.scale_y = scale_y
        self.scale_z = scale_z
        self.mesh_file = mesh_file

    def get_tag(self):
        with tag("rigidObject", name="block", position=str(self.x)+" "+str(self.y)+" "+str(self.z), rotateX=str(self.rotatex), rotateY=str(self.rotatey), rotateZ=str(self.rotatez) ):
            with tag("geometry", mesh=self.mesh_file, scale=str(self.scale_x)+" "+str(self.scale_y)+" "+str(self.scale_z),
                     translation="0 0 0"):
                pass
            with DefaultTags.default_physics_tag():
                pass


class WorldBuilder(object):

    '''
        cubestats = [mu_width, mu_length, mu_height, var_width, var_length, var_height] or
                    [mu_width, mu_length, mu_height, var_width, var_length, var_height, mu_rX, mu_rY, mu_rZ,  var_rX, var_rY, var_rZ]

    '''
    def __init__(self, worldname, ncubes, cubestats, xrange, yrange, xy0, xyf, terrain_file=None, terrain_translation=None):
        self.worldname = worldname
        self.world_file = "data/simulation_test_worlds/"+self.worldname+".xml"
        self.robot_file = "data/robot_model/robosimian_caesar_new.rob"
        self.ncubes = ncubes
        self.cubestats = cubestats
        self.xrange = xrange
        self.yrange = yrange
        self.xy0 = xy0
        self.xyf = xyf
        self.cubez = 0
        self.terrain_file = terrain_file
        self.terrain_translation = terrain_translation

    def get_cubes(self):

        cubes = []

        min_dist_from_goal = 0.0
        min_dist_from_start = 1.0

        width_dist = np.random.normal(self.cubestats[0], self.cubestats[3], self.ncubes)
        length_dist = np.random.normal(self.cubestats[1], self.cubestats[4], self.ncubes)
        height_dist = np.random.normal(self.cubestats[2], self.cubestats[5], self.ncubes)

        if len(self.cubestats) > 6:
            rX_dist = np.random.normal(self.cubestats[6], self.cubestats[9], self.ncubes)
            rY_dist = np.random.normal(self.cubestats[7], self.cubestats[10], self.ncubes)
            rZ_dist = np.random.normal(self.cubestats[8], self.cubestats[11], self.ncubes)
        else:
            rX_dist = np.zeros((1,self.ncubes))[0]
            rY_dist = np.zeros((1,self.ncubes))[0]
            rZ_dist = np.zeros((1,self.ncubes))[0]

        for i in range(self.ncubes):

            x = float(np.random.uniform(self.xrange[0], self.xrange[1], size=1))
            y = float(np.random.uniform(self.yrange[0], self.yrange[1], size=1))

            dist_to_start = np.sqrt((x-self.xy0[0])**2 + (y-self.xy0[1])**2)
            # dist_to_end = (x-self.xyf[0])**2 + (y-self.xyf[1])**2

            while dist_to_start < min_dist_from_start: # or dist_to_end < min_dist_from_goal:
                x = float(np.random.uniform(self.xrange[0], self.xrange[1], size=1))
                y = float(np.random.uniform(self.yrange[0], self.yrange[1], size=1))
                dist_to_start = np.sqrt((x-self.xy0[0])**2 + (y-self.xy0[1])**2)
                # dist_to_end = (x-self.xyf[0])**2 + (y-self.xyf[1])**2

            w = max(width_dist[i], .075)
            l = max(length_dist[i], .075)
            h = max(height_dist[i], .05)

            rX = rX_dist[i]
            rY = rY_dist[i]
            rZ = rZ_dist[i]

            cubes.append(Cube(x, y, self.cubez, w, l, h, rotatex=rX, rotatey=rY, rotatez=rZ))
        return cubes

    def build_world(self):

        cubes = self.get_cubes()
        if self.terrain_file is not None and self.terrain_translation is not None:
            terrain = Terrain(self.terrain_file, self.terrain_translation)
        elif self.terrain_file is not None:
            terrain = Terrain(terrain_file=self.terrain_file)
        else:
            terrain = Terrain()

        with tag('world'):
            terrain.get_tag()
            for cube in cubes:
                cube.get_tag()

            # simulation tags abandonded, for now (until running simulation tests)
            # with tag("simulation"):
            #     with tag("globals"):
            #         pass
            #     with tag("object",index="0"):
            #         with tag("geometry", kRestitution="0.0", kFriction="0.01",kStiffness="10000.00",kDamping="100.00",padding="0.002"):
            #             pass
            #         with tag("velocity", linear="1 0 0"):
            #             pass

        result = indent(
            doc.getvalue(),
            indentation=' ' * 4,
            newline='\r\n'
        )

        xml_header = '<?xml version="1.0" encoding="UTF-8"?>\n'
        res = xml_header + result

        f = open(self.world_file, "w")
        f.write(res)
        f.close()

    def vis_world(self):

        world = WorldModel()
        self.rposer = None
        res = world.readFile(self.world_file)
        if not res:
            raise RuntimeError("Unable to load terrain model")
        res = world.readFile(self.robot_file)
        if not res:
            raise RuntimeError("Unable to load robot model")

        vis.createWindow(self.worldname)
        vis.setWindowTitle(self.worldname + " visualization")

        vis.add("world", world)
        vp = vis.getViewport()
        # vp.w, vp.h = 800, 800
        vis.setViewport(vp)
        vis.autoFitCamera()
        vis.show()

        q = [ 0, 0.0, .5,                 # torso x y z
                0, 0, 0,                                   # torso roll pitch yaw
                0, 0, 0, -0.785398, -1.5707, -1.5707, 0.785398, 0,     # fr
                0, 0, 0, 0.785398, 1.5707, 1.5707, -0.785398, 0,       # br
                0, 0, 0, -0.785398, -1.5707, -1.5707, 0.785398, 0,     # bl
                0, 0, 0, 0.785398, 1.5707, 1.5707, -0.785398, 0,       # fl
            ]

        world.robot(0).setConfig(q)

