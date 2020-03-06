from klampt import WorldModel
from klampt import vis
import klampt
import time

world = WorldModel()

world_file = "../data/simulation_test_worlds/drc_rough_terrain_world.xml"
if not world.readFile(world_file):
    raise RuntimeError("Unable to load terrain model")

robot_file = "../data/robot_model/robosimian_caesar_new.rob"
if not world.readFile(robot_file):
    raise RuntimeError("Unable to load robot model")

vis_window_id = vis.createWindow(world_file)
vis.setWindow(vis_window_id)

vis.add("world", world)
vp = vis.getViewport()
vis.setViewport(vp)
vis.autoFitCamera()

world.robot(0).setConfig([12.621508747630084, 1.3060978650033888, 0.7271994997360561, -0.18389666460947365, -0.2336561986984183, 0.23915345995072382, 0.0, 0.12877367095232392, -0.24152711808937907, -1.352324085841938, -1.3962990011600755, -1.6288249134647879, 1.0952385369647921, 1.5131291235190494, 0.0, 1.302668674636621, -2.910148719749822, -0.2345765710424345, -0.27406908444556055, -0.05762017461060415, -1.3599420305102727, -1.4753241574471778, 0.0, -0.5972895800911506, 0.21328357279079702, -1.714578772288262, -3.0575450624726135, -1.7653233213810249, 1.701274900771741, 0.5276099427465047, 0.0, -1.2047695561654546, 0.7348791016475654, -3.1415, 1.5796965965960093, 0.32064381624016175, -2.7024078026665306, -1.8221735905151801])

world.readFile(robot_file)
world.robot(1).setConfig([10, 1.3060978650033888, 0.7271994997360561, -0.18389666460947365, -0.2336561986984183, 0.23915345995072382, 0.0, 0.12877367095232392, -0.24152711808937907, -1.352324085841938, -1.3962990011600755, -1.6288249134647879, 1.0952385369647921, 1.5131291235190494, 0.0, 1.302668674636621, -2.910148719749822, -0.2345765710424345, -0.27406908444556055, -0.05762017461060415, -1.3599420305102727, -1.4753241574471778, 0.0, -0.5972895800911506, 0.21328357279079702, -1.714578772288262, -3.0575450624726135, -1.7653233213810249, 1.701274900771741, 0.5276099427465047, 0.0, -1.2047695561654546, 0.7348791016475654, -3.1415, 1.5796965965960093, 0.32064381624016175, -2.7024078026665306, -1.8221735905151801])


vis.show()
while vis.shown():
    time.sleep(.01)
vis.kill()


# No Robot
'''***  klampt.vis: using Qt5 as the visualization backend  ***
klampt.vis: auto-fitting camera to scene.
Exception occurred during fitting to points
[[1.25, 2.75, 0.053]]
[[1.2, 2.7, 0.0030000000000000027], [1.3, 2.8, 0.253]]
Unable to auto-fit camera
Point set is degenerate
INITIALIZING Qt BACKEND
WARNING: QApplication was not created in the main() thread.
vis: creating GL window
INITIALIZING Qt BACKEND
######### QGLWidget setProgram ###############
#########################################
klampt.vis: Making window 1
#########################################
######### QGLWidget Initialize GL ###############'''