
from klampt.sim.simlog import SimLogPlayback
from klampt import Simulator
from klampt import vis
from klampt import WorldModel
import time


if __name__ == "__main__":
    world_name = "flatworld"
    world_file = "../data/simulation_test_worlds/" + world_name + ".xml"

    world = WorldModel()

    if not world.readFile(world_file):
        raise RuntimeError("Unable to load terrain model")

    if not world.readFile("../data/robot_model/robosimian_caesar_new.rob"):
        raise RuntimeError("Unable to load robot")

    robot = world.robot(0)

    vis.createWindow(world_name + " physics sim replay world")
    vis.add("world", world)
    vis.autoFitCamera()
    vis.show()
    time.sleep(2)

    # print(len(robot.getConfig()))
    q = [0] * 38
    last_time = 0
    count = 0
    with open("../flatworld_sim_output.txt", "r") as f:

        for line in f:

            t = float(line.split(" [")[0])
            str_ = line.split(" [")[1]
            str_ = str_.replace("[", "")
            str_ = str_.replace("]", "")
            comma_split = str_.split(",")

            for i in range(len(q)):
                q[i] = float(comma_split[i])

            robot.setConfig(q)

            time.sleep(t - last_time)
            last_time = t
            if count % 10 == 0:
                print("t:",round(t,2))
            count += 1

    print("done")
    while vis.shown():
        time.sleep(.01)
    vis.kill()