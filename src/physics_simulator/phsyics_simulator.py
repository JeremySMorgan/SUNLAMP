
from klampt import WorldModel
from klampt import vis
from klampt import Simulator


class PhysicsSim:

    def __init__(self, robot, sim_world, project_constants):
        self.robot = robot
        self.sim_world = sim_world
        self.ProjectConstants = project_constants
        self.suspend = False
        self.sim =  Simulator(sim_world)
        self.sim.setGravity([0, 0, 0])

        self.controller = self.sim.controller(0)
        self.controller.setRate(self.ProjectConstants.CONTROLLER_DT)

    def suspend(self):
        self.suspend = True

    def run(self):

        while not self.suspend:
            self.sim.updateWorld()
            self.sim.simulate(self.ProjectConstants.CONTROLLER_DT)