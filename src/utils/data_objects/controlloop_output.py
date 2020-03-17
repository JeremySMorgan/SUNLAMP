import pickle as pickle
from src.utils.logger import Logger
from src.utils.data_objects.output_superclass import OutputSuperclass


class ControlLoopOutput(OutputSuperclass):

    def __init__(self):

        OutputSuperclass.__init__(self, "control loop output")

        self.configs = []
        self.end_stance_idx = ControlLoopOutput.UNFILLED
        self.nb_stances = ControlLoopOutput.UNFILLED
        self.ik_errors = ControlLoopOutput.UNFILLED
        self.self_collisions = ControlLoopOutput.UNFILLED
        self.failed = ControlLoopOutput.UNFILLED
        self.dist_to_end = ControlLoopOutput.UNFILLED
        self.dist_from_start_to_end = ControlLoopOutput.UNFILLED
        self.runtime = ControlLoopOutput.UNFILLED

    def print_stats(self):
        # print("_______________________")
        print("<ControlLoopOutput Obj>")
        print(f"      failed:\t\t\t{self.failed}")
        print(f"      runtime:\t\t\t{round(self.runtime,2)}")
        print(f"      nb configs:\t\t", len(self.configs))
        print(f"      end stance idx:\t\t", self.end_stance_idx)
        print(f"      nb stances:\t\t", self.nb_stances)
        print(f"      dist to end:\t\t", round(self.dist_to_end, 4))
        print(f"      total dist:\t\t", round(self.dist_from_start_to_end, 4))
        print()
