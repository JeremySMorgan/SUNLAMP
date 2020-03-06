import pickle as pickle
from src.utils.logger import Logger


class ControlLoopOutput:

    UNFILLED = -1

    def __init__(self):
        self.configs = []
        self.end_stance_idx = ControlLoopOutput.UNFILLED
        self.ik_errors = ControlLoopOutput.UNFILLED
        self.self_collisions = ControlLoopOutput.UNFILLED
        self.failed = ControlLoopOutput.UNFILLED
        self.dist_to_end = ControlLoopOutput.UNFILLED
        self.dist_from_start_to_end = ControlLoopOutput.UNFILLED
        self.runtime = ControlLoopOutput.UNFILLED

    def print_stats(self):

        print("_______________________")
        print("<ControlLoopOutput Obj>")
        print("  runtime:", round(self.runtime,2))
        print("  # configs:", len(self.configs))
        print("  success:", Logger.log_boolean(not self.failed))
        print()

    def save(self, save_file,print_=True):
        pickle.dump(self, open(save_file + ".pickle", "wb"))
        if print_:
            print("control loop output saved")