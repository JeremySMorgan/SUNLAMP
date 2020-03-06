import pickle
from src.utils.logger import Logger
from src.utils.data_objects.controlloop_output import ControlLoopOutput


class SystemRunnerResults:

    # Default value
    UNFILLED = -1

    # Indicates a calculation was not performed - for example if hl traj fails, the step search and mplanner results should
    # be filled with NEVER_CALCULATED
    NEVER_CALCULATED = -2

    # Inidicates if a calculation failed
    FAILED = -3

    # data obj. was loaded from pickle file
    LOADED_FROM_PICKLE = -4

    def __init__(self):

        self.world_name = None

        self.project_constants = None

        self.hm_runtime = SystemRunnerResults.UNFILLED
        self.gradient_map_runtime = SystemRunnerResults.UNFILLED
        self.fs_cost_map_runtime = SystemRunnerResults.UNFILLED
        self.fs_convcost_map_runtime = SystemRunnerResults.UNFILLED
        self.fs_scatter_runtime = SystemRunnerResults.UNFILLED
        self.hl_traj_runtime = SystemRunnerResults.UNFILLED
        self.fs_seq_runtime = SystemRunnerResults.UNFILLED
        self.control_loop_output_obj = SystemRunnerResults.UNFILLED

    def print_results(self):
        title_str = "\ntest suite results for: "+Logger.styled_text(self.world_name,"BOLD")
        print(title_str)
        print("----------------------")
        print("  ", self.print_util("height map runtime", self.hm_runtime))
        print("  ", self.print_util("gradient map runtime", self.gradient_map_runtime))
        print("  ", self.print_util("cost map runtime", self.fs_cost_map_runtime))
        print("  ", self.print_util("conv. cost map runtime", self.fs_convcost_map_runtime))
        print("  ", self.print_util("fs scatter runtime", self.fs_scatter_runtime))
        print("  ", self.print_util("hl trajectory runtime", self.hl_traj_runtime))
        print("  ", self.print_util("step sequence runtime", self.fs_seq_runtime))
        print("  ", self.print_util("control loop", self.control_loop_output_obj))
        print("\n")

    @staticmethod
    def print_util(item_name, res):
        obj_str = Logger.styled_text(item_name, "bold")
        if res == SystemRunnerResults.UNFILLED:
            return obj_str + ": unfilled"
        if res == SystemRunnerResults.NEVER_CALCULATED:
            return obj_str + ": not calculated"
        if res == SystemRunnerResults.FAILED:
            return obj_str + ": "+Logger.styled_text("Failed","FAIL")
        if res == SystemRunnerResults.LOADED_FROM_PICKLE:
            return obj_str + ": "+Logger.styled_text("Loaded from pickle","BLUE")
        if type(res) is list:
            if res[0] == SystemRunnerResults.UNFILLED:
                return obj_str + ": unfilled"
            if res[0] == SystemRunnerResults.NEVER_CALCULATED:
                return obj_str + ": not calculated"
            if res[0] == SystemRunnerResults.FAILED:
                return obj_str + ": " + Logger.styled_text("Failed", "FAIL")
            if res[0] == SystemRunnerResults.LOADED_FROM_PICKLE:
                return obj_str + ": " + Logger.styled_text("Loaded from pickle", "BLUE")
            return obj_str +": "+ Logger.pp_list(res)
        if isinstance(res, ControlLoopOutput):
            if res.failed:
                return obj_str + ": " + Logger.styled_text("Failed","FAIL")+" after " + Logger.pp_double(res.runtime) +\
                       "s, at stance_idx:" + str(res.end_stance_idx)

            return obj_str + ": in " + Logger.pp_double(res.runtime) + "s"

        return obj_str + ": "+str(Logger.pp_double(res))

    def save(self, fname):
        pickle.dump(self, open(fname, "wb"))