import sys
import time
import os
from klampt import vis

import argparse
from scripts.world_builder import WorldBuilder
from src.system_runner import SystemRunner
from src.utils.logger import Logger
from src.lidar.pcloud_parser import PCloudParser
from src.utils import config
from src.utils.data_objects.system_runner_results import SystemRunnerResults


def main():


    # ___________________  Run Simulation World

    # obstacle_world[1-3], medium_diff_slant_world really easy

    # x_range = [-5, 5]
    # y_range = [-.5, 3]
    # robot_q0 = [-4, 1, 0]
    # robot_qf = [3.75, 1, 0]
    # world_name = "hectic_world"

    # x_range = [4.5, 18.25]
    # y_range = [0, 2.25]
    # robot_q0 = [5.75, 1.1, 0]
    # robot_qf = [17.25, 1, 0]
    # world_name = "drc_rough_terrain_world"

    x_range = [-5, 5, .015]
    y_range = [-.25, 3, .015]
    robot_q0 = [-4.25, 1, 0]
    robot_qf = [3.75, 1, 0]
    world_name = "flatworld"

    # world_name = "very_hectic_world"
    # x_range = [-5, 5, .015]
    # y_range = [-2.25, 2.25, .015]
    # robot_q0 = [-4, 0, 0]
    # robot_qf = [4, .5, 0]

    cloop_run_name = ""
    execution_world_enabled = True

    srunner = SystemRunner()
    srunner.initialize_sim_world(
        x_range, y_range, robot_q0, robot_qf, world_name, execution_world_enabled=execution_world_enabled,
        visualize=False)

    srunner.run()
    srunner.save_all()

    # srunner.run(ignore_saved_cloop=True)
    # srunner.run(run_mplanner=False)
    # srunner.run(run_mplanner=False)
    # srunner.run(ignore_saved_splan=True)
    # srunner.run(only_mapping=True)
    # srunner.run()
    # srunner.save_all()
    # srunner.visualize(visualize_stepseq=False, visualize_hltraj=False, skip_to_pct_through_cloop_output=0)
    # srunner.visualize_cmap_in_klampt_vis(step=7)

    shutdown()


def shutdown(u_input=None):
    if u_input:
        u_input.shutdown()
    try:
        if vis.shown():
            Logger.log("Exiting when vis window closed", "")
        while vis.shown():
            time.sleep(.01)
        Logger.log("Shutting down", "FAIL")
        vis.kill()
    except TypeError:
        Logger.log("Shutting down", "FAIL")
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)


if __name__ == "__main__":
    main()
