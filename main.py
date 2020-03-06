import sys
import time
import os
from klampt import vis
from scripts.world_builder import WorldBuilder
from src.system_runner import SystemRunner
from src.utils.logger import Logger
from src.lidar.pcloud_parser import PCloudParser
from src.utils.project_constants import ProjectConstants
from src.utils.data_objects.system_runner_results import SystemRunnerResults


def main():

    # TODO: decrease step distance

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
    visualize = True
    execution_world_enabled = True
    physics_sim_enabled = False

    srunner = SystemRunner()
    srunner.initialize_sim_world(
        x_range, y_range, robot_q0, robot_qf, world_name, execution_world_enabled, physics_sim_enabled,
        visualize=visualize, cloop_run_name=cloop_run_name)

    # srunner.run(ignore_saved_cloop=True)
    # srunner.run(run_mplanner=False)
    # srunner.run(run_mplanner=False)
    # srunner.run(run_splanner=False)
    # srunner.run(ignore_saved_splan=True)
    # srunner.run(only_mapping=True)
    # srunner.run()
    # srunner.save_all()
    # srunner.visualize(visualize_stepseq=False, visualize_hltraj=False, skip_to_pct_through_cloop_output=0)
    # srunner.visualize_cmap_in_klampt_vis(step=7)



    # ___________________ Run Lidar Generated World


    shutdown()


def run_lidar_world():
    robot_q0 = [5, 11, 0]
    robot_qf = [11, 11, 0]
    inbound_xrange = [4, 13]
    inbound_yrange = [8, 12]
    visualize = True

    pc = "pc_2"
    world_name = f"{pc}_world"
    pcd_fname = f"data/point_cloud_data/{pc}.pcd"

    srunner = SystemRunner()
    srunner.initialize_lidar_world(
        world_name, pcd_fname, robot_q0, robot_qf, visualize=visualize,
        inbound_xrange=inbound_xrange, inbound_yrange=inbound_yrange)

    srunner.run(only_mapping=True)
    srunner.run(lidar_mode=True)
    srunner.save_all()

    pc_parser = PCloudParser()
    pc_parser.parse_hm(pcd_fname, visualize=True)


def buid_new_world():

    world_name = "very_hectic_world"
    x_range = [-5, 5, .015]
    y_range = [-2.25, 2.25, .015]
    robot_q0 = [-4, 0, 0]
    robot_qf = [4, .5, 0]

    mu_width, mu_length, mu_height = .65, .6, .05
    var_width, var_length, var_height = .62, .6, .1
    mu_rX, mu_rY, mu_rZ = .0, .1, .0
    var_rX, var_rY, var_rZ = .01, .1, .5
    cubestats = [
        mu_width, mu_length, mu_height, var_width, var_length, var_height, mu_rX, mu_rY, mu_rZ, var_rX, var_rY, var_rZ]
    ncubes = 65
    wb = WorldBuilder(world_name, ncubes, cubestats, x_range, y_range, robot_q0, robot_qf)
    wb.build_world()
    wb.vis_world()


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





'''
failed at: 143 (exhaustive search failed)

ran succesfully in 2917.6363s (48 min) (think disable sleep wasn't working)

Failed after 1513.82943s, at stance_idx:143 (plan_step())

'''



