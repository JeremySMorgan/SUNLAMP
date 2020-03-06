
# x_range = [-5, 5]
# y_range = [-.5, 3]
# robot_q0 = [-4, 1, 0]
# robot_qf = [3.75, 1, 0]
# world_name = "hectic_world"

# x_range = [-5, 3]
# y_range = [-.5, 3]
# robot_q0 = [-4, 1, 0]
# robot_qf = [1.75, 1, 0]
# world_name = "slant_world"

# x_range = [-5, 2.5]
# y_range = [-0.5, 3]
# robot_q0 = [-4, 1, 0]
# robot_qf = [1, 1, 0]
# world_name = "sparse_obstacle_world"

# x_range = [-5, 5]
# y_range = [-.5, 3]
# robot_q0 = [-4, 1, 0]
# robot_qf = [4, 1.75, 0]
# world_name = "medium_diff_slant_world"

# x_range = [-5, 3, .015]
# y_range = [-.25, 3, .015]
# robot_q0 = [-4.25, 1, 0]
# robot_qf = [1.75, 1, 0]
# world_name = "flatworld"


# x_range = [-5, 3, .015]
# y_range = [-.25, 3, .015]
# robot_q0 = [-4.25, 1, 0]
# robot_qf = [1.75, 1, 0]
# world_name = "easy_world"

# x_range = [-1, 8, .015]
# y_range = [0, 3, .015]
# robot_q0 = [0, 1, 0]
# robot_qf = [6.5, 2, 0]
# world_name = "step_world_1"

# x_range = [-1, 8, .015]
# y_range = [0, 3, .015]
# robot_q0 = [0,1,0]
# robot_qf = [7,2,4]
# world_name = "step_world_2"

# x_range = [-1, 5]
# y_range = [0,  3]
# robot_q0 = [0, 1, 0]
# robot_qf = [4, 2, 0]
# world_name = "obstacle_world_2"

# x_range = [-1, 7.5]
# y_range = [0,  3]
# robot_q0 = [0, 1, 0]
# robot_qf = [6.25, 2, 0]
# world_name = "obstacle_world_3"

# x_range = [0, 20]
# y_range = [0, 2.5]
# robot_q0 = [.75, 1, 0]
# robot_qf = [18, 1, 0]
# world_name = "drc_rough_terrain_world"

# x_range = [-1.5, 7]
# y_range = [5, 8]
# robot_q0 = [-.5, 6, 0]
# robot_qf = [6, 7, -5]
# world_name = "fractal_world_1"

# fractal_world1 = SystemRunner(x_range, y_range, robot_q0, robot_qf, world_name, vis_on_init=vis_on_init)
#
# x_range = [-2, 7.5, .015]
# y_range = [0, 3, .015]
# robot_q0 = [.25, 1.2, 0]
# robot_qf = [ 6.25, 2, -5]
# world_name = "fractal_world_4"
# fractal_world4 = SystemRunner(x_range, y_range, robot_q0, robot_qf, world_name, run_mplanner=0, vis_on_init=vis_on_init)
#
# test_suites = [ obst_world, flat_world, step_world1, step_world2, fractal_world1, fractal_world3, fractal_world4 ]
# test_suites = [ obst_world, flat_world, step_world1, step_world2 ]
# tsrunner = TestingSuiteRunner(test_suites)
# tsrunner.run(visualize=True, print_results=True, save=True, test_hltraj=True, test_step_planner=False, test_mplanner=False)


# ------------- Slant world contstruction
# mu_width, mu_length, mu_height = .65, .6, .05
# var_width, var_length, var_height = .62, .6, .1
# mu_rX, mu_rY, mu_rZ = .0, .1, .0
# var_rX, var_rY, var_rZ = .01, .1, .5
# cubestats = [
#     mu_width, mu_length, mu_height, var_width, var_length, var_height, mu_rX, mu_rY, mu_rZ, var_rX, var_rY, var_rZ]
# ncubes = 75
# wb = WorldBuilder(world_name, ncubes, cubestats, x_range, y_range, robot_q0, robot_qf)
# wb.build_world()
# wb.vis_world()

#
# mu_width, mu_length, mu_height = .9, .8, .05
# var_width, var_length, var_height = .3, .3, .075
# mu_rX, mu_rY, mu_rZ = .01, .005, 1.2
# var_rX, var_rY, var_rZ = .01, .1, .25
# cubestats = [mu_width, mu_length, mu_height, var_width, var_length, var_height, mu_rX, mu_rY, mu_rZ,var_rX, var_rY, var_rZ]
# ncubes = 25
# wb = WorldBuilder(world_name, ncubes, cubestats, x_range, y_range, robot_q0, robot_qf)
# wb.build_world()