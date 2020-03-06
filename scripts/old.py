
def decimal_range(start, stop, inc):
    i = start
    while i < stop:
        yield i
        i += inc

# ___________________  Hyperparameter Search

# x_range = [-5, 1]
# y_range = [-2, 2]
# robot_q0 = [-3.75, 0, 0]
# robot_qf = [-.25, 0, 0]
# world_name = "flatworld"
# visualize = False
# physics_sim_enabled = False

# x_range = [0, 18]
# y_range = [0, 2.25]
# robot_q0 = [5.75, 1.1, 0]
# robot_qf = [16.75, 1, 0]
# world_name = "drc_rough_terrain_world"

# physics_sim_enabled, physics_vis_enabled = False, False
# i = 1

# srunner = SystemRunner()
# srunner.initialize_sim_world(x_range, y_range, robot_q0, robot_qf, world_name, physics_sim_enabled, physics_vis_enabled,
#     visualize=False)
# srunner.run(run_hltraj=False)
# srunner.save_all()
#
# for base_x in decimal_range(.5, .7, .025):
#     for base_y in decimal_range(.5, .7, .025):
#         for torso_z in [.525, .6, .7]:
#             for d1 in decimal_range(.875, 1, .025):
#                 for d2 in decimal_range(.875, 1, .025):
#
#                     pconstants: ProjectConstants = ProjectConstants()
#
#                     pconstants.BASE_STATE_END_EFF_DX_FROM_TORSO = base_x
#                     pconstants.BASE_STATE_END_EFF_DY_FROM_TORSO = base_y
#                     pconstants.TORSO_Z_DESIRED = torso_z
#                     pconstants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST = d1
#                     pconstants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST_d2 = d2
#
#                     # save_file = f"data/test_results/{round(base_x, 2)}_{round(base_y, 2)}_results.p"
#                     # print(f"BASEX: {round(base_x, 2)} BASEY: {round(base_y, 2)}")
#
#                     save_file = f"data/test_results/{world_name}/results_{i}.p"
#
#                     i += 1
#
#                     if i <= 31:
#                         continue
#
#                     srunner = SystemRunner(project_constants=pconstants)
#                     srunner.initialize_sim_world(
#                         x_range, y_range, robot_q0, robot_qf, world_name, physics_sim_enabled, physics_vis_enabled,
#                         visualize=False)
#                     srunner.run(print_results=True)
#                     srunner.get_results().save(save_file)


# srunner.save_all()

# results: SystemRunnerResults = pickle.load(open(save_file, "r+b"))
# print(results.print_results())
# ___________________  Run Simulation World


x_range = [4.5, 18.25]
y_range = [0, 2.25]
# robot_q0 = [7, 1.1, 0]
# robot_qf = [16.75, 1, 0]
# robot_qf = [10.5, 1, 0]
robot_q0 = [5.75, 1.1, 0]
robot_qf = [17.25, 1, 0]
world_name = "drc_rough_terrain_world"

# x_range = [-1, 8, .015]
# y_range = [0, 3, .015]
# robot_q0 = [0, 1, 0]
# robot_qf = [6.5, 2, 0]
# world_name = "step_world_1"

# x_range = [-1, 8, .015]
# y_range = [0, 3, .015]
# robot_q0 = [0, 1, 0]
# robot_qf = [7, 2, 4]
# world_name = "step_world_2"

# x_range = [-1, 7.5]
# y_range = [0,  3]
# robot_q0 = [0, 1, 0]
# robot_qf = [6.25, 2, 0]
# world_name = "obstacle_world_3"

visualize = 1
physics_vis_enabled = False
physics_sim_enabled = False

srunner = SystemRunner()
srunner.initialize_sim_world(x_range, y_range, robot_q0, robot_qf, world_name, physics_sim_enabled, physics_vis_enabled,
                             visualize=visualize)

# srunner.delete_maps()
# srunner.delete_cost_map()
# srunner.delete_conv_cost_map()
# srunner.delete_fs_seq()
# srunner.delete_cloop_output()
# srunner.delete_scatter()

# srunner.load()
# srunner.print_cloop_output_stats()

# srunner.build_convcostmap(save=True)
# srunner.build_costmap(debug=True, print_saved=True, save=True)

# srunner.visualize_cmap_in_klampt_vis()
# srunner.visualize_conv_cmap_in_klampt_vis()
# srunner.visualize_fs_scatter()

# srunner.run(only_mapping=True)
# srunner.run(run_step_planner=False)
# srunner.run(run_mplanner=False, conv_override=True)
# srunner.run(run_mplanner=False)
srunner.run(ignore_saved_cloop=True)
# srunner.run()
# srunner.save_all()

# srunner.run()
# srunner.save_all()

# step_seq_tester = srunner.get_stepseq_tester()
# step_seq_tester.state_is_kinematically_valid()
# step_seq_tester.test_successors()
# srunner.run_physics_sim(save_qs_to_file=f"{world_name}_sim_output.txt")
# srunner.visualize(load_first=True, visualize_hltraj=False, visualize_stepseq=0)


# ___________________ Run Lidar Generated World

# robot_q0 = [5, 11, 0]
# robot_qf = [11, 11, 0]
# inbound_xrange = [4, 13]
# inbound_yrange = [8, 12]
# visualize = True
#
# pc = "pc_2"
# world_name = f"{pc}_world"
# pcd_fname = f"data/point_cloud_data/{pc}.pcd"
#
# srunner = SystemRunner()
# srunner.initialize_lidar_world(
#     world_name, pcd_fname, robot_q0, robot_qf, visualize=visualize,
#     inbound_xrange=inbound_xrange, inbound_yrange=inbound_yrange)
#
# srunner.run(only_mapping=True)
# srunner.run(lidar_mode=True)
# srunner.save_all()

# pc_parser = PCloudParser()
# pc_parser.parse_hm(pcd_fname, visualize=True)













# ------------------ From motion planner

# if debug and end_config:
#     print("\nspace.inbounds(end_config):", Logger.log_boolean( space.inBounds(end_config)), " \t hash:", Logger.log_hash(end_config))
#     print("space.closed_loop(end_config):", Logger.log_boolean( space.closedLoop(end_config)))
#     print("space.selfCollision(end_config):", Logger.log_boolean(space.selfCollision(end_config), invert=True))
#     print("space.isFeasible(end_config)", Logger.log_boolean( space.isFeasible(end_config)))
#     self.robosimian.setConfig(start_config)

# if debug:
#     print("\nspace.inbounds(start_q):", Logger.log_boolean(space.inBounds(start_config)), "\t hash:", Logger.log_hash(start_config))
#     print("space.closed_loop(start_q):", Logger.log_boolean(space.closedLoop(start_config)) )
#     print("space.selfCollision(start_q):", Logger.log_boolean(space.selfCollision(start_config), invert=True))
#     print("space.isFeasible(start_q)", Logger.log_boolean(space.isFeasible(start_config)),"\n")


# if debug:
#     print("\nspace.inbounds(start_q):", Logger.log_boolean(space.inBounds(start_config)), "\t hash:", Logger.log_hash(start_config))
#     print("space.closed_loop(start_q):", Logger.log_boolean( space.closedLoop(start_config)))
#     print("space.selfCollision(start_q):", Logger.log_boolean(space.selfCollision(start_config),  invert=True))
#     print("space.isFeasible(start_q)", Logger.log_boolean( space.isFeasible(start_config)),"\n\n")

# if debug:
#     print("\nspace.inbounds(end_config):", Logger.log_boolean(space.inBounds(end_config)), " \t hash:", Logger.log_hash(end_config))
#     print("space.closed_loop(end_config):", Logger.log_boolean(space.closedLoop(end_config)))
#     print("space.selfCollision(end_config):", Logger.log_boolean(space.selfCollision(end_config), invert=True))
#     print("space.isFeasible(end_config)", Logger.log_boolean( space.isFeasible(end_config)), "\n")
#     self.robosimian.setConfig(start_config)