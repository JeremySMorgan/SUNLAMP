from src.generators.high_level_trajectory_generator import HighLevelTrajectoryPlanner


class hl_traj_tester:

    def __init__(self, height_map, conv_fs_cost_map_obj, xy_yaw0, xy_yawf):
        self.HLTrajGenerator = HighLevelTrajectoryPlanner(height_map, conv_fs_cost_map_obj, xy_yaw0, xy_yawf)

    def test_get_cost(self,x,y,yaw):
        cost = self.HLTrajGenerator.get_cost(x,y,yaw, debug=True)
        print("cost at:",(x,y,yaw),":",cost)