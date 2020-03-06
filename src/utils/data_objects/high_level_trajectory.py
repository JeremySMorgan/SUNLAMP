import pickle as pickle


class HLTrajectory:

    def __init__(self):

        self.xy_yaw0 = None
        self.xy_yawf = None

        self.xy_yaw_path = None

        self.higher_density_xy_yaw_path = None
        self.ave_higher_density_xy_yaw_path_distance_change = None

        self.smoothed_path = None
        self.ave_smooth_path_distance_change = None

    def get_xy_yaw0(self):
        return self.xy_yaw0

    def get_xy_yawf(self):
        return self.xy_yawf

    def get_higher_density_xy_yaw_path(self):
        return self.higher_density_xy_yaw_path

    def get_xy_yaw_path(self):
        return self.xy_yaw_path

    def get_smoothed_xy_yaw_path(self):
        return self.smoothed_path

    def save(self,save_file,print_=True):
        pickle.dump(self, open(save_file+".pickle", "wb"))
        if print_: print("hl trajectory saved")