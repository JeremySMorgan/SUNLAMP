from src.utils.data_objects.output_superclass import OutputSuperclass


class HLTrajectory(OutputSuperclass):

    def __init__(self):

        OutputSuperclass.__init__(self, "control loop output")

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

    def print_stats(self):
        print("<HL_Trajectory Obj>")
        print(f"      failed:\t\t{self.failed}")
        print(f"      runtime:\t\t{round(self.runtime, 2)}")
        print(f"      len path:\t\t{len(self.xy_yaw_path)}")
        print()