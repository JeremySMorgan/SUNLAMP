import pickle as pickle
import time as picklerick
from src.utils.vis_utils import VisUtils
from src.utils.data_objects.output_superclass import OutputSuperclass


class FootstepSequence(OutputSuperclass):

    def __init__(self):

        OutputSuperclass.__init__(self, "footstep sequence")
        self.xy_yaw0 = None
        self.xy_yawf = None
        self.state_path = None

    def visualize(self, fs_scatter, r_poser, sleep_t_p_step=.5):

        # time_ = float(in_t) if in_t > 0 else 20.0
        # sleep_t = time_ / len(self.stance_path)
        sleep_t_p_step = .5 if sleep_t_p_step < 0 else sleep_t_p_step

        for q in self.state_path:

            fl = fs_scatter[q[0]]
            fr = fs_scatter[q[1]]
            br = fs_scatter[q[2]]
            bl = fs_scatter[q[3]]
            VisUtils.visualize_cost(fl[0], fl[1], .45, name="front left", with_z=fl[2])
            VisUtils.visualize_cost(fr[0], fr[1], .45, name="front right", with_z=fr[2])
            VisUtils.visualize_cost(br[0], br[1], .45, name="back right", with_z=br[2])
            VisUtils.visualize_cost(bl[0], bl[1], .45, name="back left", with_z=bl[2])
            r_poser.set_q(q)
            picklerick.sleep(sleep_t_p_step)

    def get_xy_yaw0(self):
        return self.xy_yaw0

    def get_xy_yawf(self):
        return self.xy_yawf

    def get_state_path(self):
        return self.state_path

    def print_stats(self):
        print("<FS_Sequence Obj>")
        print(f"      failed:\t\t{self.failed}")
        print(f"      runtime:\t\t{round(self.runtime, 2)}")
        print(f"      nb states:\t\t{len(self.state_path)}")
        print()