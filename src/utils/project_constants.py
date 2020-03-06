import math


class ProjectConstants:

    END_EFFECTOR_HEIGHT = .139 - .001#.139 - .005
    END_AFFECTOR_RADIUS = .05  # 5 cm - radius of the end effector
    SHOULDER_X = .293260631869
    SHOULDER_Y = .206456211
    SHOULDER_Z = -0.2085299999988
    LEG_LENGTH = .775
    SHOULDER_TORSO_XY_EUCLIDEAN_DIF = 0.3603472707
    TORSO_W = 2 * SHOULDER_Y
    TORSO_L = .47 * 2
    TORSO_d = (TORSO_L - 2 * SHOULDER_X) / 2  # distance from shoulder to front/back of robot measured along side
    SHOULDER_TORSO_PSI_RADS = .620089205322
    HOOK_LENGTH = .155 - .0175
    HOOK_DIST_TO_GROUND = .015 -.0025
    TORSO_LINK_INDEX = 5
    FL_ACTIVE_DOFS = [31, 32, 33, 34, 35, 36, 37]
    FR_ACTIVE_DOFS = [7, 8, 9, 10, 11, 12, 13]
    BR_ACTIVE_DOFS = [15, 16, 17, 18, 19, 20, 21]
    BL_ACTIVE_DOFS = [23, 24, 25, 26, 27, 28, 29]
    ACTIVE_DOFS = FL_ACTIVE_DOFS + FR_ACTIVE_DOFS+  BR_ACTIVE_DOFS + BL_ACTIVE_DOFS
    PATH_TO_DUKE_API = '/home/motion/rockclimber/platform/caesar/interface/duke_api'

    PHYSICS_SIM_CONTROLLER_DT = .01

    # Optimizable variables are mutable
    def __init__(self):

        # Height Map GeneratorSuperclass Variables
        self.HM_X_GRANULARITY = .01
        self.HM_Y_GRANULARITY = .01

        # Footstep Cost GeneratorSuperclass variables
        self.CMAP_STEP_SIZEX = .04
        self.CMAP_STEP_SIZEY = .04
        self.CMAP_NORMALIZED_MAX_VALUE = 1.0

        self.CMAP_ROUGHNESS_HEURISTIC_COEFF = 7.5

        self.CMAP_STEP_COSTFN_DIF_SLOPE_COEFF = 2.0
        self.CMAP_STEP_COSTFN_SEARCH_SIZE = .9*ProjectConstants.END_AFFECTOR_RADIUS
        self.CMAP_STEP_COSTFN_MIN_HEIGHT_DIF = .01
        self.CMAP_STEP_COSTFN_BASELINE_COST = 1.5

        self.CMAP_SLOPE_HEURISTIC_COEFF = 0.01
        self.CMAP_MAX_NONPENALIZED_SLOPE = 20
        self.CMAP_SLOPE_HEURISTIC_MAX_CONSIDERED_DEGREE = 60   # certain overhangs will be ~65deg,

        self.COST_MAP_SEARCH_MARGIN_WIDTH = .01

        #  Convolution  Variables
        self.CONV_X_WIDTH = .015
        self.CONV_Y_WIDTH = .015
        self.USE_CONVMAP = False

        # Base State
        self.BASE_STATE_END_EFF_DX_FROM_TORSO = .67 # Defines the base state
        self.BASE_STATE_END_EFF_DY_FROM_TORSO = .61
        self.TORSO_Z_DESIRED = .65

        # Robot Measurements
        self.DELTA_Z_SHOULDER_END_AFFECTOR = 0.980182 * (-self.TORSO_Z_DESIRED) + .20965
        self.END_AFFECTOR_RADIUS_TO_SHOULDER = math.sqrt(ProjectConstants.LEG_LENGTH ** 2 - self.DELTA_Z_SHOULDER_END_AFFECTOR ** 2)

        # High Level Trajectory Planner
        self.HLTRAJ_G_WEIGHT = 1.0
        self.HLTRAJ_H_WEIGHT = 5.0
        self.HLTRAJ_YAW_OFFSET = 3.0
        self.HLTRAJ_DELTAX = .2
        self.HLTRAJ_DELTAY = .2
        self.HLTRAJ_GOAL_THRESHOLD = self.HLTRAJ_DELTAX + self.HLTRAJ_DELTAY + .2
        self.HLTRAJ_BASE_COST = 0.0
        self.HLTRAJ_NB_POINTS_BTWN_PATH_NODES = 5
        self.SEARCH_SPACE_X_MARGIN = .25
        self.SEARCH_SPACE_Y_MARGIN = .25

        # Footstep Scatter
        self.X_STEP_SIZE_FS_SCATTER = .06
        self.Y_STEP_SIZE_FS_SCATTER = .06

        # _________________Step planner parameters
        #                                                    moving_leg_to_com_dist_next
        #  base x|base y|torso z| max_fh_to_com_fhincenter| max_fh_to_com_fhincenter 2 | max diag dist |translation dist| zerocost radius
        #        |       | .525 |         .94             |            .965            |               |                |
        #   .6   |  .55  |  .7  |         .93             |            .955            |      1.95     |       .15      |      .075
        #   .6   |  .55  |  .7  |         .93             |            .951            |      1.95     |       0        |      .075
        #   drc: fails on first brick layer
        #   .67  |  .61  |  .7  |         .93             |            .951            |      1.95     |       0        |      .075
        #   Not Enough Manueverability
        #   .67  |  .61  |  .6  |         .92             |            .951            |      1.9      |       0        |      .075
        #   taking ages for drc step planning, steps too small
        #   .67  |  .61  |  .65  |        .92             |            .951           |      1.9      |       .1      |      .15  |
        # ---
        #   .55  |  .5  |  .7  |         .92              |            .951           |      1.9      |       .0      |      .15
        #   works really well on flatground. fails on first block row on drc

        #   .55  |  .5  |  .7  |         .92              |            .951           |      1.9      |       .0      |      .1  |
        #   works well on flatground. Gets really contorted on drc, fails around 85

        #   .55  |  .6  |  .7  |         .92              |            .951           |      1.9      |       .1      |      .1  |
        #   works well on flatground, very long steps. fails on drc

        #   .55  |  .6  |  .7  |         .92              |            .951           |      1.9      |       .05      |      .1  |
        #   works on flatground - looks a bit awkward. failed first run on drc @43, but didn't fail when reran w/ stance_idx, q


        # _ kinematic constraints
        self.STEPSEQ_MAX_FEASIBLE_LEG_TRANSLATION = .65
        self.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS = .75
        self.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_COMTRI_DIAG_MIDPOINT = .3
        self.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS = .35
        self.STEPSEQ_NONPENALIZED_WIDTH_FROM_BASESTATE_Y = .2
        self.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST = .92
        self.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST_d2 = .951
        self.STEPSEQ_MAX_DIAGNOL_DIST = 1.9
        self.STEPSEQ_D12_3D_MULTIPLIER = 1.05 #1.02
        self.STEPSEQ_USE_Z_IN_MAX_DIST_CALCS = 1    # set to 0 or 1
        self.STEPSEQ_HEIGHT_AROUND_ENDEFF_R = .165

        # _ Gait params
        self.STEPSEQ_TRANSLATION_DISTANCE = 0.1
        self.STEPSEQ_ZERO_COST_RADIUS = .15
        self.STEP_ORDER = [3, 2, 4, 1]

        # _ Search Params
        self.STEPSEQ_G_WEIGHT = 1.0
        self.STEPSEQ_H_WEIGHT = 1.0
        self.STEPSEQ_GOAL_THRESHOLD = .5

        # _ Heuristic weights
        self.STEPSEQ_HEIGHT_NEAR_ENDEFF_COST_COEFF = 5.0
        self.STEPSEQ_DIST_FROM_HL_TRAJ_COST_COEFF = .75
        self.STEPSEQ_TERRAIN_COST_COEFF = .6
        self.STEPSEQ_IDEAL_LOCATION_COST_COEFF = .8
        self.STEPSEQ_PREDICTED_HOOK_COLLISION_COST = 15.0
        self.STEPSEQ_HOOK_SAFETY_MARGIN = .01

        # _________________Motion params
        # self.NOMINAL_CONFIG = [
        #                         0, 0.0, self.TORSO_Z_DESIRED,                          # torso x y z
        #                         0, 0, 0,                                               # torso roll pitch yaw
        #                         0, 0, 0, -0.785398, -1.5707, -1.5707, 0.785398, 0,     # fr
        #                         0, 0, 0, 0.785398, 1.5707, 1.5707, -0.785398, 0,       # br
        #                         0, 0, 0, -0.785398, -1.5707, -1.5707, 0.785398, 0,     # bl
        #                         0, 0, 0, 0.785398, 1.5707, 1.5707, -0.785398, 0,       # fl
        #                    ]

        c = .4
        self.NOMINAL_CONFIG = [
                                0, 0.0, self.TORSO_Z_DESIRED,                          # torso x y z
                                0, 0, 0,                                               # torso roll pitch yaw
                                0, 0, 0, -0.785398 - c, -1.5707, -1.5707, 0.785398 + c, 0,     # fr
                                0, 0, 0, 0.785398 + c, 1.5707, 1.5707, -0.785398 - c, 0,       # br
                                0, 0, 0, -0.785398 - c, -1.5707, -1.5707, 0.785398 + c, 0,     # bl
                                0, 0, 0, 0.785398 + c , 1.5707, 1.5707, -0.785398 -c , 0,       # fl
                           ]

        self.MIN_TORSO_CLEARANCE = .25
        self.STEP_HEIGHT = .35

        self.MOTION_PLANNER_SAMPLE_COUNT = 500
        self.MOTION_PLANNER_RETRY_COUNT = 2
        self.KLAMPT_MPLANNER_ALGO = "sbl"       # rrt, prm, sbl, sblprt

        self.END_CONFIG_SAMPLE_COUNT = 100
        self.CONTROLLER_DT = .0075

        # _ Safety Margins
        self.END_RANGE_MULTIPLIER = 1.15
        self.SUPPORT_TRIANGLE_SAFETY_MARGIN = .05