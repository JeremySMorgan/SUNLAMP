import pickle as pickle


class OptimizationResults:

    OPTIMIZATION_SUCCESS = -2
    OPTIMIZATION_FAIL = -3
    OPTIMIZATION_UNFINISHED = -4

    def __init__(self, x_vars, y_vars, xy_yaw0, xy_yawf, world_name):
        self.x_vars = x_vars
        self.y_vars = y_vars
        self.xy_yaw0 = xy_yaw0
        self.xy_yawf = xy_yawf
        self.world_name = world_name

        self.optimization_vectors = []
        self.test_suite_results = []
        self.optimization_vector_costs = []

        self.success = OptimizationResults.OPTIMIZATION_UNFINISHED

        self.result = None

    def save(self,save_file,print_=True):
        pickle.dump(self, open(save_file+".pickle", "wb"))
        if print_:print("optimization results sequence saved")