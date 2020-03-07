from scipy import optimize

import numpy as np

from src import system_runner
from src.utils.data_objects.optimization_output import OptimizationResults
from src.utils.data_objects.system_runner_results import SystemRunnerResults
from src.utils.logger import Logger
from src.utils import project_constants


class HyperparameterOptimizer:

    '''
    x = [BASE_STATE_END_EFF_DX_FROM_TORSO, BASE_STATE_END_EFF_DY_FROM_TORSO, TORSO_Z_DESIRED,
            HLTRAJ_H_WEIGHT, STEPSEQ_H_WEIGHT, STEP_SEQ_COST_PARABOLA_SLOPE, STEPSEQ_TRANSLATION_DISTANCE ]
    '''

    HL_TRAJ_FAILED = 25
    STEP_SEQ_FAILED = 15

    def __init__(self):
        pass

    def optimize_for_world(self, x_vars, y_vars, xy_yaw0, xy_yawf, world_name, method="minimize"):

        opt_results = OptimizationResults(x_vars, y_vars, xy_yaw0, xy_yawf,world_name)

        def paremeter_vector_cost(x):

            tsuite = system_runner(x_vars, y_vars, xy_yaw0, xy_yawf, world_name)
            tsuite.run()
            results = tsuite.get_results()

            if results.hl_traj_runtime == SystemRunnerResults.FAILED:
                cost = HyperparameterOptimizer.HL_TRAJ_FAILED
            elif results.fs_seq_runtime == SystemRunnerResults.FAILED:
                cost = HyperparameterOptimizer.STEP_SEQ_FAILED
            else:
                if results.cloop_results[0] == SystemRunnerResults.LOADED_FROM_PICKLE:
                    Logger.log("Error: cloop loaded from pickle.", "FAIL")
                    return False
                elif len(results.cloop_results)  == 1:
                    cost = results.cloop_results[0]
                else:
                    cost = 0

            opt_results.optimization_vectors.append(x)
            opt_results.test_suite_results.append(results)
            opt_results.optimization_vector_costs.append(cost)
            results.print_results()

            print(" (", method, ") parameters:", x)
            print("\n                     cost:",cost,"\n\n")
            return cost

        x0 = np.array([.593, .5, 0.5, 1.18518, 4.94816, 4.0, 0.125])
        bounds = [(.25,.65),(.25,.65),(.2,.75),(0.0,100.0),(0.0,25.0),(0.0,25.0),(0.0,1.25)]
        minimizer_kwargs = {'bounds':bounds}

        # Run once to save maps
        tsuite = system_runner(x_vars, y_vars, xy_yaw0, xy_yawf, world_name)
        tsuite.run()
        tsuite.save_all()

        if method == "minimize":
            ret = optimize.minimize(paremeter_vector_cost, x0, bounds=bounds)
        elif method == "basinhopping":
            ret = optimize.basinhopping(paremeter_vector_cost, x0, minimizer_kwargs=minimizer_kwargs)
        else:
            print("unrecognized method:",method)
            return

        opt_results.result = ret
        print(ret)
        opt_results.save("opt_results")


