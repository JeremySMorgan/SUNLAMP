from src.utils.logger import Logger


class TestingSuiteRunner:

    def __init__(self, test_suites=None):
        self.test_suites = [] if test_suites is None else test_suites

    def add_test_suites(self,test_suites):
        for ts in test_suites:
            self.test_suites.append(ts)

    def run(self, visualize=False, print_results=True, save=True, test_hltraj=True, test_step_planner=True, test_mplanner=True):

        msg = "Starting test suites for " + str(len(self.test_suites))+" worlds"
        Logger.log(msg, "OKBLUE")

        for test_suite in self.test_suites:
            print("\n  --   --  \n ")
            msg = "Running test suite for "+Logger.bold_txt(test_suite.world_name)
            Logger.log(msg, "")
            print("\n  --   --  \n ")
            test_suite.test_hltraj = test_hltraj
            test_suite.test_step_planner = test_step_planner
            test_suite.test_mplanner = test_mplanner
            test_suite.run(print_results=print_results)
            if save:
                test_suite.save_all()
            if visualize:
                test_suite.visualize()
        Logger.log("Done", "OKGREEN")

    def save_all(self):
        for test_suite in self.test_suites:
            test_suite.save_all()



