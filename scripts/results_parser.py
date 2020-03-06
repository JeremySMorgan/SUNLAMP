import sys
import time
import os
from klampt import vis
import pickle
from src.system_runner import SystemRunner
from src.utils.logger import Logger
from src.lidar.pcloud_parser import PCloudParser

from src.utils.data_objects.system_runner_results import SystemRunnerResults

for i in range(1,41):
    save_file = f"../data/test_results/results_{i}.p"
    results: SystemRunnerResults = pickle.load(open(save_file, "r+b"))

    print(results.print_results())