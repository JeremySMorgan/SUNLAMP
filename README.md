# Overview
The objective of this project is to build a system capable of autonomously navigating the robosimian
through obstacle filled environments, while ensuring the robot does not colliside with itself or the environment and that the 
robot is static stable throughout its motion. To achieve this, three separate planners of different abstraction levels 
were designed and implemented - a high level trajectory planner, a step sequence planner, and a real time full body motion planner. 

> See *'project report.md'* for a system writeup


#### Dependencies

> Recommended to use a virtual environment for development

- python 3.6
- scipy
- numpy
- klampt
- shapely (https://pypi.org/project/Shapely/)
- matplotlib
- yattag
- progressbar `pip3.6 install progressbar2`
- PyOpenGL
- PyQt5 *both need to be installed*


#### Directory Layout

Source code is stored in /src. 
- /src/generators stores generators which produce DataObjects (found in /src/utilities/DataObjects), such as Maps, 
FootstepSequence objects, HLTrajectory objects, etc. 
- /src/lidar stores lidar classes. LidarSubscriber.py currently analyzes a saved point cloud and displays it in klampt.
- /src/motion stores all motion related classes, including ControlLoop.py, MotionPlanner.py, MotionUtils.py and 
IKSolverUtilities.py (the last two are convenience classes)
- /testing stores useful test automation utilities


#### Examples

Ex1: Run the system on "step_world_1", save and visualize the results
```python
    x_range = [-5, 5, .015]
    y_range = [-.25, 3, .015]
    robot_q0 = [-4.25, 1, 0]
    robot_qf = [3.75, 1, 0]
    world_name = "step_world_1"

    srunner = SystemRunner()
    srunner.initialize_sim_world(x_range, y_range, robot_q0, robot_qf, world_name, visualize=False)
    srunner.run()
    srunner.save_all()
```

Ex2: load and visualize the results of Ex1
```python
    x_range = [-5, 5, .015]
    y_range = [-.25, 3, .015]
    robot_q0 = [-4.25, 1, 0]
    robot_qf = [3.75, 1, 0]
    world_name = "step_world_1"

    srunner = SystemRunner()
    srunner.initialize_sim_world(x_range, y_range, robot_q0, robot_qf, world_name, visualize=True)
    srunner.visualize(ignore_hltraj=True, ignore_step_seq=True)
```

Ex4: Run only the generators and HL, and Step Sequence planners
```python
    x_range = [-5, 5, .015]
    y_range = [-.25, 3, .015]
    robot_q0 = [-4.25, 1, 0]
    robot_qf = [3.75, 1, 0]
    world_name = "step_world_1"

    srunner = SystemRunner()
    srunner.initialize_sim_world(x_range, y_range, robot_q0, robot_qf, world_name, visualize=False)
    srunner.run(run_splanner=True, run_hltplanner=True, run_mplanner=False)   # Note they are all true by default
    srunner.save_all()
    srunner.get_results().print_results()
```

When saving an output, SystemRunner will hash the worldname, and all relevent hyperparameters for each output object to create a file name.
When running, SystemRunner will check to see if the output has already been performed by performing the same hash. If a file exists, it loads the file
instead of reperforming the same calculation. This functionality can be disabled by adding SystemRunner.\<obj\>_type to the SystemRunner.default_stettings\[dont_load_settings_key\] array.



#### To Do


- [x] add slope cost to cmap. Currently disabled bc doesn;t work
- [x] Robot too low. -> save max distance vals, then raise torso z
- [x] find distance that q143 is especially large at
- [x] Known jumps between configs queued to execution robot. likely bc of fix to !closedLoop(startconfig) issue.
- [x] add joint constaints
> joint constraints added, do not prevent low manueverability stances

- [x] add more sample worlds
- [x] add testing suite
- [x] randomized test suite worlds

- [ ] slerp outputting non rotations
- [ ] 'reset' procedure to return robot to good state. Idea is to reset legs into manuevable regions
- [ ] 
- [ ] Default stance should be wider in y, possibly x aswell
- [ ] Stats on how similar to default stance robot is during normal steps
- [ ] Redefind nominal stance


##### Algorithmic Areas of Interest
- Hyperparameter optimization
- End configuration optimization
- Reuse config space map (even w/ new obstacles?)
- Motion primitives to speed up motion planning?


##### Known Bugs

1. When setting the motion planner's endpoints, the start configuration will occassionally not be closed loop ( space.closed_loop(start_q) == False.) even though
  the same configuration WAS closed loop when it was treated as the end configuration. For example: 
    ```
    q_idx: 45
    torso_com_validator(end_config): True , 97533
    space.inbounds(end_config): True , 97533
    space.closed_loop(end_config), hash: True , 97533
    space.selfCollision(end_config), hash: False , 97533
    space.isFeasible(end_config), hash True , 97533 
    ...    
    q_idx: 46
    torso_com_validator(start_q), hash: True , 97533
    space.inbounds(start_q), hash: True , 97533
    space.closed_loop(start_q), hash: False , 97533
    space.selfCollision(start_q), hash: False , 97533
    space.isFeasible(start_q), hash: False , 97533
    ```

    *note that hash: the last 5 characters of the hashed config*

    There is currently a very 'hacky' solution implemented to fix this bug: the torso is moved by randomly choosen between [-2,2]cm
    in the x, y, and z direction until the config is closed loop. The start config is then set to this new config. See `MotionPlanner.py` line 184. 
    This issue should be investigated further.

2. 'ValueError: insecure string pickle'. Error thrown by pickle occasionally. Not sure why. Simple solution is to recalculate 
    whatever object is trying to be loaded
    
    
3. Need rigid objects in world files, klampt vis fails o.w.