# Overview
The objective of this project is to build a system capable of autonomously navigating the robosimian
through obstacle filled environments, while ensuring the robot does not colliside with itself or the environment and that the 
robot is static stable throughout its motion. To achieve this, three separate planners of different abstraction levels 
were designed and implemented - a high level trajectory planner, a step sequence planner, and a real time full body motion planner. 

Cost Map
--------
The foothold cost map stores the cost to place an end effector at each $`(x,y)`$ coordinate in the search grid. The cost 
at each point is the sum of a slope, roughness, and step height heuristic. These three heuristic's are fairly common in 
step planning literature, and can be found in papers such as 'Stereo Camera Based Navigation of Mobile Robots on Rough Terrain' by 
Annett Chilian and Heiko Hirschmüller, and 'Navigation Planning for Legged Robots in Challenging Terrain', written by 
Wermelinger, et al. 

The slope heuristic calculates the average slope in the x and y direction in a search grid of width equal to the radius 
of the robots end effector plus a safety margin (ProjectConstants.END_AFFECTOR_RADIUS + ProjectConstants.COST_MAP_SEARCH_MARGIN_WIDTH)
at the specified $`(x,y)`$ coordinate. If the x or y slope is greater than a specified maximum allowable slope, stored as  
ProjectConstants.CMAP_MAX_NONPENALIZED_SLOPE, the function returns a cost linearly proportional to the difference of the 
calculated slope and the max allowable slope. If the slope is greater than a predefined value, stored as 
ProjectConstants.CMAP_SLOPE_HEURISTIC_MAX_CONSIDERED_DEGREE, it is presumed to be as a result of a large height discrepancy 
in the world, cause by example, the edge of a platform, and is thus ignored as such a feature falls outside the scope of the heuristic.

The roughness heuristic returns the cumulative difference between the x and y slope at all points inside the search grid 
described above versus the averaged x and y slopes in the same region. 

The step height heuristic returns the difference in height between the point in question and the point in the search 
grid that has the largest height difference. This difference is weighted linearly. This heuristic is to weight areas
that are heigher or lower than the surroinding area. For example if there is a rectangular prism a meter high, this heuristic
will ensure the top is very costly to step on, even if it is flat and would otherwise make a good foothold. 


To find the slope at a point the gradient is first calculated. The calculation is given as:

```math 
grad_x(x,y) = \frac{ h(x+\Delta x,y) - h(x-\Delta x,y)}{2\Delta x}
```
```math
grad_y(x,y) = \frac{ h(x,y+\Delta y) - h(x,y-\Delta y)}{2\Delta y}
```
```math 
\text{where } h(x,y) \text{ is the height at }  (x,y)
``` 
The slope, measured in degrees, in the x and y direction is then calculated as:
```math
 slope_x(x,y) = a*\text{arctan} (b*grad_x(x,y))
```
```math
 slope_x(x,y) = a*\text{arctan} (b*grad_x(x,y))
```
```math
 \text{where } a = 57.352, b = 99.36, \text{ found with regression tools. } R^2=.9999
```

Once the cost map has been built it is normalized by dividing all costs by the maximum cost. This is done to keep weighting
standardized. 

Scatter List
------------
A scatter-list is a list of potential foot step holds and associated foot-step hold costs. The costs are referenced from 
the cost map. Each $`(x,y)`$ scatter point is spaced by a predetermined granularity (ProjectConstants.(X/Y)_STEP_SIZE_FS_SCATTER) 
away from other points.
```math
\text{Scatter List: } [(x_0,y_0,z_0,c_0), ... ,(x_i,y_i,z_i,c_i),...,(x_n,y_n,z_n,c_n)]
```

High Level Trajectory Planner
-----------------------------

The high level trajectory planner creates a route for the robot to follow in order to arrive at its target destination. 
The planner accepts a starting and end coordinate describing the position and heading of the torso, $`(x_{t,0},y_{t,0},\psi_{t,0})`$, 
and $`(x_{t,f},y_{t,f},\psi_{t,f})`$, respectively, and returns a list of coordinates, given as
$`[(x_{t,0},y_{t,0},\psi_{t,0}), ... ,(x_{t,i},y_{t,i},\psi_{t,i}),...,(x_{t,f},y_{t,f},\psi_{t,f})]`$, for the torso to follow, 
representing the calculated route. The coordinates are calculated by running the A\* search algorithm. The cost at a state 
is given as the sum of the costs of the positions the end effectors would take if the robot were posed in its
base state, defined by ProjectConstants.END_EFFECTOR_DELTA_X/Y_FROM_TORSO at the `x,y` coordinates specified by the state 
and rotated `\psi` rads. The heuristic function returns the distance between the current state, and the target state, 
where the difference in heading is scaled by 1/25. This is because heading is measured in degrees, whereas x and y 
coordinates are measured in meters. The functions are weighted by $`w_h`$, and $`w_g`$, the traveling and heuristic
weights, respectivly. These variables are set in ProjectConstants.HL_TRAJ_(G/H)_WEIGHT. Reasonable values were found to 
be $`1.0`$ and $`1.5`$.  Increasing the $`w_g`$ to $`h`$ ratio will result in an increased avoidance of obstacles, 
possibly at the expense of a contorted path. If the opposite is true, the path will efficiently navigate to the target, 
with the risk of running into avoidable obstacles.

Step Sequence Planner
---------------------

The step sequence planner generates a list of foot step holds the robot will take as it travels along the high level trajectory. The holds are
generated by applying the A\* search procedure.

Other notable functions include State-Is-Kinematically-Valid(q), Get-Next-Leg-To-Move(leg$`_{current}`$), and Get-Estimated-Torso-XY-Yaw(q). As the name suggests,
State-Is-Kinematically-Valid filters kinematically invalid states. This is done by ensuring the line segments defined by the left and right end effectors, and front 
and back end effectors do not cross. It also returns false if the distance between end effectors does not fall into acceptable ranges.
Get-Next-Leg-To-Move($`leg_{current}`$) returns the next leg to move in the step order. The provided step order is \[3,2,4,1\], defined by ProjectConstants.STEP_ORDER, indicating that
the back right, then front right, then back left, then front left steps. This order was found to provide large support areas in all stages of the
gait, although the order has yet to be rigorously tested and there may be more stable step orders to follow. 

The function Get-Estimated-Torso-XY-Yaw(q) returns the torso's estimated $`(x,y,\psi)`$ from a given state. This is performed by averaging the $`x`$ and $`y`$
coordinates of the end effectors foot holds, and taking the $`\psi`$ commanded by $`v`$, where $`v=v_{l,n}+v_{r,n}`$. The vector $`v_{l,n}`$ points
from the back left to the front left end effector, and $`v_{r,n}`$ is the vector from the back right to front right end effector.

To evaulate the cost of a state, the planner creates a hybrid cost map, created by adding a parobola of slope ProjectConstants.STEP_SEQ_COST_PARABOLA_SLOPE centered at 
an idealized position to the footstep cost map, and returns the cost at the end effector position at that point in the hyrid cost map. 
The idealized end effector position is found by shifting the robot, when posed in its base state, along the high level trajectory by a predefined
distance $`dL`$, and returning the position the particular end effector would take at that base state. The $`dL`$ parameter is clearly very important, 
as it defines the speed of the robot gait, and greatly impacts the margin, and existence of the robots stability region.
This is a largely naive cost function, as it does not account for either 1) the position of the other end effectors, or 2) the gait
implemented by the motion planner. The cost function can lead to over stretched and comparatively unstable robot poses if one leg is
staggered, or especially far to the side. In complex environments with numerous obstacles this in an issue however generally this method
effectively generates a feasible and stable step sequence pattern.

The planners run time is largely determined by the length of the provided scatter-list. To this end the $`x`$ and $`y`$ granularities of the scatter-list 
are set to $`.15`$(m). This value clearly rather large, however it tends to result in the generation of around 100 successors from any given state, 
which is in the desired range for a successor function. 


Motion Planner and Control Loop
-------------------------------
The control loop takes as input the step sequence, and is tasked with generating non colliding, statically stable, and kinematic feasible
configurations to leading the robot from the initial state to the end state. To this end, the motion planner is tasked with two
objectives per state. The first is to move to the torso such that its center of mass falls inside the current support triangle, 
which is definied by the three end effectors which are not moving in a particular step. The motion planner is then tasked with
moving the moving end effector to its next foothold, while ensuring that the torso stays inside the current support triangle. These two tasks 
are repeated for each state in the state path. The motion planner accepts as input a start configuration, and a desired objective. It must find a
end configuration satisfying the objective, whether it is move the torso inside the support region or move the end effector 
to its end positoin, and then create a path between these two configurations. To find the end configuration, the function get_end_config()
is called. This function creates an inverse kinematics problem whose solution satisfies the end configuation objective. The ik problem is 
solved ProjectConstants.END_CONFIG_SAMPLE_COUNT times, whereas in each iteration the torso is randomly offset by x,y,z $`\in \(-.025,.025,)`$.
The configuration from the valid output set of this process with the lowest cost with the lowest cost is returned. Cost is measured as the change between
the start configuration and the given configuation. This cost is utilized so as to penalized configuations proportionally to their distance
in configuration space from the start configuration. If the valid output set is empty, a failure is reported. The motion planner then uses
klampt.cspace.MotionPlan() to generate a path between the start and end configuration. This path is made finer by space.discretizePath(path) and returned
to the control loop.


The control loop takes a largely managerial role - it is responsible for creating and updating support support triangle objects,
error checking, and user input management. If a UserInput() instance is supplied to the ControlLoop, users use the following to control the robot:

```
*`e`*- Exit key: stop the control loop
*`c`*- Print the robots configuration
*`p`*- Pause the control loop (resumes when key is no longer pressed)
*`m`*- Enter manual control mode (use arroy keys to translate the robots torso, *`e`* to exit)
```

Hyperparameter Optimization
---------------------------

The current system is highly depenedent on provided hyperparameters. The parameters must be carefully picked for each environment to ensure
timely and successful runs. Furthermore, upon failure, the question of whether a solution is possible 
given an optimal hyperparameter set remains unanswered. 

Basic functionality to optimize hyperparameters has been built in the HyperparameterOptimizer.optimize_for_world().
The function takes as input a world name, start and end positions, and boundries, and uses either scipy.minimize or 
scipy.basinhopping to optimize relevent hyperparameters. At each evaluation, a SystemRunner is created with the given 
hyperparameters, and the distance the robot ends up from the target position is returned as the cost. If the high level
or step sequence planner time out, fixed costs are returned. Each iteration is very costly, and can take several minutes to run.

parameters

    - BASE_STATE_END_EFF_DX_FROM_TORSO: base state end effector x
    - BASE_STATE_END_EFF_DY_FROM_TORSO: base state end effector y
    - TORSO_Z_DESIRED: base state torso height
    - HLTRAJ_H_WEIGHT: the hl trajectory heuristic weight scalar
    - STEPSEQ_H_WEIGHT: the step sequence heuristic weight scalar
    - STEP_SEQ_COST_PARABOLA_SLOPE: the slope of the cost parabola used to created hybrid cost maps in the step sequence
          planner. This parameter affects the distance from idealized positions in the direction of the target state that 
          foothold positions will take.
    - STEPSEQ_TRANSLATION_DISTANCE: desribed above in the 'Step Sequence Planner' 
    
Initial tests have been unsuccessful due to the computation time required per iteration. Bayesian optimization and/or evolutionary optimization
are more appropriate approaches to this problem, as this approach relies on gradient based optimization which requires smooth cost functions
to be effective, which the provided cost function is not. Baysian and evolutionary optimization are better suited at 'global optimization of 
noisy black-box functions' and thus make better algorithm classes to apply for this problem.

Lidar
-----
Please note the lidar input system is still in development.

##### Raspberry Pi
Download and install ros. I reccomend installing Ubuntu MATE to your RPi, as installing ros on Raspbian is a 
headache and probably wont work. Once ros is installed, install urg-node (https://sourceforge.net/p/urgnetwork/wiki/ROS_en/) 
and configure the ros network. Currently the lab desktop is master. Currently a script in the desktop's ~/.bashrc sets 
the master uri to the desktops local ip. It is left to the developer to set this same ip to the MASTER_URI variable in 
the raspberry pi's ~/.bash_rc. A possible solution to automate this process would to have the raspberry pi scan the local 
network looking for devices with the same mac address as the desktop, then setting the MASTER_URI ip to that computer's ip. 
After the ros network is configured, to publish lidar data  run `sudo chmod a+r2 /dev/ttyACM0`, followed by `rosrun urg_node urg_node`.
 'LaserScan' messages will now
be published to the '/scan' topic. Next run `pcloud_publisher.py`. This script continuously rotates the servo on the robot, and subscribes 
to messages sent in '/scan', publishing 'PointCloud' (http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud.html)
messages to '/pcloud'. Laser scan messages sent through '/scan' contain ~2000 data points. The current conversion method is 
slow, and can only convert 1/8 of the data points. If any more are translated a queue grows and the output PointCloud lags.
This is an open issue. A couple solutions (non exclusive) are possible: 
 1) Use numpy to convert the entire array, instead of converting point by point
 2) Use a C/C++ module to perform calculations
 3) Run the conversion on a more powerful computer, like the desktop. The problem with this method is the desktop needs to 
 know, with great precision, the servo's angle at the time the laser scan was sent.

username to the rpi: *jeremysmorgan*, pw: *jsm*

##### Point Cloud to Height Map Conversion
This is an open problem. The task is to convert the given point cloud to a height map. Available options include using 
klampt.robotsim.Geometry3D.convert(), and scipy.spatial.Delaunay(). The convert() function converts a structured klampt 
PointCloud() into a klampt TriangleMesh(). Note that this function is currently only available in the logging-devel
branch of Klampt. Note that there is currently a install bug in the branch. To fix it, in the Klampt home directory
you need to call `mv Library OldLibrary`. Delaunay() performs delaunay triangluation on the points to create a triangle 
mesh. Another possible solution is to implement the method described in the paper 'A Novel Way to Organize 3D LiDAR Point 
Cloud as 2D Depth Map Height Map and Surface Normal Map', written by Yuhang He, Long Chen, Jianda Chen and Ming Li 
(https://ieeexplore.ieee.org/document/7418964/). This is a difficult problem.

Software
--------
Source code is stored in /src. 
- /src/generators stores generators which produce DataObjects (found in /src/utilities/DataObjects), 
such as Maps, FootstepSequence objects, HLTrajectory objects, etc. 
- /src/lidar stores lidar classes. LidarSubscriber.py currently analyzes a saved point cloud and displays it in klampt.
- /src/motion stores all motion related classes, including ControlLoop.py, MotionPlanner.py, MotionUtils.py and IKSolverUtilities.py (the last two are convenience classes)
- /testing stores useful test automation utilities, namely SystemRunner.py. This is, in my opinion, the cleanest, most efficient, 
    and easiest way to work with the generators and planners.

Ex1: Run the system on "step_world_1", save and visualize the results
```python
    x_vars = [-1, 8, .015]
    y_vars = [0, 3, .015]
    xy_yaw0 = [0, 1, 0]
    xy_yawf = [6.5, 2, 0]
    world_name = "step_world_1"
    step_world1 = SystemRunner(x_vars, y_vars, xy_yaw0, xy_yawf, True, True, True, world_name)
    step_world1.run()
    step_world1.save()
    step_world1.get_results().print_results()
    step_world1.visualize(ignore_hltraj=True, ignore_step_seq=True)
```

Ex2: load and visualize the results of Ex1
```python
    x_vars = [-1, 8, .015]
    y_vars = [0, 3, .015]
    xy_yaw0 = [0, 1, 0]
    xy_yawf = [6.5, 2, 0]
    world_name = "step_world_1"
    step_world1 = SystemRunner(x_vars, y_vars, xy_yaw0, xy_yawf, True, True, True, world_name)
    step_world1.load()
    step_world1.visualize(ignore_hltraj=True, ignore_step_seq=True)
```

Ex3: Run and display multiple tests
```python
    x_vars = [-1, 8, .015]
    y_vars = [0, 3, .015]
    xy_yaw0 = [0, 1, 0]
    xy_yawf = [6.5, 2, 0]
    world_name = "step_world_1"
    step_world1 = SystemRunner(x_vars, y_vars, xy_yaw0, xy_yawf, True, True, True, world_name)
    
    x_vars = [-1, 8, .015]
    y_vars = [0, 3, .015]
    xy_yaw0 = [0,1,0]
    xy_yawf = [7,2,4]
    world_name = "step_world_2"
    step_world2 = SystemRunner(x_vars, y_vars, xy_yaw0, xy_yawf, True, True, True, world_name)
    
    tsrunner = TestingSuiteRunner([step_world1, step_world2])
    tsrunner.run(visualize=True, save=True)
```

Ex4: Run only the generators and HL, and Step Sequence planners
```python
    x_vars = [-1, 8, .015]
    y_vars = [0, 3, .015]
    xy_yaw0 = [0, 1, 0]
    xy_yawf = [6.5, 2, 0]
    world_name = "step_world_1"
    step_world1 = SystemRunner(x_vars, y_vars, xy_yaw0, xy_yawf, True, True, False, world_name)    # Notice the 'False' in last boolean parameter.
    step_world1.run()
    step_world1.save()
    step_world1.get_results().print_results()
    step_world1.visualize(ignore_hltraj=True, ignore_step_seq=False)
```

Ex5: Run only the generators and HL, and Step Sequence planners, and manually run the ControlLoop
```python
    x_vars = [-1, 8, .015]
    y_vars = [0, 3, .015]
    xy_yaw0 = [0, 1, 0]
    xy_yawf = [6.5, 2, 0]
    world_name = "step_world_1"
    step_world1 = SystemRunner(x_vars, y_vars, xy_yaw0, xy_yawf, True, True, False, world_name)    # Notice the 'False' in last boolean parameter.
    step_world1.save()
    control_loop = ControlLoop(None,None,None,None,None,None, test_suite=step_world1)
    control_loop.initialize()
    control_loop.run()
```

Ex6: Optimize the hyperparameter set for 'flat_world'
```python
    x_vars = [-5, 5, .015]
    y_vars = [0, 3, .015]
    xy_yaw0 = [-3.1, 1, 0]
    xy_yawf = [3.5, 1.5, 0]
    world_name = "flatworld"
    hyperper_param_opt = HyperparameterOptimizer()
    hyperper_param_opt.optimize_for_world(x_vars, y_vars, xy_yaw0, xy_yawf, world_name,method="basinhopping")
```


When saving an output, SystemRunner will hash the worldname, and all relevent hyperparameters for each output object to create a file name.
When running, SystemRunner will check to see if the output has already been performed by performing the same hash. If a file exists, it loads the file
instead of reperforming the same calculation. This functionality can be disabled by adding SystemRunner.\<obj\>_type to the SystemRunner.default_stettings\[dont_load_settings_key\] array.


##### Requirements
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

To Do
-----

- [x] add slope cost to cmap. Currently disabled bc doesn;t work
- [x] Robot too low. -> save max distance vals, then raise torso z
- [x] find distance that q143 is especially large at
- [x] Known jumps between configs queued to execution robot. likely bc of fix to !closedLoop(startconfig) issue.
- [x] - add joint constaints
- [x] - Adding torso z, dist to incenter metrics to config_cost may have fixed this
- [x] Failing bc choosing end state with link1's rotated into body, reducing movability. Enforce joint constraints?

> joint constraints added, not effective.

- [ ] slerp outputting non rotations
- [ ]  'reset' procedure to return robot to good state. Idea is to reset legs into manuevable regions
- [ ] 'rectangularness' heuristic for step plans?
- [ ] Need rigid objects in world files, klampt vis fails o.w.
- [ ] Default stance should be wider in y, possibly x aswell
- [ ] Stats on how similar to default stance robot is during normal steps
- [ ] Redefind nominal stance




##### Unsolved Algorithmic Problems
- Hyperparameter optimization
- End configuration optimization
- Reuse config space map (even w/ new obstacles?)
- Motion primitives to speed up motion planning?

##### Software Features to Add
- ~~more sample worlds~~
- ~~testing suite~~
- ~~randomized test suite worlds~~




##### Known Bugs

1. When setting the motion planner's endpoints, the start configuration will occassionally not be closed loop ( space.closed_loop(start_q) == False.) even though
  the same configuration WAS closed loop when it was treated as the end configuration. For example: 
    ```
    q_idx: 45
    torso_com_validator(end_config), hash: True , 97533
    space.inbounds(end_config), hash: True , 97533
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

2. 'ValueError: insecure string pickle'. Error thrown by pickle occasionally. Not sure why. Simple solution is to recalculated 
    whatever object is trying to be loaded