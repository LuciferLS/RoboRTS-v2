## Run
* Put the package under the same `src` directory as `RoboRTS-v2` and `gazebo_temp` repositories.
* build the project
* open *four* terminals and run `source devel/setup.bash` for each of them
* at terminal1, run
```bash
roslaunch roborts_bringup multibots_sim.launch
```
* at terminal2, run
```bash
ROS_NAMESPACE=/r1 rosrun roborts_decision goal_factory_node
```
* at terminal3, run
```bash
 rosrun roborts_sim sim_node 
```
