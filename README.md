# ITMO robot programming course project: "Encoder data velocity estimator"
Eugene Palkowski, Nikolay Dobryshev

## Installation
```bash
git clone https://github.com/Palkowski/itmo_rp_project.git
cd itmo_rp_project
bash docker/install_docker.bash
bash docker/build_docker.sh
bash docker/run_docker.sh
cd catkin_ws && catkin_make
```

## Run
Run docker container:
```bash
bash docker/run_docker.sh
```
then:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
in new terminal tab:
```bash
bash docker/exec_docker.sh
python catkin_ws/src/bot_control/bot_control.py 
```

## Result

The right and left wheels position and velocity estimation compared to the one from /odom, after the command sequence:
```python
bot_controller.go_x(5, 0.2)  # move forward 5 seconds with speed 0.2
bot_controller.turn(2, 1, -1)  # turn right 2 seconds with angular speed 1
bot_controller.go_x(5, 0.2)  # move forward 5 seconds with speed 0.2
```

![Alt text](catkin_ws/src/bot_control/Figure_1.png?raw=true "Figure_1")