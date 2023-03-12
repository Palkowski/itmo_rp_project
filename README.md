# ITMO robot programming course project
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
