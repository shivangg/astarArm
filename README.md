# astarArm
Implemented A-Star for maze solving for the KUKA iiwa robotic arm

Install using 
```sh
cd ~/catkin_ws/src
git clone https://github.com/shivangg/astarArm.git 
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
```

Then build using,

```sh
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Run using 

```sh
cd ~/catkin_ws/src/rll_planning_project/scripts/
./start_project.sh
```