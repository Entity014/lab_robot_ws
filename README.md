# lab_robot_ws
this project use ros2 distro is humble.
# Getting started
1.Clone the repo:
```
git clone https://github.com/Entity014/lab_robot_ws.git
```
2.Navigate to your workspace:
```
cd ~/lab_robot_ws
```
3.Update dependencies:
```
rosdep update
```
4.Install dependencies:
```
rosdep install --ignore-src --from-paths src -y -r
```
5.Compile
```
colcon build
```
# Operating Instructions
After you build, remember to source the proper install folder...
```
source ~/lab_robot_ws/install/local_setup.bash
```
And then run the launch file...
```
ros2 launch abu_description moveit.launch.py 
```
After that run the this file to control robot
```
ros2 run abu_description teleop_twist_keyboard.py 
```
