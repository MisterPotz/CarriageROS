# CarriageROS

## Some Info
This is a set of packages that provides a model of a robot that is used in carrying various things in specialized warehouses. 
Each side of this robot is equipped with wheels. Robot can travel in 2-dimensional space - along x-axis or along y-axis at one time. At switches between different axis wheels are changed.
Also model of warehouse field is provided so tests with robot welcome.
Robot uses velocity_controllers package to manipulate with wheels (lift or drop them down). 
To ride between cells, robot uses custom-written code to build appropriate velocity profiles and move smoothly, effectively and accurately.

## Current capabilities
Manipulation with robot can be done via client console menu. The following commands are supported: move to cell with given coordinates, lift/drop wheels, execute demo ride around center of the field.

## Packages to be installed
You should install the following packages:
```
ros-melodic-ros-control
ros-melodic-ros-controllers
ros-melodic-gazebo-ros-control
```
## Launch
To launch this model in Gazebo, simply type
```
roslaunch launch simple_bot.launch
```
Then the field and robot will appear.
To play with it, run server and client as different nodes.
To run server:
```
rosrun carriage_control carriage_server
```
Then spin client:
```
rosrun carriage_control carriage_client
```
Client will greet with its simple menu.
## Work-in-progress
Currently, client doesn't support interruptions to running processes, so you have to actually wait after the task being finished. That will be fixed soon.
## Notes
Some strange behavior of robots prismatic joint controllers. It is not usually easily noticed but robot vibrates (so it is not steady even when it stands). Probably this is because of velocity_controllers principle of work. This is not really a big problem because robot is still able to move very accurate.

### A.N.O.N. team
Gornostaev ALexander,
Dmitriev Oleg,
Belousov Nikita,
Lyaschenko Nikita
