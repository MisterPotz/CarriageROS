# CarriageROS

# Some Info
This is a set of packages that provides a model of a robot that is used in carrying various things in specialized warehouses. 
Each side of this robot is equipped with wheels. Robot can travel in 2-dimensional space - along x-axis or along y-axis at one time. When it is switched between axises, some wheels go down and others go up.
## Packages to be installed
You should install the following packages:
```
ros-melodic-ros-control
ros-melodic-ros-controllers
```

## Launch
To launch this model in Gazebo, simply type
```
roslaunch launch simple_bot.launch
```
To play with the dropdown wheels, open rqt with
```
rqt
```
then open plugins menu, then enter topics, then messages. Select topics that end with "command", add them to your workspace with "+" button. Then click on it, in data enter "-0.7" to drop wheel down or "0" to get the wheel up.
Very soon API for easy use will be provided.

### A.N.O.N. team
Gornostaev ALexander,
Dmitriev Oleg,
Belousov Nikita,
Lyaschenko Nikita
