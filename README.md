# Data viewer for complex urban data set

Maintainer: Jinyong Jeong (jjy0923@kaist.ac.kr)

This program is a data viewer for the complex urban data set. If a user installs the ROS using "Desktop-Full version", there is only one additional dependent package, except for the ROS default package. First, clone this package into the src folder of your desired ROS workspace.

## 1. Obtain dependent package (defined msg)

```
$cd ~/catkin_ws/src
$wstool init
$wstool merge data_viewer/depend_pack.rosinstall
$wstool update
```

## 2. Build workspace

```
$cd ~/catkin_ws
$catkin_make
```

## 3. Run data viewer

```
$source devel/setup.bash
$roslaunch data_viewer data_viewer.launch
```

