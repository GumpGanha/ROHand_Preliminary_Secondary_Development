# ROHAND URDF PACKAGE FOR ROS2

**This warehouse is based on [ROHand](https://github.com/oymotion/rohand_urdf_ros2.git) for secondary development**
rohand urdf package

## 1.Clone

```BASH
mkdir  ros2_ws && cd ros2_ws

git clone https://github.com/GumpGanha/ROHand_Preliminary_Secondary_Development.git src
```

Install dependencies(This may solve some of your dependency issues):

```BASH
   rosdep update --rosdistro=$ROS_DISTRO
   sudo apt-get update
   rosdep install --from-paths src --ignore-src -r -y
```

## 2.Compile

```BASH
colcon build
source install/setup.bash
```

## 3.Node rohand_urdf

| Topic             | Description                                                               |
| ----------------- | ------------------------------------------------------------------------- |
| "if_slider_joint" | slider for index finger, control index finger by changing it's position   |
| "mf_slider_joint" | slider for middle finger, control middle finger by changing it's position |
| "rf_slider_joint" | slider for ring finger, control ring finger by changing it's position     |
| "lf_slider_joint" | slider for little finger, control little finger by changing it's position |
| "th_slider_joint" | slider for thumb, control thumb by changing it's position                 |
| "th_root_joint"   | slider for thumb root, control thumb root by changing it's position       |

## 4.RUN

Launch 'launch.py' file:
Left hand：

```BASH
ros2 launch rohand_urdf_ros2 left_rviz2.launch.py 
```

Right hand：

```BASH
ros2 launch rohand_urdf_ros2 right_rviz2.launch.py
```

## GumpGan's Modify

In **two different Terminal** , launch these two files to start hands simulation which subcribe from extern joints data publication.The extern data publisher is defined in *rohand_urdf_ros2/scripts/extern_publish.py*.

And may be it's better to read the description above to gain a understanding of this project wholely.  

```BASH
ros2 launch rohand_urdf_ros2 left_extern_rviz2.launch.py
```

```BASH
ros2 launch rohand_urdf_ros2 right_extern_rviz2.launch.py 
```

And then **in another two Terminals** , run these two command.

```BASH
ros2 run rohand_urdf_ros2 imitate_extern_extern_pub   --ros-args --remap /external_external_joint_states:=/rohand_left/external_external_joint_states
```

```BASH
ros2 run rohand_urdf_ros2 imitate_extern_extern_pub   --ros-args --remap /external_external_joint_states:=/rohand_right/external_external_joint_states
```

In this file *rohand_urdf_ros2/rohand_urdf_ros2/scripts/imitate_extern_extern_pub.py* , I offer a demo to display the logic of data being passed in from the outside. You can replace this code with your need. 
