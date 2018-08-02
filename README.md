# QT and Pepper object recognition

This repository aims to perform object recognition with the QT robot and Pepper in order to have a better understanding of the human-robot interacton context. 

## Getting Started

You should have installed on your computer [**ROS Kinetic**](http://wiki.ros.org/kinetic/Installation) and **Python 2.7**(which is preinstalled on Ubuntu).

Clone this git repository on your local machine (QT or your computer) in your ros catkin workspace.

`~/catkin_ws$ git clone --recursive https://github.com/AIRobolab-unilu/qt-pepper-emotion-classification.git`

In order to be able to use those programs on your computer or on the robots, you will have to make some modifications in your **.bashrc** files in the /home directory.

First open the .bashrc.

`~$ sudo nano .bashrc`

 Then add these lines in it.
 
```
########################
# ROS network connections

#MY IP Playground
MY_IP=10.212.232.16

#QT IP
#ROBOT_IP=10.212.232.14

#Pepper IP
#ROBOT_IP=10.212.232.100

#export ROS_MASTER_URI=http://$ROBOT_IP:11311/
#export ROS_MASTER_URI=http://$MY_IP:11311/

export ROS_IP=$MY_IP
```
Replace adress in MY_IP with your actual IP adress. Check it with the `ifconfig` command, for me it was in the **wlp50** section in **inet addr** (2nd line) :

```
wlp5s0    Link encap:Ethernet  HWaddr 24:fd:52:7a:04:cb  
          inet addr:10.212.232.13  Bcast:10.212.233.255  Mask:255.255.254.0
          inet6 addr: 2001:a18:a:2fec:768e:5ad7:3234:81b8/128 Scope:Global
          inet6 addr: fe80::5b75:af:ce61:d31/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:81063 errors:0 dropped:0 overruns:0 frame:0
          TX packets:44139 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000 
          RX bytes:81344563 (81.3 MB)  TX bytes:5680750 (5.6 MB)
```

You will also have to **check and replace the IP addresses of both robots** by the same process using `ifconfig` on them.

- If you are running the roscore on your computer just uncomment this line :

`#export ROS_MASTER_URI=http://$MY_IP:11311/`

- If you are running the roscore on QT uncomment these 2 lines : 

`#ROBOT_IP=10.212.232.14`

`#export ROS_MASTER_URI=http://$ROBOT_IP:11311/`

- If you are running the roscore on Pepper uncomment these 2 lines : 

`#ROBOT_IP=10.212.232.100`

`#export ROS_MASTER_URI=http://$ROBOT_IP:11311/`

## The darknet_ros package

This package is used to perform object recognition with the QT robot and Pepper. You can use it with the camera of the robot or any camera, and even apply it to a video stream.

### Prerequisites

If you don't have a node publishing the camera images in a specific topic, you may also have to install a ROS package that can open the camera ([cv_camera](https://github.com/OTL/cv_camera)).


### Installing and running

To perform the installation, go to the catkin main folder and then run the following commands :

- Compile the package using the catkin_make command and report the modification of the workspace to ROS by sourcing the setup.bash

`~/catkin_ws$ catkin_make --pkg darknet_ros`

`~/catkin_ws$ source devel/setup.bash`

- Then just run the launch file

`~/catkin_ws$ roslaunch darknet_ros darknet_ros.launch`

- Run the camera in another terminal and you should see a window displaying the recognized objects

`~/catkin_ws$ rosrun cv_camera cv_camera`

- If you already want to use another node publishing the camera images in a specific topic, use the following option to specify the name of your topic when running the launchfile.

`~/catkin_ws$ roslaunch darknet_ros darknet_ros.launch input_camera_topic:=yourTopicName`

- If you want to remotely visualize the resulting images from the detection, use this topic **/darknet_ros/detection_image** (see the readme [here](https://github.com/leggedrobotics/darknet_ros/blob/master/README.md) for more details) : 

`rosrun image_view image_view image:=/darknet_ros/detection_image`

- If you want to use a video from which images are streamed to a specific topic and apply the recognition, use the following command :


`~/catkin_ws$ roslaunch darknet_ros darknet_ros_with_video.launch video_path:=yourVideopath`




## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Chris Ibrahim** 

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

This is a non-exhaustive list of the people whose codes were used to achieve this project :


* Marko Bjelonic ([darknet_ros package](https://github.com/leggedrobotics/darknet_ros))
* Dat Tran ([object_detector_app](https://github.com/datitran/object_detector_app))

