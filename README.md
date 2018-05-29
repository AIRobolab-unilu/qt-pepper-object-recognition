# qt-pepper-emotion-classification

This repository aims to perform emotion classification and object recognition with the QT robot and Pepper in order to have a better understanding of the human-robot interacton context. 

## Getting Started

You should have installed on your computer [**ROS Kinetic**](http://wiki.ros.org/kinetic/Installation) and **Python 2.7**(which is preinstalled on Ubuntu).

Clone this git repository in your local machine in you ros catkin workspace

```
~/catkin_ws$ git clone --recursive https://github.com/AIRobolab-unilu/qt-pepper-emotion-classification.git
```

## The face_classification package

This package is used to perform face emotion recognition with the QT robot and Pepper.

### Prerequisites 
You need to install the the following python libraries before running the package. To do that, you can use pip (to install it, write in the terminal `sudo apt-get install python-pip`)

```
pip3 install numpy scipy scikit-learn pillow tensorflow pandas h5py opencv-python keras==2.0.9 statistics pyyaml pyparsing cycler matplotlib Flask
pip install panda
```
Please note that you need to install keras version 2.0.9 and not another one.

You might also need to install [tensorflow with virtualenv](https://www.tensorflow.org/install/install_linux#InstallingVirtualenv) if the pip version creates errors when the package is run.

### Installing and running

To run it, go to the src folder of the face classification package and then run the video_emotion_color_demo_qt.py node :

```
~/catkin_ws$ roscd face_classification/src
~/catkin_ws/face_classification/src$ rosrun face_classification video_emotion_color_demo_qt.py
```

## The darknet_ros package

This package is used to perform object recognition with the QT robot and Pepper.

### Prerequisites

If you don't have a node publishing images in a specific topic, you may also have to install a ROS package that can open the camera ([cv_camera](https://github.com/OTL/cv_camera)).


### Installing and running

To run it, go to the catkin main folder of the face classification package and then run these commands :

- Compile the package using the catkin_make command and report the modification of the workspace to ROS by sourcing the setup.bash

```
~/catkin_ws$ catkin_make --pkg darknet_ros
~/catkin_ws$ source devel/setup.bash
```

- Then just run the launchfile

```
~/catkin_ws$ roslaunch darknet_ros darknet_ros.launch
```

- Run the camera in another terminal and you should see a window displaying the recognized objects

```
~/catkin_ws$ rosrun cv_camera cv_camera

```

- If you already have a node publishing images in a specific topic, you will have first to open the **launchfile** located in **darknet_ros/darknet_ros/launch/** and modify the **line 24** where you have to replace /cv_camera/image_raw with the name of your topic.



## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Chris Ibrahim** 

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

This is a non-exhaustive list of the people whose code was used to achieve this project :

* Octavio Arriaga (https://github.com/oarriaga/face_classification)
* Dat Tran (https://github.com/datitran/object_detector_app)
* Francisco Lera (https://github.com/FranLera/simple_face_detection)
* Marko Bjelonic (https://github.com/leggedrobotics/darknet_ros)

