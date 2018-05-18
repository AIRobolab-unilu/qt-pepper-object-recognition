# qt-pepper-emotion-classification

This repository aims to perform emotion classification and object recognition with the QT robot and Pepper in order to have a better understanding of the human-robot interacton context. 

## The face_classification package

This package is used to perform face emotion recognition with the QT robot and Pepper.

### Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

The libraries you need to install before running the package and how to install them

```
pip3 install numpy scipy scikit-learn pillow tensorflow pandas h5py opencv-python keras==2.0.9 statistics pyyaml pyparsing cycler matplotlib Flask
pip install panda
```
Please note that you need to install keras version 2.0.9 and not another one.

You might also need to install [tensorflow with virtualenv](https://www.tensorflow.org/install/install_linux#InstallingVirtualenv) if the pip version creates errors when the package is run.

### Installing and running

To run it, go to the src folder of the face classification and then run the video_emotion_color_demo_qt.py node :

```
roscd face_classification/src
rosrun face_classification video_emotion_color_demo_qt.py
```


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

