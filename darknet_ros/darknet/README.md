## Training Yolo darknet

The models used in the darknet ros package can be trained either with available datasets online (the VOC dataset) or your own custom dataset to detect custom objetcs.

### Prerequierements and Installing

The following instructions comes from this [tutorial](https://pjreddie.com/darknet/yolo/) which explains how to train yolo using different datasets.

You have to clone first the following git repository even if I already have something similar in my darknet ros package.

`catkin_ws/src$ git clone https://github.com/pjreddie/darknet.git`

And then, compile the package :

```
catkin_ws/src$ cd darknet
catkin_ws/src/darknet$ make
```

For the rest, you can follow the command from the [tutorial](https://pjreddie.com/darknet/yolo/) in the same directory but the rest of my tutorial is the training part with the VOC dataset and how to train the network with a custom one.


### Downloading the VOC datasets

It can be downloaded to your folder /darknet/data from the link shown in the following commands (VOC is just for the example but there many other datasets out there as shown in the [tutorial](https://pjreddie.com/darknet/yolo/)) :

* Download the datasets
```
wget https://pjreddie.com/media/files/VOCtrainval_11-May-2012.tar
wget https://pjreddie.com/media/files/VOCtrainval_06-Nov-2007.tar
wget https://pjreddie.com/media/files/VOCtest_06-Nov-2007.tar
```
* Unzip them
```
tar xf VOCtrainval_11-May-2012.tar
tar xf VOCtrainval_06-Nov-2007.tar
tar xf VOCtest_06-Nov-2007.tar
```

### Train the model with an already existing dataset (VOC)


To generate the label, go to the scripts/ folder and download the voc_label.py file.
```
wget https://pjreddie.com/media/files/voc_label.py
python voc_label.py
```

Concatenate the label files.
```
cat 2007_train.txt 2007_val.txt 2012_*.txt > train.txt
```

Modify the cfg/voc.data config file by pointing the variables to the train.txt file.
```
classes= 20
train  = <path-to-voc>/train.txt
valid  = <path-to-voc>2007_test.txt
names = data/voc.names
backup = backup
```

Download the pre-trained weights.

`wget https://pjreddie.com/media/files/darknet53.conv.74`

### Train the model with your own custom dataset

Just follow this [tutorial section](https://github.com/AlexeyAB/darknet#how-to-train-to-detect-your-custom-objects).

And when you arrive to the part you are supposed to make your own datasets and all the labels, just clone this git repository https://github.com/AlexeyAB/Yolo_mark and read the readme provided. Afterwards you can resume the tutorial.

[This](https://timebutt.github.io/static/how-to-train-yolov2-to-detect-custom-objects/) is also a way to build your own dataset but I did not try it yet.


## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Chris Ibrahim** 

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

This is a non-exhaustive list of the people whose codes were used to achieve this project :


* [Darknet project website](http://pjreddie.com/darknet)
