
## How to run the code
In a terminal window, type:
```sh
$ roslaunch delta delta.launch
```
in an other terminal type:
```sh
$ rostopic pub /delta/servo std_msgs/UInt16MultiArray '{data: [0,0,0]}' --once
```
## Tip
Check your webcam encodings (yuyv,mjpeg,...) and set them accordingly at file launch/usb_camera.launch
# delta
