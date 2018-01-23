
# Robot delta
## How to run the code
In a terminal window, type:
```sh
$ roslaunch delta delta.launch
```
Per a poder enviar ordres a l'arduino:
```sh
$ rostopic pub /delta/servo std_msgs/UInt16MultiArray '{data: [0,0,0]}' --once
```

## Nou mòdul Kinetmatics

Per a poder publicar una posició i que recalculi la posició dels motors
```sh
$ rostopic pub /delta/position geometry_msgs/Vecr3 22 22 220
```

Per a poder veure la posició dels motors
```sh
$ rostopic echo /delta/thetas
```

Per a que tot funcionés directament (a partir d'una posició moure els motors) nomes s'ha de modificar el nom del topic thetas pera  que coincideixi amb el de servo de l'arduino.


## Tip
Check your webcam encodings (yuyv,mjpeg,...) and set them accordingly at file launch/usb_camera.launch

