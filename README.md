
# Hybrid VSLAM

Ros 2 implementation of the visual-based SLAM approach described in the paper:

R Munguia, JC Trujillo, E Guerra, A Grau "A Hybrid Visual-Based SLAM Architecture: Local Filter-Based SLAM with KeyFrame-Based Global Mapping" Sensors 22 (1), 210.

IMPORTANT: Note that this software is under development and several functionalities have been not implemented yet.

 Tested in: Ubuntu 20.04

 **Dependencies:**

1.- ROS 2 (Galactic)

2.- Armadillo C++ library (tested with version 9.800.4), http://arma.sourceforge.net/

3.- OpenCV with extra modules installed (opencv_contrib) (tested with OpenCV version 4.2.0), https://opencv.org/ Particularly, SFM and VIZ modules are required. VIZ requires VTK support (tested with version 6.3.0).

**Usage:**

1.- Use colcon for compiling the project  (in the root project folder):  
```
foo@bar:~/Hybrid_VLSAM$ colcon build
```
2.- Extract the sample dataset included in folder /Dataset_sample:
```
foo@bar:~/Hybrid_VLSAM/Dataset_sample$ tar -xf 2022-1-22-16-6.tar.xz
 ```
 3.- In a text editor open the file "params.yaml" located in the folder "config", and set the parameter "Dataset_path:" with the absolute path of the extracted dataset folder. Example:
 ```
 Dataset_path: /home/Hybrid_VLSAM/Dataset_sample/2022-1-22-16-6/
``` 
4.-  In the root project folder open a terminal and source the overlay:
```
foo@bar:~/Hybrid_VLSAM$ . install/setup.bash
```
5.- In the same terminal run the Hybrid SLAM using the launch file "launch/slam.launch.py". At this point, a graphical interface must be opened.
```
foo@bar:~/Hybrid_VLSAM$ ros2 launch launch/slam.launch.py
```
6.- In the root project folder open a second terminal and source the overlay: 
```
foo@bar:~/Hybrid_VLSAM$ . install/setup.bash
```
7.- Run the keyboard interface component:
```
foo@bar:~/Hybrid_VLSAM$ ros2 run keyboard keyboard_node
```
At this point, the following menu must appear in the console:
```
| Dataset commands :
|   'q' to quit.
|   'p'-> Play/Pause  r-> Reset
|   '-'-> zoom out '+' ->  zoom in '1'-> x-y view '2' -> x-z view  '3'->y-z view
|   '8'-> view up '5' ->  view down '4'-> view left '6' -> view right  'c'-> clear plot
```
8.- Press the key "p".