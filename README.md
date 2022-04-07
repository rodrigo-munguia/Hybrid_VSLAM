# Hybrid_VSLAM

Ros 2 implementation of the visual-based SLAM approach described in the paper:

R Munguia, JC Trujillo, E Guerra, A Grau "A Hybrid Visual-Based SLAM Architecture: Local Filter-Based SLAM with KeyFrame-Based Global Mapping" Sensors 22 (1), 210.

IMPORTANT: Note that this software is under development and several functionalities have been not implemented yet.

Tested in: Ubuntu 20.04

- Dependencies:

1.- ROS 2 (Galactic)

2.- Armadillo C++ library  (tested with version 9.800.4), http://arma.sourceforge.net/

3.- OpenCV with extra modules installed (opencv_contrib) (tested with OpenCV version 4.2.0), https://opencv.org/
    Particularly, SFM and VIZ modules are required. VIZ requires VTK support (tested with version 6.3.0).    
