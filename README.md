### Ground Filter

This program is designed to provide a library for point cloud ground filtering. The current inherited method is only CSF. For the detailed description of the CSF method, please refer to the following paper.

```
W.Zhang, J.Qi*, P.Wan, H.Wang, D.Xie, X.Wang, and G.Yan, “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sens., vol. 8, no. 6, p. 501, 2016. (http://www.mdpi.com/2072-4292/8/6/501/htm)
```

In the development of our package, we reference to [CSF](git@github.com:jianboqi/CSF.git).

### Prerequisites

A third-party library is required to run this program:

1. Ubuntu and ROS( pcl_ros、tf、sensor_msgs)
2. Eigen3 
3. PCL1.8 
4. JsonCpp 

### Build

Clont the repository and make:

```shell
git clone git@github.com:chronofei/ground_filter.git ground_filter
cd ground_filter
mkdir build
cd build
cmake ..
make -j8
```

### Run

```shell
source build/devel/setup.bash
cd launch
roslaunch offline.launch
```

### Additional Notes

1. The package name in the offline.launch needs to be specified as yours.
2. When using, you need to adjust the parameters in the config/*.json to suit your lidar