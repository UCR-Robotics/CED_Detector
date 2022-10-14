# CED_Detector

## 1. Introduction

We propose an efficient multi-modal keypoint detector that can extract both geometry-salient and color-salient keypoints in a colored point cloud, with the potential to be applied to point clouds with multiple modalities (e.g., colored by multi-spectrum images).

The proposed CEntroid Distance (CED) keypoint detector comprises
- an intuitive and effective measure for keypoint saliency, the distance to centroid, which can leverage directly the distribution of points and **does not require normal estimation or eigenvalue decomposition**, and
- a multi-modal non-maximum suppression algorithm that can select keypoints with high saliency in two or more modalities.

Despite the fact that colored point clouds can be readily obtained, there currently exists no effective detectors that make good use of both modalities. We hope that this work can inspire researchers in the field and benefit downstream systems that can leverage color modality to improve performance.

**Authors:** Hanzhe Teng, Dimitrios Chatziparaschis, Xinyue Kan, Amit K. Roy-Chowdhury, and Konstantinos Karydis from [ARCS Lab](https://sites.google.com/view/arcs-lab/) and [VCG Lab](https://vcg.ece.ucr.edu/) at [UC Riverside](https://www.ucr.edu/).

**Videos:** Our presentation at WACV 2023 will be posted once available.

**Related Publications:**
H. Teng, D. Chatziparaschis, X. Kan, A. K. Roy-Chowdhury and K. Karydis, "**Centroid Distance Keypoint Detector for Colored Point Clouds**", in IEEE/CVF Winter Conference on Applications of Computer Vision (WACV), 2023. To appear. ([preprint](https://arxiv.org/abs/2210.01298))


## 2. Qualitative Example

Below is an example of keypoints detected on an arbitrary frame in the [Redwood Scan dataset](http://redwood-data.org/indoor_lidar_rgbd/index.html) (real-world scene, RGB-D camera), where our proposed CED detector can capture color changes between floor tiles and extract keypoints with high regularity, and (out of all methods) only ISS, CED and CED-3D (its geometry-only variant) detectors can extract keypoints on four stove knobs. 

![](./results/qualitative.png)


## 3. Installation

### 3.1 Prerequisites
- Ubuntu 18 (where this project was developed) or Ubuntu 20 (tested)
- PCL 1.8 (default version in Ubuntu 18) or above
- Eigen
- CMake
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
  - We use yaml-cpp as the YAML parser to manage configurations and tune parameters at runtime.
- [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus)
  - TEASER++ is a point cloud registration library, which can solve a rigid-body transformation provided a list of tentative correspondences in 3D space. We apply this library to estimate transformation in the point cloud registration example.
- (Optional) [Open3D](https://github.com/isl-org/Open3D)
  - Compared with PCL, Open3D can provide a better visualization for keypoints. We recommend using Open3D to visualize detected keypoints if possible.

### 3.2 Steps
- To install PCL, Eigen, CMake and yaml-cpp, run `sudo apt install libpcl-dev libeigen3-dev libyaml-cpp-dev cmake`.

- To install TEASER++, follow the steps below (as provided in the official repository).
  ```
  git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
  cd TEASER-plusplus && mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig
  ```

- To install Open3D, the simplest way is to install from pip as shown below. Alternatively, you can build from source.
  ```
  sudo apt install python3-pip python-dateutil
  pip3 install open3d
  ```

- To build this project, follow the steps below.
  ```
  git clone https://github.com/UCR-Robotics/CED_Detector.git
  cd CED_Detector && mkdir build
  cd build
  cmake ..
  make -j
  ```


## 4. Usage

### 4.1 Test keypoint detection and visualize in Open3D (recommended)
```
cd scripts
python3 view_keypoint.py
```

### 4.2 Test keypoint detection and visualize in PCL
Set `pcl_visualization` to true in `test_kp_config` node in the `config.yaml` file, and run
```
cd build
./test_keypoint ../config/config.yaml
```

### 4.3 Test registration pipeline and visualize in PCL
```
cd build
./test_registration ../config/config.yaml
```

### 4.4 Tune parameters

Modify `config/config.yaml` file to change the paths for input clouds and output results, control the running flow of keypoint detection and point cloud registration tasks, and tune parameters at runtime. The meaning of each parameter has been explained in the comments in the YAML file.
Default parameters currently presented in the YAML file are tuned empirically or set according to related literature/implementation (e.g., [ISS](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/keypoints/include/pcl/keypoints/iss_3d.h#L72), [Harris3D](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/examples/keypoints/example_get_keypoints_indices.cpp#L65)).


## 5. Code Structure
- `config` folder
  - `config.yaml`: The YAML file to manage configurations. It is convenient for tuning parameters at runtime, without the need of re-compilation of the C++ program.
- `data` folder
  - Contains a few sample point clouds from [Redwood Synthetic](http://redwood-data.org/indoor/) and [Redwood Scan](http://redwood-data.org/indoor_lidar_rgbd/) datasets.
- `include` folder
  - `ced_6d.h` and `impl/ced_6d.hpp`: The implementation of the proposed CED keypoint detector, which can detect keypoints in both 3D space and color space. It is following PCL coding style and organized as a derived class of the `pcl::Keypoint` base class.
  - `ced_3d.h` and `impl/ced_3d.hpp`: The implementation of the proposed CED-3D keypoint detector, which is a geometry-only variant of the CED detector. It can detect keypoints in 3D space.
  - `random_keypoint.h` and `impl/random_keypoint.hpp`: The implementation of the random keypoint detector, which is used as the baseline for comparison.
  - `perception.h` and `impl/perception.hpp`: A convenient interface class that wraps up common operations in keypoint detection and point cloud registration tasks.
  - `loop_timer.h`: A timing tool optimized for high frequency loops (e.g., millions of iterations in a short time), which has been used to measure the performance of detectors during the development of this project. It is independent from other released code.
- `scripts` folder
  - `view_keypoint.py`: A python script to run the keypoint detection program and visualize detected keypoints in Open3D visualizer.
- `src` folder
  - `test_keypoint.cpp`: A sample code to test keypoint detection pipeline.
  - `test_registration.cpp`: A sample code to test point cloud registration pipeline.


## 6. Future Work
The proposed CED detector is inspired and motivated by the ORB keypoint detector in 2D images (where intensity centroid was used as the measure of orientation), and aims to attain relatively high performance while keeping computational time to a minimum (cf. ORB vs. SIFT). It targets real-time CV & robotics applications. If running time is not a concern, combining Harris/ISS detector in 3D space and a new eigenvalue -based detector in (multi-spectrum/dimension) color space may lead to a better result.

As for the performance of specific tasks as a whole, we also need to pay attention to the downstream components.
For example, in the point cloud registration pipeline, we shall ask:
- Have feature descriptors made the best use of the detected keypoints? How much information has been retained?
- Is there any better way to establish correspondences rather than computing L1 distance in feature vector space?
- Can we make use of the relative positions (in 3D space) to match feature descriptors and establish correspondences? 
- From the perspective of information theory, can we analyze the information flow during the process of detection, description, matching and transformation estimation? Can we provide any mathematical guarantees on it? 

In addition, there also lacks multi-modal datasets for evaluation in the field, especially in outdoor environments. 
- Geometric registration benchmarks (e.g., [3DMatch](https://3dmatch.cs.princeton.edu/)) do not consider color information, though captured by the original sensors.
- Most odometry/SLAM datasets do not provide 6D poses of the sensor origin at each moment; ground truth trajectories in 3D space (or 6D poses at the robot body) are not sufficient.
- Autonomous driving datasets (e.g., KITTI) provide limited field of view for cameras, which makes point cloud colorization incomplete and inaccurate. 

We hope that you find this work interesting and inspiring, and we would like to call for more attention in the field of multi-modal perception.
IMHO, (multi-spectrum) colored point clouds can be really beneficial to applications such as mapping, place recognition or loop closure.

You are very welcome to contact us or open a new issue in this repository if you have any questions/comments! Discussions on these open problems or future work are also welcome!
