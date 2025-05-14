<div align = "center">
  <h1>
    Voxel-SVIO: Stereo Visual-Inertial Odometry based on Voxel Map
  </h1>
</div>
<div align = "center">
  <h2>
    A MSCKF based stereo visual-inertial odometry that utilizes voxel-based map management
  </h2>
</div>
<div align="center">
  <strong>
        Zikang Yuan,
        Fengtian Lang,
        Jie Deng,
        Hongcheng Luo, and
        Xin Yang<sup>†</sup>
  </strong>
  <p>
    <sup>†</sup>Corresponding Author
  </p>
  <a href="https://ieeexplore.ieee.org/abstract/document/10993347"><img src="https://img.shields.io/badge/Paper-IEEE RAL-004088.svg"/></a>
  <a href='https://www.youtube.com/watch?v=i085OQ0ESJ8'><img alt="Video" src="https://img.shields.io/badge/YouTube-Video-red"/></a>
</div>

## 💡 News
* **[2025.05.08]** **Voxel-SVIO** can be early accessed in IEEE Xplore ! 
* **[2025.04.30]** The source code of **Voxel-SVIO** is released !
* **[2025.04.21]** **Voxel-SVIO** is accepted by RAL 2025 **without revision** 🚀 !

## 📜 Introduction

**Voxel-SVIO** enables VIO systems to efficiently retrieve the most suitable points for optimizer inclusion, thereby ensuring optimal allocation of computational resources to the variables most critical for optimization. The map points in green voxels are feded into the estimator for MSCKF-based state update. Please refer to [**YouTube**](https://youtu.be/i085OQ0ESJ8) for smoother video playback.

<div align="left">
<img src="doc/demo_MH_02.gif" width=100.0% />
</div>

Please cite our paper if you use this project in your research:

```
@article{yuan2025voxel,
  author={Yuan, Zikang and Lang, Fengtian and Deng, Jie and Luo, Hongcheng and Yang, Xin},
  journal={IEEE Robotics and Automation Letters}, 
  title={Voxel-SVIO: Stereo Visual-Inertial Odometry based on Voxel Map}, 
  year={2025},
  volume={},
  number={},
  pages={}
}
```

Please kindly star ⭐️ this project if it helps you. We take great efforts to develop and maintain it 😁.

## 🛠️ Installation

### 1. Requirements

> GCC >= 7.5.0
>
> Cmake >= 3.16.0
> 
> [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.3.4
>
> [OpenCV](https://github.com/opencv/opencv) == 4.2.0 for Ubuntu 20.04
> 
> [PCL](https://pointclouds.org/downloads/) == 1.10 for Ubuntu 20.04
>
> [Ceres](http://ceres-solver.org/installation.html) >= 1.14
>
> [ROS](http://wiki.ros.org/ROS/Installation)

##### Have Tested On:

| OS    | GCC  | Cmake | Eigen3 | OpenCV | PCL | Ceres |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| Ubuntu 20.04 | 9.4.0  | 3.16.3 | 3.3.7 | 4.2.0 | 1.10.0 | 1.14 |

### 2. Create ROS workspace

```bash
mkdir -p ~/Voxel-SVIO/src
cd Voxel-SVIO/src
```

### 3. Clone the directory and build

```bash
git clone https://github.com/ZikangYuan/voxel_svio.git
cd ..
catkin_make
```

## 🚀 Run on Public Datasets

Noted:

A. **Please create a folder named "output" before running.** When **Voxel-SVIO** is running, the estimated pose is recorded in real time in the **pose.txt** located in the **output folder**.

B. We store the public datasets on Baidu drive. Chinese users can download the rosbag data of [*TUM_VI*](https://pan.baidu.com/s/1WLhnyq09KMpG4J4McT841Q?pwd=8pen) and [*KAIST*](https://pan.baidu.com/s/1KHkGmQ7nH5Une3VNerLyHQ?pwd=ss9a).

###  1. Run on [*EuRoC_MAV*](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

Please go to the workspace of **Voxel-SVIO** and type:

```bash
cd Voxel-SVIO
source devel/setup.bash
roslaunch voxel_svio vio_euroc.launch
```

Then open the terminal in the path of the bag file, and type:

```bash
rosbag play SEQUENCE_NAME.bag --clock -d 1.0
```

###  2. Run on [*TUM_VI*](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)

Please go to the workspace of **Voxel-SVIO** and type:

```bash
cd Voxel-SVIO
source devel/setup.bash
roslaunch voxel_svio vio_tum_vi.launch
```

Then open the terminal in the path of the bag file, and type:

```bash
rosbag play SEQUENCE_NAME.bag --clock -d 1.0
```

For the TUM_VI dataset, different configuration files are required for the scenarios of **corridor**, **magistrale**, and **room**. Please select the appropriate configuration file according to the sequence being processed.

###  3. Run on [*KAIST*](https://sites.google.com/view/complex-urban-dataset)

Please go to the workspace of **Voxel-SVIO** and type:

```bash
cd Voxel-SVIO
source devel/setup.bash
roslaunch voxel_svio vio_kaist.launch
```

Then open the terminal in the path of the bag file, and type:

```bash
rosbag play SEQUENCE_NAME.bag --clock -d 1.0
```

For the KAIST dataset, the extrinsic parameters of sequences *urban38* and *urban39* differ from other sequences. When processing *urban38* or *urban39*, please use **kaist2.yaml**; for all other sequences, please use **kaist.yaml**.

## 🤓 Acknowledgments

Thanks for [Open-VINs](https://github.com/rpng/open_vins), [DSO](https://github.com/JakobEngel/dso) and [VINs-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).
