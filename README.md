# Multi-Livox-LiDAR FAST-LIO

---

[![multi_fast_lio](https://user-images.githubusercontent.com/47074271/234742994-20d127e4-7d45-4179-884a-6ab18f649af3.jpg)](https://www.youtube.com/watch?v=FcOf_IFVa_Y&t=311s)


This algorithm uses Multi-Livox LIDAR and originated from [https://github.com/hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)

- Time synchronization was performed using PTP sync.
- Tested with MID360, its built-in IMU, and MID70.

**FAST-LIO**
 (Fast LiDAR-Inertial Odometry) is a computationally efficient and robust LiDAR-inertial odometry package. It fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. Our package addresses many key issues:

## Requirements

---

- FAST-LIO Prerequisites
    - Ubuntu >= 18.04
    - ROS >= melodic [ROS Installation](http://wiki.ros.org/ROS/Installation)
    - PCL >= 1.8, Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
    - Eigen >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).
    - Livox_SDK1 [https://github.com/Livox-SDK/Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
    - Livox_ros_driver1 [https://github.com/Livox-SDK/livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)

- MID-360 Prerequisites
    - Livox_SDK2 [https://github.com/Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
    - Livox_ros_driver2 [https://github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## Multi-LiDAR Connection to Local Computer

---

Ex) LiDAR1: MID70 (SDK1), LiDAR2: MID360 (SDK2)

- Connect LiDAR1 to ethernet port & LiDAR2 to ctype port using ethernet-to-c adapter
- For LiDAR1, set static IP of LiDAR in the Livox Viewer1 (ex. 192.168.1.xx)

and set the IP for ethernet port of your computer as 192.168.1.xx

- For LiDAR2, set static IP of LiDAR in the Livox Viewer2 (ex. 192.158.1.xx)

and set the IP for ethernet of your computer as 192.158.1.xx.

Points IP, IMU IP, LiDAR Info IP should be same as the IP of your computer.

- Config file settings
    - In livox_ros_driver1, set ‘xfer_format’ in livox_lidar_msg.launch or livox_lidar_rviz.launch
        - as 1 to get livox custom msg
        - as 0 or 2 to get pointcloud2 msg
    - In livox_ros_driver2, set ‘multi_topic’ in msg_MID360.launch as 1 to get rostopic as different
        - LiDAR1 (SDK1) ⇒ /livox/lidar
        - LiDAR2 (SDK2) ⇒ /livox/lidar_192_158_1_xx
    - In livox_ros_driver2, set ‘xfer_format’ in msg_MID360.launch
        - as 1 to get livox custom msg
        - as 0 or 2 to get pointcloud2 msg
    - In livox_ros_driver2, set values in MID360_config.json.
        - IPs of host_net_info as the same as computer IP.
        - IP of lidar_configs as the same as LiDAR IP you set before.


```bash
roslaunch livox_ros_driver livox_lidar_msg.launch
roslaunch livox_ros_driver2 msg_MID360.launch
```

## PTP Time Synchronization

---

- First, *ifconfig* to identify connected IPs of your LiDARs. (*ex.* enx00e04d6afc2e, eno2)
- Use *ptpd* command to synchronize

    ```bash
    sudo ptpd -M -i enx00e04d6afc2e -C # Terminal 1
    sudo ptpd -M -i eno2 -L -V # Terminal 2
    ```

    - [option]
        -L : ignore-lock
        -V : for verbose

- You can see two lidars are synchronized to the local computer in each viewer.

Also, check that the timestamps in rostopics are changing according to the current time.

## Multi-LiDAR Calibration

---

The algorithm from [https://github.com/TIERS/tiers-lidars-dataset](https://github.com/TIERS/tiers-lidars-dataset) is utilized.

```bash
git clone https://github.com/TIERS/tiers-lidars-dataset.git
```

You should change some parts of the code in **lidars_extrinsic_comp.cpp**

Change the line 338~342

```bash
Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
Eigen::AngleAxisf init_rot_z(  M_PI / 4 , Eigen::Vector3f::UnitZ());
Eigen::Translation3f init_trans(0.0,0.0,0.0);
Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();
```

to

```bash
Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
//Eigen::AngleAxisf init_rot_z(  M_PI / 4 , Eigen::Vector3f::UnitZ());
Eigen::Translation3f init_trans(0.0,0.0,0.0);
Eigen::Matrix4f init_tf = (init_trans * init_rot_y * init_rot_x).matrix();
```

Then, run your calibration bagfile to match pointclouds

```bash
catkin build
roslaunch dataset_tools lidars_extri_comp.launch
rosbag play calib3.bag /livox/lidar:=/livox_avia /livox/lidar_192_158_1_110:=/os_cloud_node/points
```

Two pointclouds from MID360(yellow) and MID70(green) are matched using GICP

You can get relative transformation information as **x, y, z, yaw, pitch, roll**
apply the information to the config file in our package

## Build

---

In mid360.yaml,

set lid_topic as body topic (mid 360 topic here) as same as IMU topic,

and set multi_lidar configuration from additional livox LiDAR.

```python
common:
    lid_topic:  "/livox/lidar_192_158_1_110" # lidar_192_158_1_110
    imu_topic:  "/livox/imu_192_158_1_110"
    time_sync_en: false         # ONLY turn on when external time synchronization is really not possible

preprocess:
    lidar_type: 0                # 0 for MID360, 1 for Livox SDK1, 2 for Velodyne LiDAR, 3 for ouster LiDAR,
    scan_line: 1
    timestamp_unit: 3                 # 0-second, 1-milisecond, 2-microsecond, 3-nanosecond.
    blind: 0.1

multi_lidar:
    use_multi: true
    lidar_type: 1 # 0 for MID360, 1 for Livox SDK1, 2 for Velodyne LiDAR, 3 for ouster LiDAR,
    lid_topic: "/livox/lidar"
    scan_line: 1
    R_m360_child_r: 3.13459
    R_m360_child_p: 3.1383
    R_m360_child_y: 3.13954
    T_m360_child_x: 0.03035962
    T_m360_child_y: 0.0257171
    T_m360_child_z: -0.0890391
```

```bash
roslaunch fast_lio mapping_mid360.launch
rosbag play data.bag
```
