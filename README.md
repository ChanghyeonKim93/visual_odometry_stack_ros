# visual_odometry_ros
This repository includes 'monocular visual odometry' and 'stereo visual odometry' wrapped by ROS1 and ROS2.

*  **stereo_vo_node:** This module is a stereo visual odometry node. 
   - It requires streaming '**time-synchronized**' stereo images.
     - Time synchronization means both timestamps of stereo images are marked by using same clock source, and the stereo cameras simultaneously start to capture images.
   - This module yields the metric-scale camera motion estimations in real-time.

*  **mono_vo_node:** This module is a monocular visual odometry node. 
   - It only requires streaming monocular images. 
   - This module yields the up-to-scale camera motion estimations in real-time.

## Installation
### 1. git clone
```
cd ~/{YOUR_ROS_WS}/src
git clone "https://github.com/ChanghyeonKim93/visual_odometry_ros.git"
```
### 2. build the library & install the library files
```
cd ~/{YOUR_ROS_WS}/src/visual_odometry_ros
mkdir build
cd build
cmake .. && make -j8
sudo make install -y
```
### 3. catkin (colcon) build to make ROS1 (or 2) nodes
#### ROS1
```
cd ~/{YOUR_ROS1_WS}
catkin build visual_odometry_ros
```
#### ROS2
```
cd ~/{YOUR_ROS2_WS}
colcon build --base-path src/visual_odometry_ros
source install/local_setup.bash
source install/setup.bash
```

## Run
#### ROS1
```
cd ~/{YOUR_ROS1_WS}
ros launch visual_odometry stereo_vo.launch.py
```
#### ROS2
```
cd ~/{YOUR_ROS2_WS}
ros2 launch visual_odometry stereo_vo.launch.py
```

## Customization
### Launch file
* In visual_odometry_ros/ros2/launch/stereo, you can see `stereo_vo.launch.py` file.
* `topicname_image_left/right`: (input topic) ros2 topic name of the left/right image. (type: sensor_msgs::msg::Image)
* `directory_intrinsic`: (input dir.) the directory where your parameters, intrinsic and extrinsic files are in.
* `topicname_pose`: (output topic) ros2 topic name of the estimated pose
* `topicname_trajectory`: (output topic) ros2 topic name of the estimated pose trajectory
* `topicname_map_points`: (output topic) ros2 topic name of the estimated map points
* `topicname_debug_image`: (output topic) ros2 topic name of the debugging feature matching image

### Config file
* Configuration yaml file contains visual odometry parameters, intrinsic, extrinsic parameters of the camera rig.
  * Example is based on the `visual_odometry_ros/config/stereo/exp_stereo.yaml`.
#### Camera parameters
* `flagDoUndistortion` (0 or 1): If your images are not rectified, set this 1 to undistort images. To undistort images, you should provide the accurate distortion parameters for each camera.
* `Camera.left/right.fx,fy,cx,cy,width,height`: Camera intrinsic parameters. Camera projection model is assumed to be pinhole model. Skewness of the image pixel is not considered.
* `Camera.left/right.k1,k2,k3,p1,p2`: Camera distortion parameters. Plumb bob distortion model is used. k1,k2,k3 are radial distortion parameters and p1, p2 are the tangential distortion parameters, respectively.
* `T_lr`: SE(3) matrix representing relative pose from the left camera to the right camera. It is 4x4 matrix (SE(3)). You only need to fill `data` field which has the row-major 16 elements corresponding to 4x4 matrix. If you have stereo-rectified stereo images, you just need to set `baselink distance` on the fourth elements of the `data` field (to make SE(3), for the stereo-rectified case, the diagonal elements need to be 1.)

#### Algorithm parameters
* visual_odometry_ros utilizes FAST feature extractor to find salient point features in images and KLT tracker to track the features over the consecutive (and stereo) images. 
* `feature_tracker.thres_error`: Maximum SSD (Sum of Squared Distance) value of the tracked feature to be considered as inlier
* `feature_tracker.thres_bidirection`: Maximum pixel distance of the bidirectionally to be considered as inlier 
* `feature_tracker.thres_sampson`: Maximum Sampson distance of the tracked features to be considered as inlier 
* `feature_tracker.window_size`: Window size for KLT tracker
* `feature_tracker.max_level`: Maximum KLT tracker depths (related course-to-fine track)

* `feature_extractor.n_features`: Maximum number of extracted **raw** FAST features. (recommend: 1000~2000 for realtime performance)
* `feature_extractor.n_bins_u`: The number of bins along the u-axis (of pixel coordinate frame). 
* `feature_extractor.n_bins_v`: The number of bins along the v-axis (of pixel coordinate frame).
  * I select the most strong feature for each bin. Thus, the maximum number of **selected** features is n_bins_v * n_bins_u.
* `feature_extractor.thres_fastscore`: FAST score threshold for FAST feature detection.  
* `feature_extractor.radius`: radius of non-maximum suppression (NMS).

* `motion_estimator.thres_1p_error`:
* `motion_estimator.thres_5p_error`:
* `motion_estimator.thres_poseba_error`: maximum reprojection error to determine the inlier feature after the pose-only bundle adjustment.
* `motion_estimator.lba.max_iter`: Maximum iterations of local bundle adjustment. This idea is originally found in ORB-SLAM paper. Please refer to the ORB-SLAM, ORB-SLAM2, ORB-SLAM3 papers.
* `keyframe_update.thres_alive_ratio`: 
* `keyframe_update.thres_mean_parallax`: 
* `keyframe_update.thres_trans`: If the norm of translation motion from the last keyframe to the current frame is over the thres_trans, make current frame to keyframe. 
* `keyframe_update.thres_rotation`: If the norm of rotation motion from the last keyframe to the current frame is over the thres_rotation, make current frame to keyframe. 
