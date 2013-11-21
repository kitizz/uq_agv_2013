# How to shop web #
If you just want to get this up and running with some stereo cameras, this should get you on your merry way.

## Connecting the Cameras ##
When plugging in the cameras, ensure that the LEFT camera is plugged in first. This ensures that it is assigned to /dev/video0.

Next you want to get the uvc_camera node (wiki.ros.org/uvc_camera) publishing the camera frames as ROS topics.
```
roslaunch uvc_camera stereo_node_agvc.launch
```
The stereo_node_agvc.launch file can be found in this repo.

### Calibrating the Cameras
If this is the first time, or the cameras have been in storage for a few days or more, run the calibration. Follow the really nice tutorial here (wiki.ros.org/uvc_camera).

If you're on the Official laptop, there's a script in the ~/bin/ folder called cameraCalibrate or something which will start everything up for you.

The node, stereo_image_proc, turns the stereo data into depth data.

You need to get the stereo_image_proc parameters working nicely. These should stay the same since Kit last set them and they should be fine. But if you are getting crappy results for the disparity map (command for showing it is below), then follow this tutorial here (wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters). You can probably skip to Step 3 and use this command to start the node instead:
```
rosrun stereo_image_proc stereo_image_proc
```
also run this to show the disparity result:
```
rosrun image_view stereo_view stereo:=/ image:=image_rect_color
```
and then run the following to show the GUI for tweaking:
```
rosrun dynamic_reconfigure reconfigure_gui 
```
Read the tuturial for what they all mean. I found the more important ones were: uniqueness_ratio, min_disparity, disparity_range, speckle_range.

### Getting the depth data
The stereo_image_proc node is used to convert the calibrated stereo cameras into a depth map. This node is started by:
```
rosrun stereo_image_proc stereo_image_proc
```

Once that's all running, we're ready to plug some code into it.

# Running some code
The best one to look at will be liveObstacleDetection.py. It shows how to subscribe to the ROS topics for the point cloud and image data. And it also shows how the ObstacleDetector class can be used.

```
./liveObstacleDetection.py
```

