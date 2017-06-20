# Assorted Fetch Packages

## Fetch Mapping
Using Google Cartographer to build a map for navigation.

To run the mapping with a pre-recorded ros bag, in seperate terminals:
```
roslaunch fetch_mapping fetch_mapping_2d.launch
rosbag play /path/to/bagfile/file.bag
```

[![Watch the video](https://github.com/JamesUnicomb/fetch_ros/blob/master/cas_mapping.png)](https://www.youtube.com/watch?v=wfDlKFzTJpk)
Video Link: https://www.youtube.com/watch?v=wfDlKFzTJpk

## Fetch Navigation

## Fecth Arm Control with Joystick
Python script with modified API from OpenAI Fetch ROSBridge.

[![Watch the video](https://github.com/JamesUnicomb/fetch_ros/blob/master/Fetch_arm_joy.png)](https://www.youtube.com/watch?v=3cNEbnp6NGQ)
Video Link: https://www.youtube.com/watch?v=3cNEbnp6NGQ

## Fetch AR Tracking and Object Manipulation
Assorted python scripts and launch files for moving the arm to a tf frame (projected by an AR tag).

[![Watch the video](https://github.com/JamesUnicomb/fetch_ros/blob/master/fetch_AR_tags.png)](https://www.youtube.com/watch?v=46WrcRfxe9E)
Video Link: https://www.youtube.com/watch?v=46WrcRfxe9E

## Fecth Room Logger
