# Hawkeye: Simultaneous Localization and Mapping With AprilTags
Hawkeye is a software designed for use within the FIRST Robotics Competiton that performs fast and accurate pose estimation from AprilTags. Hawkeye does not require prior knowledge of the positions of the AprilTags in order to perform pose estimation, but a known map of tags can be loaded as prior knowledge for Hawkeye to optimize on the fly. Hawkeye performs sensor fusion between odometry data received over networktables and vision data to estimate robot and tag poses.
## Optimization
Hawkeye uses GTSAM to implement sensor fusion. iSAM (incremental smoothing and mapping) is used to perform real-time optimization of incoming data, as opposed to traditional SLAM approaches that require reprocessing of previous data every time an update is requested. Hawkeye constructs a strong known prior on the location of the first AprilTag it sees and weaker priors on successive AprilTags to establish an absolute reference in a system where all positions are calculated relative to one another, and to make it more resistant to poor sensor readings when mapping newly detected AprilTags. Timesynced odometry data is also used as factors in conjunction with vision estimates for iSAM to optimize.
## NetworkTables (WIP)
Hawkeye uses NT4 as its primary interface to communicate with other processors. Three topics are used: one to receive odometry data, one to send pose data, and one for enabling/disabling purposes.
## Replay
Hawkeye outputs all pose data per iteration to a JSON file that is replayable through scripts/visualize2.py for debugging and analysis purposes.<br/>
<img width="434" alt="Screenshot 2024-09-19 at 5 16 15 PM" src="https://github.com/user-attachments/assets/5633943b-2779-4198-99dc-5ad93ace02f1">
## Usage
In the root folder execute:

'''
$ mkdir build
$ cd build
$ cmake ..
$ make
'''

WIP

### Important Installation Notes
Hawkeye requires the following libraries to be installed on your system:
OpenCV 4.10.0
CMake 3.22.1 or greater
GTSAM 4.0 or greater
allwpilib 2024.3.2
