# Hawkeye: Simultaneous Localization and Mapping With AprilTags
Hawkeye is a software designed for use within the FIRST Robotics Competiton that performs fast and accurate pose estimation from AprilTags. Hawkeye does not require prior knowledge of the positions of the AprilTags in order to perform pose estimation, but a known map of tags can be loaded as prior knowledge for Hawkeye to optimize on the fly. Hawkeye can be used both with or without odometry estimates included, but accuracy will be noticeably better when odometry is included as a factor in optimization.
## Optimization
Hawkeye uses GTSAM to implement sensor fusion. iSAM (incremental smoothing and mapping) is used to perform real-time optimization of incoming data, as opposed to traditional SLAM approaches that require reprocessing of previous data every time an update is requested. Hawkeye constructs a strong known prior on the location of the first AprilTag it sees and weaker priors on successive AprilTags to establish an absolute reference in a system where all positions are calculated relative to one another, and to make it more resistant to poor sensor readings when mapping newly detected AprilTags. Odometry data is used as an initial pose estimate for each iteration for Hawkeye to optimize upon. If odometry is not present, an estimation based on all AprilTags in frame is used.
## CAN/NetworkTables (WIP)
## Replay
Hawkeye outputs all pose data per iteration to a JSON file that is replayable through scripts/visualize2.py for debugging and analysis purposes.<br/>
<img width="434" alt="Screenshot 2024-09-19 at 5 16 15 PM" src="https://github.com/user-attachments/assets/5633943b-2779-4198-99dc-5ad93ace02f1">
## Usage (WIP)
