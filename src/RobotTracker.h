#ifndef ROBOTTRACKER_H
#define ROBOTTRACKER_H

#include <string>
#include <gtsam/geometry/Pose3.h>
#include <opencv4/opencv2/opencv.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <map>
#include <gtsam/slam/BetweenFactor.h>
#include <units/base.h>
#include <map>

#include "TrackedTarget.h"
#include "TargetMap.h"

using namespace gtsam;

class RobotTracker {
private:
    double markerLengthMeters;
    int timestamp;
    int latestTime;
    std::queue<int> timestamps;
    ISAM2Params params;
    ISAM2 isam;
    Values initialEstimates;
    Pose3 pose;
    NonlinearFactorGraph graph;
    std::unordered_multimap<int, BetweenFactor<Pose3> > factorCache;
    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr tagNoise;

public:
    TargetMap tagMap;
    std::set<int> ids;

    RobotTracker(double markerLengthMeters, int initTime);

    void addInitialPoseEstimate(const Pose3 *estimatedPose, int time);

    void addOdoFactor(const Pose3 *pose, int time);

    void addLandmark(int id, const Pose3 *targetPose);

    void addPrior(int firstTargetId, const Pose3 *initialRobotPose, const Pose3 *initialTagPose);

    void addFactor(int targetId, const Pose3 *transform);

    void updateInformation();

    int getLatestTime();

    void reset();

    void setTimestamp(int time);

    std::queue<int> &getTimestamps();

    std::vector<cv::Point3d> getObjectPointsByTag(int id);

    static cv::Point3d pose3ToPoint3d(const Pose3 *pose);

    Pose3 getPose();
};
#endif
