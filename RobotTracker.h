#ifndef ROBOTTRACKER_H
#define ROBOTTRACKER_H

#include <string>
#include <gtsam/geometry/Pose3.h>
#include <opencv4/opencv2/opencv.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <map>

#include "TrackedTarget.h"
#include "TargetMap.h"

using namespace gtsam;

class RobotTracker
{
private:
    double markerLengthMeters;
    int poseCount;
    ISAM2Params params;
    ISAM2 isam;
    Values initialEstimates;
    Pose3 pose;
    NonlinearFactorGraph graph;
    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr tagNoise;

public:
    TargetMap tagMap;
    std::set<int> ids;
    explicit RobotTracker(double markerLengthMeters);
    void addInitialPoseEstimate(const Pose3 *estimatedPose);
    void addOdoFactor(const Pose3 *odoPose);
    void addLandmark(int id, const Pose3 *targetPose);
    void addPrior(int firstTargetId, const Pose3 *initialRobotPose);
    void addFactor(int targetId, const Pose3 *transform);
    void updateInformation();
    void reset();
    std::vector<cv::Point3d> getObjectPointsByTag(int id);
    static cv::Point3d pose3ToPoint3d(const Pose3 *pose);
    Pose3 getPose();
};
#endif