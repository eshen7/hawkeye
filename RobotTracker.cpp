#include "RobotTracker.h"
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

RobotTracker::RobotTracker(double markerLengthMeters) : markerLengthMeters(markerLengthMeters), poseCount(0), params(ISAM2GaussNewtonParams(), 0.01, 1), isam(params), priorNoise(noiseModel::Diagonal::Sigmas(Vector6(0.1, 0.1, 0.1, 0.03, 0.03, 0.03))), tagNoise(noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3, 0.1, 0.1, 0.1))), tagMap() {}

void RobotTracker::addPrior(int firstTagId, const Pose3 *initialRobotPose)
{
    // create priors for the first recorded robot pose and first tag in sight in order to establish absolute position
    graph.push_back(PriorFactor<Pose3>(Symbol('p', poseCount), *initialRobotPose, priorNoise));
    graph.push_back(PriorFactor<Pose3>(Symbol('l', firstTagId), Pose3(), priorNoise));
}

void RobotTracker::addLandmark(int id, const Pose3 *targetPose)
{
    // add any new tags that have not been seen before and create initial estimates for them
    // no need for priors, priors only used for known locations
    if (ids.find(id) == ids.end())
    {
        initialEstimates.insert(Symbol('l', id), *targetPose);
        ids.insert(id);
    }
}

void RobotTracker::addFactor(int targetId, const Pose3 *transform)
{
    // add a transform between the robot and a tag to be used as a factor in optimization
    graph.push_back(BetweenFactor<Pose3>(Symbol('p', poseCount), Symbol('l', targetId), *transform, tagNoise));
}

void RobotTracker::addInitialPoseEstimate(const Pose3 *estimatedPose)
{
    // take an initial reading from current known tags to later optimize through iSAM's nonlinear optimization
    poseCount++;
    initialEstimates.insert(Symbol('p', poseCount), *estimatedPose);
}

void RobotTracker::addOdoFactor(const Pose3 *odoPose)
{
    graph.push_back(BetweenFactor<Pose3>(Symbol('p', poseCount - 1), Symbol('p', poseCount), pose.transformPoseTo(*odoPose), tagNoise));
}

void RobotTracker::updateInformation()
{
    // perform incremental optimization based on new data and relinearize as needed
    // update known map of tags based on results
    isam.update(graph, initialEstimates);
    Values results = isam.calculateEstimate();
    pose = results.at<Pose3>(Symbol('p', poseCount));
    for (int id : ids)
    {
        auto pose = results.at<Pose3>(Symbol('l', id));
        tagMap.update(id, &pose);
    }
        reset();
}

std::vector<cv::Point3d> RobotTracker::getObjectPointsByTag(int id)
{
    // get the 3d coordinates of a tag in world coordinates
    Pose3 targetPose = tagMap.targets.find(id)->second.getPose();
    std::vector<cv::Point3d> points;
    Pose3 topLeft = targetPose.transformPoseFrom(Pose3(Rot3(), Point3(-markerLengthMeters / 2, markerLengthMeters / 2, 0)));
    points.push_back(pose3ToPoint3d(&topLeft));
    Pose3 topRight = topLeft.transformPoseFrom(Pose3(Rot3(), Point3(markerLengthMeters, 0, 0)));
    points.push_back(pose3ToPoint3d(&topRight));
    Pose3 bottomRight = topRight.transformPoseFrom(Pose3(Rot3(), Point3(0, -markerLengthMeters, 0)));
    points.push_back(pose3ToPoint3d(&bottomRight));
    Pose3 bottomLeft = bottomRight.transformPoseFrom(Pose3(Rot3(), Point3(-markerLengthMeters, 0, 0)));
    points.push_back(pose3ToPoint3d(&bottomLeft));
    return points;
}

cv::Point3d RobotTracker::pose3ToPoint3d(const Pose3 *pose)
{
    // get Point3d from Pose
    return cv::Point3d{pose->x(), pose->y(), pose->z()};
}

void RobotTracker::reset()
{
    // clear factor graph so redundant data isn't added
    graph.resize(0);
    initialEstimates.clear();
}

Pose3 RobotTracker::getPose()
{
    return pose;
}