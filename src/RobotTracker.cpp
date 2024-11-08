#include "RobotTracker.h"
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

RobotTracker::RobotTracker(double markerLengthMeters, int initTime) : markerLengthMeters(markerLengthMeters),
                                                                      timestamp(initTime), latestTime(0),
                                                                      params(ISAM2GaussNewtonParams(), 0.01, 1),
                                                                      isam(params),
                                                                      priorNoise(noiseModel::Diagonal::Sigmas(
                                                                          Vector6(0.1, 0.1, 0.1, 0.03, 0.03, 0.03))),
                                                                      tagNoise(noiseModel::Diagonal::Sigmas(
                                                                          Vector6(0.3, 0.3, 0.3, 0.1, 0.1, 0.1))),
                                                                      tagMap() {
}

void RobotTracker::addPrior(int firstTagId, const Pose3 *initialRobotPose, const Pose3 *initialTagPose) {
    // create priors for the first recorded robot pose and first tag in sight in order to establish absolute position
    graph.push_back(PriorFactor<Pose3>(Symbol('p', timestamp), *initialRobotPose, priorNoise));
    graph.push_back(PriorFactor<Pose3>(Symbol('l', firstTagId), *initialTagPose, priorNoise));
}

void RobotTracker::addLandmark(int id, const Pose3 *targetPose) {
    // add any new tags that have not been seen before and create initial estimates for them
    // no need for priors, priors only used for known locations
    if (ids.find(id) == ids.end()) {
        initialEstimates.insert(Symbol('l', id), *targetPose);
        ids.insert(id);
    }
}

void RobotTracker::addFactor(int targetId, const Pose3 *transform) {
    // add a transform between the robot and a tag to be used as a factor in optimization
    factorCache.emplace(
        timestamp, BetweenFactor<Pose3>(Symbol('p', timestamp), Symbol('l', targetId), *transform, tagNoise));
}

void RobotTracker::addInitialPoseEstimate(const Pose3 *estimatedPose, int time) {
    // take an initial reading from current known tags to later optimize through iSAM's nonlinear optimization
    initialEstimates.insert(Symbol('p', time), *estimatedPose);
}

void RobotTracker::addOdoFactor(const Pose3 *pose, int time) {
    latestTime = time;
    addInitialPoseEstimate(pose, time);
    for (auto it = factorCache.equal_range(time).first; it != factorCache.equal_range(time).second; it++) {
        graph.push_back(it->second);
    }
}

void RobotTracker::updateInformation() {
    // perform incremental optimization based on new data and relinearize as needed
    // update known map of tags based on results
    isam.update(graph, initialEstimates);
    Values results = isam.calculateEstimate();
    pose = results.at<Pose3>(Symbol('p', latestTime));
    for (int id: ids) {
        auto pose = results.at<Pose3>(Symbol('l', id));
        tagMap.update(id, &pose);
    }
    reset();
}

int RobotTracker::getLatestTime() {
    return latestTime;
}


std::vector<cv::Point3d> RobotTracker::getObjectPointsByTag(int id) {
    // get the 3d coordinates of a tag in world coordinates
    Pose3 targetPose = tagMap.targets.find(id)->second.getPose();
    std::vector<cv::Point3d> points;
    Pose3 topLeft = targetPose.transformPoseFrom(
        Pose3(Rot3(), Point3(-markerLengthMeters / 2, markerLengthMeters / 2, 0)));
    points.push_back(pose3ToPoint3d(&topLeft));
    Pose3 topRight = topLeft.transformPoseFrom(Pose3(Rot3(), Point3(markerLengthMeters, 0, 0)));
    points.push_back(pose3ToPoint3d(&topRight));
    Pose3 bottomRight = topRight.transformPoseFrom(Pose3(Rot3(), Point3(0, -markerLengthMeters, 0)));
    points.push_back(pose3ToPoint3d(&bottomRight));
    Pose3 bottomLeft = bottomRight.transformPoseFrom(Pose3(Rot3(), Point3(-markerLengthMeters, 0, 0)));
    points.push_back(pose3ToPoint3d(&bottomLeft));
    return points;
}

void RobotTracker::setTimestamp(int time) {
    timestamps.emplace(time);
    timestamp = time;
}


cv::Point3d RobotTracker::pose3ToPoint3d(const Pose3 *pose) {
    // get Point3d from Pose
    return cv::Point3d{pose->x(), pose->y(), pose->z()};
}

std::queue<int> &RobotTracker::getTimestamps() {
    return timestamps;
}

void RobotTracker::reset() {
    // clear factor graph so redundant data isn't added
    graph.resize(0);
    initialEstimates.clear();
}

Pose3 RobotTracker::getPose() {
    return pose;
}
