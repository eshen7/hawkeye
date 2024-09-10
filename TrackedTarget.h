#ifndef TRACKEDTARGET_H
#define TRACKEDTARGET_H

#include <string>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtsam;

class TrackedTarget
{
private:
public:
    int id;
    Pose3 priorPose;
    TrackedTarget();
    TrackedTarget(int id, const Pose3 *priorPose);
    void update(const Pose3 *pose);
    Pose3 getPose() const;
};
#endif