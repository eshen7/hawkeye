#include "TrackedTarget.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
TrackedTarget::TrackedTarget(int id, const Pose3 *priorPose)
{
    this->id = id;
    this->priorPose = *priorPose;
}
TrackedTarget::TrackedTarget()
{
    this->id = 0;
    this->priorPose = Pose3();
}
void TrackedTarget::update(const Pose3 *pose)
{
    this->priorPose = *pose;
}
Pose3 TrackedTarget::getPose() const
{
    return priorPose;
}