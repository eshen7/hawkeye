#ifndef TARGETMAP_H
#define TARGETMAP_H

#include <string>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <map>

#include "TrackedTarget.h"

using namespace gtsam;

class TargetMap
{
private:
public:
    TargetMap();
    void addTag(TrackedTarget *target);
    void updateTag(int id, const Pose3 *pose);
    void update(int id, const Pose3 *pose);

    std::map<int, TrackedTarget> targets;
};
#endif