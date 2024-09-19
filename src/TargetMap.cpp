#include "TargetMap.h"

TargetMap::TargetMap() {}

void TargetMap::addTag(TrackedTarget *target)
{
    // add a tag to the map
    targets.insert({target->id, *target});
}

void TargetMap::updateTag(int id, const Pose3 *pose)
{
    // update the position of a tag in the map
    targets.find(id)->second.update(pose);
}

void TargetMap::update(int id, const Pose3 *pose)
{
    // check if a tag exists with the desired id and then either update or add the tag
    if (targets.find(id) != targets.end())
    {
        updateTag(id, pose);
    }
    else
    {
        TrackedTarget newTarget(id, pose);
        addTag(&newTarget);
    }
}