#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <queue>
#include <gtsam/geometry/Pose3.h>
#include <ntcore/networktables/DoubleArrayTopic.h>

class Odometry {
public:
    Odometry();

    void getUpdate(std::queue<int> &timestamps,
                   const std::function<void(const gtsam::Pose3 *, int)> &addOdoMeasurement);

private:
    nt::DoubleArrayTopic odoTopic;
    nt::DoubleArraySubscriber odoSubscriber;

    static gtsam::Pose3 pose2DtoPose3(const std::vector<double> pose);
};
#endif
