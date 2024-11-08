#include "Odometry.h"

#include <networktables/NetworkTableInstance.h>

Odometry::Odometry() : odoTopic(
                           nt::NetworkTableInstance::GetDefault().GetTable("hawkeye")->GetDoubleArrayTopic("odom")),
                       odoSubscriber(odoTopic.Subscribe({}, {
                                                            .keepDuplicates = true
                                                        })) {
}

void Odometry::getUpdate(std::queue<int> &timestamps,
                         const std::function<void(const gtsam::Pose3 *, int)> &addOdoMeasurement) {
    const std::vector<nt::TimestampedDoubleArray> odoQueue = odoSubscriber.ReadQueue();

    for (const nt::TimestampedDoubleArray& measurement: odoQueue) {
        if (measurement.time >= timestamps.front()) {
            gtsam::Pose3 pose = pose2DtoPose3(measurement.value);
            addOdoMeasurement(&pose, timestamps.front());
            timestamps.pop();
        }
    }
}

gtsam::Pose3 Odometry::pose2DtoPose3(std::vector<double> pose) {
    gtsam::Rot3 R = gtsam::Rot3::AxisAngle(gtsam::Point3(0, 0, 1), pose.at(2));
    gtsam::Point3 P(pose.at(0), pose.at(1), 0);
    return gtsam::Pose3(R, P);
}
