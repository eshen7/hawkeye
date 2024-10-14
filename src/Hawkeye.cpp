#include <iostream>
#include <gtsam/geometry/Point3.h>
#include <nlohmann/json.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/objdetect/aruco_detector.hpp>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/DoubleArrayTopic.h>
#include <ntcore/networktables/BooleanTopic.h>
#include <ntcore/ntcore.h>
#include "TrackedTarget.h"
#include "RobotTracker.h"

using namespace cv;
using namespace std;
using namespace gtsam;
using namespace nt;

std::tuple<Mat, Mat> loadCameraModel(const std::string &path)
{
    // read camera intrinsics from json
    std::ifstream file(path);
    if (!file.is_open()) {
        cerr << "Could not open file " << path << endl;
        return std::make_tuple(Mat(), Mat());
    }
    nlohmann::json json_data;
    try {
        json_data = nlohmann::json::parse(file);
    } catch (nlohmann::json::parse_error &e) {
        cerr << "Error parsing JSON: " << e.what() << endl;
        return std::make_tuple(Mat(), Mat());
    }

    if (json_data["camera_matrix"].size() < 9 || json_data["distortion_coefficients"].size() < 8) {
        cerr << "Invalid camera matrix size" << endl;
        return std::make_tuple(Mat(), Mat());
    }

    // Camera matrix
    Eigen::Matrix<double, 3, 3> camera_matrix;

    for (int i = 0; i < camera_matrix.rows(); i++)
    {
        for (int j = 0; j < camera_matrix.cols(); j++)
        {
            camera_matrix(i, j) = json_data["camera_matrix"][(i * camera_matrix.cols()) + j];
        }
    }

    // Distortion coefficients
    Eigen::Matrix<double, 8, 1> camera_distortion;

    for (int i = 0; i < camera_distortion.rows(); i++)
    {
        for (int j = 0; j < camera_distortion.cols(); j++)
        {
            camera_distortion(i, j) = json_data["distortion_coefficients"][(i * camera_distortion.cols()) + j];
        }
    }

    Mat newCamMatrix, newDistCoefss;

    eigen2cv(camera_matrix, newCamMatrix);
    eigen2cv(camera_distortion, newDistCoefss);

    file.close();

    return std::make_tuple(newCamMatrix, newDistCoefss);
}

std::map<int, Pose3> loadTagMap(const std::string &path) {
    // load known apriltag map
    std::map<int, Pose3> tagMap;
    std::ifstream file(path);
    if (!file.is_open()) {
        cerr << "Could not open file " << path << endl;
        return tagMap;
    }
    nlohmann::json json_data;
    try {
        json_data = nlohmann::json::parse(file);
    } catch (nlohmann::json::parse_error &e) {
        cerr << "Error parsing JSON: " << e.what() << endl;
        std::map<int, Pose3> map;
        return tagMap;
    }

    for (nlohmann::json tag : json_data["tags"]) {
        nlohmann::json translation = tag.at("pose").at("translation");
        nlohmann::json rotation = tag.at("pose").at("rotation").at("quaternion");
        Point3 point = Point3(translation.at("x"), translation.at("y"), translation.at("z"));
        Rot3 rot = Rot3(Eigen::Quaterniond(rotation.at("W"), rotation.at("X"), rotation.at("Y"), rotation.at("Z")));
        tagMap.insert({tag.at("ID"), Pose3(rot, point)});
    }
    return tagMap;
}

vector<Point2f> reorderCorners(const vector<Point2f> &corners)
{
    // reorder corners from aruco detection to be tl->tr->bl->br
    vector<Point2f> newCorners;
    newCorners.push_back(corners.at(1));
    newCorners.push_back(corners.at(0));
    newCorners.push_back(corners.at(3));
    newCorners.push_back(corners.at(2));
    return newCorners;
}

Pose3 tvecrvecToPose(const std::vector<double> &tvec, const std::vector<double> &rvec)
{
    // convert tvec and rvec from solvePnP to a Pose3 object
    Mat rotMatrix;
    Eigen::Matrix3d eigenMat;
    Rodrigues(rvec, rotMatrix);
    cv2eigen(rotMatrix, eigenMat);
    Eigen::Quaterniond quat(eigenMat);
    Rot3 R(quat);
    Point3 P(tvec.at(0), tvec.at(1), tvec.at(2));
    return Pose3{R, P};
}

std::tuple<Mat, Mat> poseTotvecrvec(const Pose3 *pose)
{
    // convert a Pose3 object to tvec and rvec
    Mat tvec;
    Mat rvec;
    Eigen::Matrix<double, 3, 1> eigenM{pose->x(), pose->y(), pose->z()};
    eigen2cv(eigenM, tvec);
    Eigen::Quaterniond quat = pose->rotation().toQuaternion();
    Eigen::Matrix3d eigenMat = quat.toRotationMatrix();
    eigen2cv(eigenMat, rvec);
    return std::make_tuple(tvec, rvec);
}

void drawTag(Mat &frame, const Mat &rvec, const Mat &tvec, const Mat &cameraMatrix, const Mat &distCoeffs, const vector<Point3d> &singleObjectPoints, const vector<Point3d> &singleObjectPoints3d)
{
    // overlay a 3d box on the frame based on the robot's calculated position
    vector<Point2d> imagePoints(4, Point2d(0.0, 0.0));
    vector<Point2d> imagePoints2(4, Point2d(0.0, 0.0));
    projectPoints(singleObjectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
    projectPoints(singleObjectPoints3d, rvec, tvec, cameraMatrix, distCoeffs, imagePoints2);
    for (int i = 0; i < 4; i++)
    {
        if (i == 3)
        {
            line(frame, imagePoints.at(3), imagePoints.at(0), Scalar(0, 255, 255), 5);
            line(frame, imagePoints2.at(3), imagePoints2.at(0), Scalar(255, 0, 255), 5);
        }
        else
        {
            line(frame, imagePoints.at(i), imagePoints.at(i + 1), Scalar(0, 255, 255), 5);
            line(frame, imagePoints2.at(i), imagePoints2.at(i + 1), Scalar(255, 0, 255), 5);
        }
        line(frame, imagePoints.at(i), imagePoints2.at(i), Scalar(255, 0, 255), 5);
    }
}

void drawAxes(Mat &frame, const Mat &rvec, const Mat &tvec, const Mat &cameraMatrix, const Mat &distCoeffs)
{
    // draw axes of length 0.4 meters on the frame based on the robot's calculated position
    vector<Point2d> imagePoints(4, Point2d(0.0, 0.0));
    vector<Point3d> objectPoints = {Point3d(0, 0, 0), Point3d(0.4, 0, 0), Point3d(0, 0.4, 0), Point3d(0, 0, 0.4)};
    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
    line(frame, imagePoints.at(0), imagePoints.at(1), Scalar(255, 0, 0), 5);
    line(frame, imagePoints.at(0), imagePoints.at(2), Scalar(0, 255, 0), 5);
    line(frame, imagePoints.at(0), imagePoints.at(3), Scalar(0, 0, 255), 5);
}

Pose3 tvecrvecToPoseNeg(const std::vector<double> &tvec, const std::vector<double> &rvec)
{
    // return the inverse of the transformation computed by solvePnP (robot to object -> object to robot)
    return tvecrvecToPose(tvec, rvec).inverse();
}

int main()
{
    int deviceID = 2;
    double markerLengthMeters = 0.1651;
    std::string camModelPath = "./../json/test.json";
    std::string outputPath = "./../json/output.json";
    std::string tagMapPath = "./../json/2024-crescendo.json";
    std::ofstream outputJson(outputPath);
    if (!outputJson.is_open()) {
        cerr << "Could not open output JSON file" << endl;
        return -1;
    }
    nlohmann::json outputData;
    outputData["samples"] = nlohmann::json::array();
    auto [cameraMatrix, distCoeffs] = loadCameraModel(camModelPath);
    map<int, Pose3> knownMap = loadTagMap(tagMapPath);
    vector<Point3d> singleObjectPoints = {
        Point3d(-markerLengthMeters / 2, markerLengthMeters / 2, 0),
        Point3d(markerLengthMeters / 2, markerLengthMeters / 2, 0),
        Point3d(markerLengthMeters / 2, -markerLengthMeters / 2, 0),
        Point3d(-markerLengthMeters / 2, -markerLengthMeters / 2, 0)
    };
    vector<Point3d> singleObjectPoints3d = {
        Point3d(-markerLengthMeters / 2, markerLengthMeters / 2, -markerLengthMeters),
        Point3d(markerLengthMeters / 2, markerLengthMeters / 2, -markerLengthMeters),
        Point3d(markerLengthMeters / 2, -markerLengthMeters / 2, -markerLengthMeters),
        Point3d(-markerLengthMeters / 2, -markerLengthMeters / 2, -markerLengthMeters)
    };
    namedWindow("HawkeyeViz");
    VideoCapture cap(deviceID);
    if (!cap.isOpened())
    {
        cerr << "could not open camera\n";
        return 0;
    }
    RobotTracker tracker(markerLengthMeters);
    Mat markerImage;
    aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    vector<int> markerIds;
    int unknownId = -1;
    vector<Point3d> objectPoints;
    vector<Point2d> imagePoints;
    vector<Point2d> unknownImagePoints;
    map<int, vector<Point2f>> singleImageMap;
    vector<std::vector<cv::Point2f>> markerCorners;
    aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    detectorParams.cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    aruco::ArucoDetector detector(dictionary, detectorParams);
    std::vector<double> translationVectorOutput;
    std::vector<double> rotationVectorOutput;
    int count = -1;
    int tagCount = 0;
    nlohmann::json tagJson;
    tagJson["tags"] = nlohmann::json::array();
    // networktables
    NetworkTableInstance defaultInst = nt::NetworkTableInstance::GetDefault();
    auto table = defaultInst.GetTable("hawkeye");
    BooleanTopic enabledTopic = table->GetBooleanTopic("enabled");
    DoubleArrayTopic poseTopic = table->GetDoubleArrayTopic("pose");
    BooleanSubscriber enabledSub = enabledTopic.Subscribe(true);
    DoubleArrayPublisher posePub = poseTopic.Publish({.keepDuplicates = true});
    defaultInst.StartClient4("hawkeye");
    defaultInst.SetServerTeam(3647);
    bool lastEnabled = true;
    while (true)
    {
        count++;
        Mat frame;
        cap >> frame;
        if (frame.empty())
        {
            cerr << "no image detected";
            return 0;
        }
        Mat viz = frame.clone();
        detector.detectMarkers(frame, markerCorners, markerIds);
        if (!markerIds.empty()) // if tags are detected
        {
            if (tracker.tagMap.targets.empty()) // if no targets have been detected yet
            {
                // add first landmark, create a prior on the original robot pose and the first landmark
                int id = markerIds.at(0);
                vector<Point2f> newCorners = reorderCorners(markerCorners.at(0));
                const Pose3& initialTagPose = knownMap.at(id);
                solvePnP(singleObjectPoints, newCorners, cameraMatrix, distCoeffs, rotationVectorOutput, translationVectorOutput, false);
                tracker.addLandmark(id, &initialTagPose);
                Pose3 tagToRobot = tvecrvecToPoseNeg(translationVectorOutput, rotationVectorOutput);
                Pose3 robotPose = initialTagPose.transformPoseFrom(tagToRobot);
                tracker.addInitialPoseEstimate(&robotPose);
                tracker.addPrior(id, &robotPose, &initialTagPose);
            }
            else
            {
                int unknownCount = 0;
                // gather information from all known markers in frame
                for (int i = 0; i < markerIds.size(); i++)
                {
                    if (tracker.tagMap.targets.count(markerIds.at(i)))
                    {
                        for (Point3d point : tracker.getObjectPointsByTag(markerIds.at(i)))
                        {
                            objectPoints.push_back(point);
                        }
                        vector<Point2f> newCorners = reorderCorners(markerCorners.at(i));
                        singleImageMap.insert({markerIds.at(i), newCorners});
                        for (Point2d point : newCorners)
                        {
                            imagePoints.push_back(point);
                        }
                    }
                    else
                    {
                        // only one new tag allowed per frame
                        if (unknownCount > 0)
                        {
                            continue;
                        }
                        unknownCount++;
                        unknownId = markerIds.at(i);
                        vector<Point2f> newCorners = reorderCorners(markerCorners.at(i));
                        for (Point2d point : newCorners)
                        {
                            unknownImagePoints.push_back(point);
                        }
                    }
                }
                if (imagePoints.size() % 4 == 0 && !imagePoints.empty())
                {
                    // tvec and rvec are the vectors transforming the robot to the "object" (in this case, the origin)
                    // calculate pose based on known markers to use as an initial estimate
                    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rotationVectorOutput, translationVectorOutput, false, SOLVEPNP_SQPNP);
                    Pose3 robotPose = tvecrvecToPoseNeg(translationVectorOutput, rotationVectorOutput);
                    tracker.addInitialPoseEstimate(&robotPose);
                    for (pair<int, vector<Point2f>> data : singleImageMap)
                    {
                        // create factors between robot and each known landmark
                        if (data.second.size() == 4)
                        {
                            solvePnP(singleObjectPoints, data.second, cameraMatrix, distCoeffs, rotationVectorOutput, translationVectorOutput, false, SOLVEPNP_IPPE_SQUARE);
                            Pose3 robotToTag = tvecrvecToPose(translationVectorOutput, rotationVectorOutput);
                            tracker.addFactor(data.first, &robotToTag);
                        }
                    }
                    // tvec and rvec are the vectors transforming the robot to the tag
                    // add new landmark if a new tag is detected
                    if (unknownCount > 0)
                    {
                        solvePnP(singleObjectPoints, unknownImagePoints, cameraMatrix, distCoeffs, rotationVectorOutput, translationVectorOutput, false, SOLVEPNP_IPPE_SQUARE);
                        Pose3 robotToTag = tvecrvecToPose(translationVectorOutput, rotationVectorOutput);
                        Pose3 tagPose = tracker.getPose().transformPoseFrom(robotToTag);
                        tracker.addLandmark(unknownId, &tagPose);
                    }
                }
            }
            // perform incremental optimization and clear factor graph for next iteration
            tracker.updateInformation();
        }
        // write all data to output json
        for (pair<int, TrackedTarget> target : tracker.tagMap.targets)
        {
            Pose3 robotToTarget = tracker.getPose().transformPoseTo(target.second.getPose());
            auto [tvec, rvec] = poseTotvecrvec(&robotToTarget);
            drawTag(viz, rvec, tvec, cameraMatrix, distCoeffs, singleObjectPoints, singleObjectPoints3d);
            tagJson["tags"][tagCount]["apriltagId"] = target.first;
            tagJson["tags"][tagCount]["apriltagX"] = target.second.getPose().x();
            tagJson["tags"][tagCount]["apriltagY"] = target.second.getPose().y();
            tagJson["tags"][tagCount]["apriltagZ"] = target.second.getPose().z();
            tagCount++;
        }
        double pose[3] = {tracker.getPose().x(), tracker.getPose().y(), tracker.getPose().rotation().yaw()};
        int64_t time = Now();
        posePub.Set(pose, time);
        Pose3 robotToOrigin = tracker.getPose().transformPoseTo(Pose3());
        auto [tvec2, rvec2] = poseTotvecrvec(&robotToOrigin);
        drawAxes(viz, rvec2, tvec2, cameraMatrix, distCoeffs);
        outputData["samples"][count]["robotPoseX"] = tracker.getPose().x();
        outputData["samples"][count]["robotPoseY"] = tracker.getPose().y();
        outputData["samples"][count]["robotPoseZ"] = tracker.getPose().z();
        outputData["samples"][count]["tags"] = tagJson["tags"];
        tagJson["tags"].clear();
        imshow("HawkeyeViz", viz);
        waitKey(20);
        objectPoints.clear();
        imagePoints.clear();
        unknownImagePoints.clear();
        singleImageMap.clear();
        markerIds.clear();
        unknownId = -1;
        tagCount = 0;
        if (enabledSub.Get() == false && lastEnabled == true) {
            outputJson << outputData.dump(4) << "\n";
            outputData.clear();
            outputData["samples"] = nlohmann::json::array();
        }
        lastEnabled = enabledSub.Get();
    }
    destroyAllWindows();
}
