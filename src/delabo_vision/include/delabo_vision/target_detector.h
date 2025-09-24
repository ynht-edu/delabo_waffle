#ifndef TARGET_DETECTOR_H
#define TARGET_DETECTOR_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/CameraInfo.h>

namespace delabo_vision {

struct TargetPose {
    cv::Vec3d position;
    bool valid;
    TargetPose() : valid(false) {}
};

class TargetDetector {
public:
    TargetDetector();
    TargetPose detectRedBall(const cv::Mat& frame);
    void setCameraInfo(const sensor_msgs::CameraInfo& camera_info);

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool show_debug_;
    
    std::pair<cv::Point2f, float> findRedBall(const cv::Mat& frame);
    float estimateDistance(float ball_radius_pixels);
};

}

#endif
