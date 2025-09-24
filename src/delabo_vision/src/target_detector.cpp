#include "delabo_vision/target_detector.h"

namespace delabo_vision {

TargetDetector::TargetDetector() {
    ros::NodeHandle nh;
    nh.param("show_vision", show_debug_, false);
    
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 525, 0, 320, 0, 525, 240, 0, 0, 1);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
}

void TargetDetector::setCameraInfo(const sensor_msgs::CameraInfo& camera_info) {
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
        camera_info.K[0], camera_info.K[1], camera_info.K[2],
        camera_info.K[3], camera_info.K[4], camera_info.K[5],
        camera_info.K[6], camera_info.K[7], camera_info.K[8]);
    
    if (camera_info.D.size() >= 5) {
        dist_coeffs_ = (cv::Mat_<double>(5, 1) << 
            camera_info.D[0], camera_info.D[1], camera_info.D[2], 
            camera_info.D[3], camera_info.D[4]);
    }
}

TargetPose TargetDetector::detectRedBall(const cv::Mat& frame) {
    TargetPose pose;
    
    auto result = findRedBall(frame);
    cv::Point2f ball_center = result.first;
    float ball_radius = result.second;
    
    if (ball_center.x < 0 || ball_center.y < 0) {
        return pose;
    }
    
    cv::Point2f image_center(frame.cols / 2.0f, frame.rows / 2.0f);
    float pixel_x = ball_center.x - image_center.x;
    float pixel_y = ball_center.y - image_center.y;
    
    double fx = camera_matrix_.at<double>(0, 0);
    double fy = camera_matrix_.at<double>(1, 1);
    
    double angle_x = atan(pixel_x / fx);
    double angle_y = atan(pixel_y / fy);
    
    float distance = estimateDistance(ball_radius);
    
    pose.position[0] = distance * sin(angle_x);
    pose.position[1] = distance * sin(angle_y);
    pose.position[2] = distance * cos(angle_x);
    pose.valid = true;
    
    return pose;
}

std::pair<cv::Point2f, float> TargetDetector::findRedBall(const cv::Mat& frame) {
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    cv::Mat mask1, mask2;
    cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    if (show_debug_) {
        cv::imshow("Red Mask", mask);
        cv::waitKey(1);
    }
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Point2f best_center(-1, -1);
    float best_radius = 0;
    double best_score = 0;
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 1000) continue;
        
        double perimeter = cv::arcLength(contour, true);
        double circularity = (4 * M_PI * area) / (perimeter * perimeter);
        
        if (circularity < 0.5) continue;
        
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        
        double score = area * circularity;
        if (score > best_score) {
            best_score = score;
            best_center = center;
            best_radius = radius;
        }
    }
    
    if (show_debug_ && best_center.x >= 0) {
        cv::Mat display = frame.clone();
        cv::circle(display, best_center, (int)best_radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(display, best_center, 3, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Red Ball Detection", display);
        cv::waitKey(1);
    }
    
    return std::make_pair(best_center, best_radius);
}

float TargetDetector::estimateDistance(float ball_radius_pixels) {
    float real_radius = 0.15f;
    float focal_length = camera_matrix_.at<double>(0, 0);
    return (real_radius * focal_length) / ball_radius_pixels;
}

}
