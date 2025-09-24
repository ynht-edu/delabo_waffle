#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/SetBool.h>
#include "delabo_vision/target_detector.h"

class VisionNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher target_pub_;
    ros::ServiceServer toggle_srv_;
    
    delabo_vision::TargetDetector detector_;
    bool detection_enabled_;
    bool camera_info_received_;
    
public:
    VisionNode() : it_(nh_), detection_enabled_(false), camera_info_received_(false) {
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &VisionNode::imageCallback, this);
        camera_info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &VisionNode::cameraInfoCallback, this);
        target_pub_ = nh_.advertise<geometry_msgs::Point>("/delabo_vision/target/position", 10);
        toggle_srv_ = nh_.advertiseService("/delabo_vision/toggle_detection", &VisionNode::toggleDetection, this);
        
        ROS_INFO("Vision node started");
        
        while (!camera_info_received_ && ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
    
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        if (!camera_info_received_) {
            detector_.setCameraInfo(*msg);
            camera_info_received_ = true;
            camera_info_sub_.shutdown();
        }
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!detection_enabled_ || !camera_info_received_) return;
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        delabo_vision::TargetPose pose = detector_.detectRedBall(cv_ptr->image);
        
        if (pose.valid) {
            geometry_msgs::Point target_pos;
            target_pos.x = pose.position[2];
            target_pos.y = -pose.position[0];
            target_pos.z = -pose.position[1];
            
            target_pub_.publish(target_pos);
            
            ROS_INFO_THROTTLE(1.0, "Red ball detected at: x=%.2f, y=%.2f, z=%.2f", 
                             target_pos.x, target_pos.y, target_pos.z);
        }
    }
    
    bool toggleDetection(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        detection_enabled_ = req.data;
        res.success = true;
        res.message = detection_enabled_ ? "Detection enabled" : "Detection disabled";
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "delabo_vision_node");
    VisionNode vision_node;
    ros::spin();
    return 0;
}
