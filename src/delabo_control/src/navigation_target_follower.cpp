#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/SetBool.h>
#include <random>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigationTargetFollower {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Subscriber costmap_sub_;
    ros::Publisher goal_pub_;
    ros::ServiceClient vision_client_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    MoveBaseClient move_base_client_;
    
    geometry_msgs::Point last_target_;
    geometry_msgs::Point last_target_map_;
    nav_msgs::OccupancyGrid costmap_;
    bool has_target_;
    bool has_costmap_;
    bool vision_enabled_;
    ros::Time last_target_time_;
    
    double target_distance_;
    double goal_tolerance_;
    double retreat_distance_;
    std::mt19937 rng_;
    
public:
    NavigationTargetFollower() : tf_listener_(tf_buffer_), move_base_client_("move_base", true), 
                                has_target_(false), has_costmap_(false), vision_enabled_(false), rng_(std::random_device{}()) {
        target_sub_ = nh_.subscribe("/delabo_vision/target/position", 10, 
                                   &NavigationTargetFollower::targetCallback, this);
        costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1,
                                    &NavigationTargetFollower::costmapCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        vision_client_ = nh_.serviceClient<std_srvs::SetBool>("/delabo_vision/toggle_detection");
        
        nh_.param("target_distance", target_distance_, 1.0);
        nh_.param("goal_tolerance", goal_tolerance_, 0.3);
        nh_.param("retreat_distance", retreat_distance_, 2.0);
        
        ROS_INFO("Waiting for move_base server...");
        move_base_client_.waitForServer();
        
        enableVision();
        
        ROS_INFO("Navigation target follower started");
    }
    
    void enableVision() {
        ros::Duration(2.0).sleep(); // wait for vision node
        
        std_srvs::SetBool srv;
        srv.request.data = true;
        
        if (vision_client_.call(srv)) {
            if (srv.response.success) {
                vision_enabled_ = true;
                ROS_INFO("Vision detection enabled");
            } else {
                ROS_WARN("Failed to enable vision: %s", srv.response.message.c_str());
            }
        } else {
            ROS_WARN("Could not call vision service, will retry");
        }
    }
    
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        costmap_ = *msg;
        has_costmap_ = true;
    }
    
    void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        last_target_ = *msg;
        has_target_ = true;
        last_target_time_ = ros::Time::now();
        
        updateTargetMapPosition();
        navigateToTarget();
    }
    
    void updateTargetMapPosition() {
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            double robot_yaw = yaw;
            
            last_target_map_.x = robot_x + 
                (last_target_.x * cos(robot_yaw) - last_target_.y * sin(robot_yaw));
            last_target_map_.y = robot_y + 
                (last_target_.x * sin(robot_yaw) + last_target_.y * cos(robot_yaw));
                
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform error: %s", ex.what());
        }
    }
    
    geometry_msgs::Point findEmptySpace(double robot_x, double robot_y) {
        geometry_msgs::Point empty_spot;
        empty_spot.x = robot_x;
        empty_spot.y = robot_y;
        
        if (!has_costmap_) return empty_spot;
        
        std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);
        std::uniform_real_distribution<double> radius_dist(retreat_distance_, retreat_distance_ + 1.0);
        
        for (int attempts = 0; attempts < 30; attempts++) {
            double angle = angle_dist(rng_);
            double radius = radius_dist(rng_);
            
            double candidate_x = robot_x + radius * cos(angle);
            double candidate_y = robot_y + radius * sin(angle);
            
            if (isSpaceFree(candidate_x, candidate_y)) {
                empty_spot.x = candidate_x;
                empty_spot.y = candidate_y;
                return empty_spot;
            }
        }
        
        // fallback: move away from target
        double dx = robot_x - last_target_map_.x;
        double dy = robot_y - last_target_map_.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist > 0.1) {
            empty_spot.x = robot_x + (dx/dist) * retreat_distance_;
            empty_spot.y = robot_y + (dy/dist) * retreat_distance_;
        }
        
        return empty_spot;
    }
    
    bool isSpaceFree(double x, double y) {
        if (!has_costmap_) return true;
        
        int mx = (x - costmap_.info.origin.position.x) / costmap_.info.resolution;
        int my = (y - costmap_.info.origin.position.y) / costmap_.info.resolution;
        
        if (mx < 0 || my < 0 || mx >= (int)costmap_.info.width || my >= (int)costmap_.info.height) {
            return false;
        }
        
        int index = my * costmap_.info.width + mx;
        return costmap_.data[index] < 50;
    }
    
    void navigateToTarget() {
        if (!has_target_) return;
        
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            double robot_yaw = yaw;
            
            double target_x_base = last_target_.x;
            double target_y_base = last_target_.y;
            
            double distance = sqrt(target_x_base * target_x_base + target_y_base * target_y_base);
            
            // always face target
            double goal_yaw = atan2(last_target_map_.y - robot_y, last_target_map_.x - robot_x);
            
            if (distance < target_distance_) {
                geometry_msgs::Point empty_spot = findEmptySpace(robot_x, robot_y);
                sendGoal(empty_spot.x, empty_spot.y, goal_yaw);
                ROS_INFO("Target too close, moving to empty space while facing target");
                return;
            }
            
            if (distance <= target_distance_ + goal_tolerance_) {
                sendGoal(robot_x, robot_y, goal_yaw);
                ROS_INFO("Maintaining distance, facing target");
                return;
            }
            
            double approach_distance = distance - target_distance_;
            if (approach_distance < 0.5) approach_distance = 0.5;
            
            double scale = approach_distance / distance;
            double approach_x_base = target_x_base * scale;
            double approach_y_base = target_y_base * scale;
            
            double approach_x_map = robot_x + 
                (approach_x_base * cos(robot_yaw) - approach_y_base * sin(robot_yaw));
            double approach_y_map = robot_y + 
                (approach_x_base * sin(robot_yaw) + approach_y_base * cos(robot_yaw));
            
            sendGoal(approach_x_map, approach_y_map, goal_yaw);
            ROS_INFO("Approaching target: dist=%.2fm", distance);
            
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform error: %s", ex.what());
        }
    }
    
    void sendGoal(double x, double y, double yaw) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.orientation = tf2::toMsg(q);
        
        goal_pub_.publish(goal);
        
        move_base_msgs::MoveBaseGoal mb_goal;
        mb_goal.target_pose = goal;
        move_base_client_.sendGoal(mb_goal);
    }
    
    void spin() {
        ros::Rate rate(2);
        
        while (ros::ok()) {
            ros::spinOnce();
            
            if (!vision_enabled_) {
                enableVision();
                ros::Duration(1.0).sleep();
                continue;
            }
            
            if (has_target_ && (ros::Time::now() - last_target_time_).toSec() > 3.0) {
                ROS_WARN_THROTTLE(10.0, "Lost target, stopping");
                move_base_client_.cancelAllGoals();
                has_target_ = false;
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_target_follower");
    NavigationTargetFollower follower;
    follower.spin();
    return 0;
}
