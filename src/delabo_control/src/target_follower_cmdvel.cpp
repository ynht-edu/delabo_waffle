#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
#include <random>

class TargetFollowerCmdVel {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Subscriber costmap_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient vision_client_;
    
    geometry_msgs::Point last_target_;
    nav_msgs::OccupancyGrid costmap_;
    bool has_target_;
    bool has_costmap_;
    bool vision_enabled_;
    ros::Time last_target_time_;
    
    double target_distance_;
    double goal_tolerance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double linear_kp_;
    double angular_kp_;
    double safety_distance_;
    std::mt19937 rng_;
    
public:
    TargetFollowerCmdVel() : has_target_(false), has_costmap_(false), vision_enabled_(false), rng_(std::random_device{}()) {
        target_sub_ = nh_.subscribe("/delabo_vision/target/position", 10, 
                                   &TargetFollowerCmdVel::targetCallback, this);
        costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1,
                                    &TargetFollowerCmdVel::costmapCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        vision_client_ = nh_.serviceClient<std_srvs::SetBool>("/delabo_vision/toggle_detection");
        
        ros::NodeHandle pnh("~");
        pnh.param("target_distance", target_distance_, 0.8);
        pnh.param("goal_tolerance", goal_tolerance_, 0.2);
        pnh.param("max_linear_vel", max_linear_vel_, 0.3);
        pnh.param("max_angular_vel", max_angular_vel_, 0.8);
        pnh.param("linear_kp", linear_kp_, 0.5);
        pnh.param("angular_kp", angular_kp_, 1.0);
        pnh.param("safety_distance", safety_distance_, 0.5);
        
        ROS_INFO("Parameters loaded: target_distance=%.2f, goal_tolerance=%.2f", 
                 target_distance_, goal_tolerance_);
        
        enableVision();
        
        ROS_INFO("Target follower cmdvel started");
    }
    
    void enableVision() {
        ros::Duration(1.0).sleep();
        
        std_srvs::SetBool srv;
        srv.request.data = true;
        
        if (vision_client_.call(srv)) {
            if (srv.response.success) {
                vision_enabled_ = true;
                ROS_INFO("Vision detection enabled");
            }
        } else {
            ROS_WARN("Could not call vision service");
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
    }
    
    bool isObstacleInDirection(double angle, double check_distance) {
        if (!has_costmap_) return false;
        
        int steps = (int)(check_distance / costmap_.info.resolution);
        double step_size = check_distance / steps;
        
        for (int i = 1; i <= steps; i++) {
            double check_x = i * step_size * cos(angle);
            double check_y = i * step_size * sin(angle);
            
            int mx = (check_x - costmap_.info.origin.position.x) / costmap_.info.resolution + costmap_.info.width/2;
            int my = (check_y - costmap_.info.origin.position.y) / costmap_.info.resolution + costmap_.info.height/2;
            
            if (mx >= 0 && my >= 0 && mx < (int)costmap_.info.width && my < (int)costmap_.info.height) {
                int index = my * costmap_.info.width + mx;
                if (costmap_.data[index] > 50) {
                    return true;
                }
            }
        }
        return false;
    }
    
    double findSafeDirection(double desired_angle) {
        if (!isObstacleInDirection(desired_angle, safety_distance_)) {
            return desired_angle;
        }
        
        std::vector<double> angles = {-0.3, 0.3, -0.6, 0.6, -1.0, 1.0, -1.5, 1.5, M_PI};
        
        for (double offset : angles) {
            double test_angle = desired_angle + offset;
            if (!isObstacleInDirection(test_angle, safety_distance_)) {
                return test_angle;
            }
        }
        
        return desired_angle;
    }
    
    void followTarget() {
        geometry_msgs::Twist cmd;
        
        if (!has_target_) {
            cmd_vel_pub_.publish(cmd);
            return;
        }
        
        double distance = sqrt(last_target_.x * last_target_.x + last_target_.y * last_target_.y);
        double angle_to_target = atan2(last_target_.y, last_target_.x);
        
        if (distance < target_distance_ - goal_tolerance_) {
            // too close - back away while facing target
            double retreat_angle = angle_to_target + M_PI;
            double safe_retreat = findSafeDirection(retreat_angle);
            
            cmd.angular.z = angular_kp_ * angle_to_target;
            cmd.linear.x = -0.15;
            
            // check if backing up is safe
            if (isObstacleInDirection(M_PI, safety_distance_)) {
                // can't back up, try lateral movement
                std::uniform_real_distribution<double> side_dist(-1.0, 1.0);
                cmd.linear.x = 0.0;
                cmd.angular.z = max_angular_vel_ * (side_dist(rng_) > 0 ? 1.0 : -1.0);
            }
            
            ROS_INFO("Too close (%.2fm), retreating", distance);
            
        } else if (distance > target_distance_ + goal_tolerance_) {
            cmd.angular.z = angular_kp_ * angle_to_target;
            cmd.angular.z = std::max(-max_angular_vel_, std::min(max_angular_vel_, cmd.angular.z));
            
            if (fabs(angle_to_target) < 0.3) {
                if (!isObstacleInDirection(0.0, safety_distance_)) {
                    double distance_error = distance - target_distance_;
                    cmd.linear.x = linear_kp_ * distance_error;
                    cmd.linear.x = std::max(0.0, std::min(max_linear_vel_, cmd.linear.x));
                } else {
                    cmd.angular.z = max_angular_vel_ * (angle_to_target > 0 ? 1.0 : -1.0);
                }
            }
            
            ROS_INFO_THROTTLE(1.0, "Approaching: dist=%.2fm, angle=%.2f", distance, angle_to_target);
            
        } else {
            cmd.angular.z = angular_kp_ * angle_to_target * 0.5;
            cmd.angular.z = std::max(-max_angular_vel_, std::min(max_angular_vel_, cmd.angular.z));
            
            ROS_INFO_THROTTLE(2.0, "Maintaining distance: %.2fm", distance);
        }
        
        cmd_vel_pub_.publish(cmd);
    }
    
    void spin() {
        ros::Rate rate(20);
        
        while (ros::ok()) {
            ros::spinOnce();
            
            if (!vision_enabled_) {
                enableVision();
                ros::Duration(1.0).sleep();
                continue;
            }
            
            if (has_target_ && (ros::Time::now() - last_target_time_).toSec() > 2.0) {
                ROS_WARN_THROTTLE(5.0, "Lost target, stopping");
                geometry_msgs::Twist stop_cmd;
                cmd_vel_pub_.publish(stop_cmd);
                has_target_ = false;
            } else {
                followTarget();
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_follower_cmdvel");
    TargetFollowerCmdVel follower;
    follower.spin();
    return 0;
}
