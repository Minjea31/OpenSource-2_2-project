#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "xycar_msgs/xycar_motor.h"
#include <vector>
#include <cmath>

struct ARData {
    std::vector<int> ID;
    std::vector<float> DZ;
    std::vector<float> DX;
};

class ARParkingCrossroad {
private:
    ros::NodeHandle nh_;
    ros::Publisher lane_mode_pub_;
    ros::Publisher motor_pub_;
    ros::Subscriber mfc_sub_;
    ros::Subscriber ar_tag_sub_;

    int active_junction_id_;
    ARData ar_data_;
    bool waiting_for_ar_id_;  // AR ID 수신 대기 상태

    void mfcCallback(const std_msgs::Int32::ConstPtr& msg) {
        active_junction_id_ = msg->data;
        waiting_for_ar_id_ = false;
        ROS_INFO("Received AR ID: %d. Ready for processing.", active_junction_id_);
    }

    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
        ar_data_.ID.clear();
        ar_data_.DZ.clear();
        ar_data_.DX.clear();

        for (const auto& marker : msg->markers) {
            ar_data_.ID.push_back(marker.id);
            ar_data_.DZ.push_back(marker.pose.pose.position.z * 100.0);
            ar_data_.DX.push_back(marker.pose.pose.position.x * 100.0);
        }
    }

    int findClosestAR(float& x_pos, float& z_pos) {
        if (ar_data_.ID.empty()) return -1;

        float min_distance = std::numeric_limits<float>::max();
        int closest_id = -1;

        for (size_t i = 0; i < ar_data_.ID.size(); ++i) {
            if (ar_data_.DZ[i] < min_distance) {
                min_distance = ar_data_.DZ[i];
                closest_id = ar_data_.ID[i];
                x_pos = ar_data_.DX[i];
                z_pos = ar_data_.DZ[i];
            }
        }
        return closest_id;
    }

    void reverseAndAlign(int id) {
        xycar_msgs::xycar_motor msg;
        if (id == 0) {
            msg.angle = -45;
            msg.speed = -3;
            motor_pub_.publish(msg);
            ros::Duration(3).sleep();

            msg.angle = 45;
            msg.speed = 3;
            motor_pub_.publish(msg);
            ros::Duration(3).sleep();
        } else {
            msg.angle = 45;
            msg.speed = -3;
            motor_pub_.publish(msg);
            ros::Duration(3).sleep();

            msg.angle = -45;
            msg.speed = 3;
            motor_pub_.publish(msg);
            ros::Duration(3).sleep();
        }

        msg.angle = 0;
        msg.speed = 0;
        motor_pub_.publish(msg);
    }

public:
    ARParkingCrossroad() : active_junction_id_(-1), waiting_for_ar_id_(true) {
        lane_mode_pub_ = nh_.advertise<std_msgs::Bool>("lane_follow_mode", 10);
        motor_pub_ = nh_.advertise<xycar_msgs::xycar_motor>("xycar_motor", 10);
        mfc_sub_ = nh_.subscribe("ros_int_value", 10, &ARParkingCrossroad::mfcCallback, this);
        ar_tag_sub_ = nh_.subscribe("ar_pose_marker", 10, &ARParkingCrossroad::arTagCallback, this);
    }

    void process() {
        ros::Rate rate(10);
        while (ros::ok()) {
            if (waiting_for_ar_id_) {
                ROS_INFO_THROTTLE(1, "Waiting for AR ID...");
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            float x_pos = 0.0, z_pos = 0.0;
            int closest_id = findClosestAR(x_pos, z_pos);

            if (closest_id == -1) {
                ROS_WARN_THROTTLE(1, "No AR Tag detected");
            } else if (closest_id == active_junction_id_) {
                std_msgs::Bool lane_msg;
                lane_msg.data = (x_pos > 0);
                lane_mode_pub_.publish(lane_msg);
                ROS_INFO("Following %s lane", lane_msg.data ? "right" : "left");
            } else if (closest_id == 3 || closest_id == 4 || closest_id == 0) {
                reverseAndAlign(closest_id);
                ros::Duration(5).sleep();

                if (closest_id == 0) {
                    std_msgs::Bool lane_msg;
                    lane_msg.data = false;
                    lane_mode_pub_.publish(lane_msg);
                    ROS_INFO("Switching to LEFT lane after parking.");
                }
                waiting_for_ar_id_ = true;  // 다음 AR ID 대기
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ar_parking_crossroad_node");
    ARParkingCrossroad node;
    node.process();
    return 0;
}

