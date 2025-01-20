#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <numeric>

// HSV 값 설정 (노란색)
#define HSV_LOWER cv::Scalar(20, 90, 110)
#define HSV_UPPER cv::Scalar(40, 200, 255)

// 차선 추종 거리
const int LEFT_LANE_DISTANCE = 150; // 회전을 위한 거리 보정값 증가
const int RIGHT_LANE_DISTANCE = 150;

class YellowLineFollower {
private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub_;
    ros::Subscriber image_sub_;
    ros::Subscriber lane_mode_sub_;
    ros::Subscriber ar_id_sub_;

    bool follow_right_lane_;
    bool image_received_;
    bool start_driving_;
    int current_ar_id_;

    float kp_, ki_, kd_;
    float previous_error_, integral_;

    cv::Mat cv_image_;

    double roi_percentage_;
    int offset_height_;

    void initParameters() {
        nh_.param("kp", kp_, 0.007f); // 회전에서 더 민감하게 반응
        nh_.param("ki", ki_, 0.0001f);
        nh_.param("kd", kd_, 0.003f); // 급격한 변화 방지
        nh_.param("roi_percentage", roi_percentage_, 0.3);
        nh_.param("offset_height", offset_height_, 10);
    }

public:
    YellowLineFollower()
        : follow_right_lane_(false), image_received_(false), start_driving_(false),
          current_ar_id_(-1), previous_error_(0), integral_(0) {
        initParameters();
        control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 10);
        image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &YellowLineFollower::imageCallback, this);
        lane_mode_sub_ = nh_.subscribe("/lane_follow_mode", 1, &YellowLineFollower::laneModeCallback, this);
        ar_id_sub_ = nh_.subscribe("/ros_int_value", 1, &YellowLineFollower::arIdCallback, this);
    }

    void laneModeCallback(const std_msgs::Bool::ConstPtr& msg) {
        follow_right_lane_ = msg->data;
    }

    void arIdCallback(const std_msgs::Int32::ConstPtr& msg) {
        current_ar_id_ = msg->data;
        start_driving_ = true;
        ROS_INFO("Received AR ID: %d. Starting vehicle.", current_ar_id_);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            image_received_ = true;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    std::pair<int, int> processImage(const cv::Mat& image) {
        int height = image.rows;
        int width = image.cols;

        int roi_height = static_cast<int>(height * roi_percentage_);
        int roi_start = height - roi_height;

        cv::Mat roi = image(cv::Rect(0, roi_start, width, roi_height));
        cv::Mat hsv, mask;
        cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, HSV_LOWER, HSV_UPPER, mask);

        int band_height = static_cast<int>(roi_height * 0.1);
        cv::Mat offset_band = mask(cv::Rect(0, roi_height - band_height, width, band_height));

        std::vector<cv::Point> left_points, right_points;
        cv::findNonZero(offset_band(cv::Rect(0, 0, width / 2, band_height)), left_points);
        cv::findNonZero(offset_band(cv::Rect(width / 2, 0, width / 2, band_height)), right_points);

        int left_x = left_points.empty() ? -1 : std::accumulate(left_points.begin(), left_points.end(), 0,
            [](int sum, const cv::Point& p) { return sum + p.x; }) / left_points.size();
        int right_x = right_points.empty() ? -1 : (width / 2) + std::accumulate(right_points.begin(), right_points.end(), 0,
            [](int sum, const cv::Point& p) { return sum + p.x; }) / right_points.size();

        return {left_x, right_x};
    }

    void controlVehicle(std::pair<int, int> lane_positions) {
        if (!start_driving_) return;

        int offset = 0;
        if (follow_right_lane_ && lane_positions.second != -1) {
            offset = (cv_image_.cols - lane_positions.second) - RIGHT_LANE_DISTANCE;
        } else if (!follow_right_lane_ && lane_positions.first != -1) {
            offset = lane_positions.first - LEFT_LANE_DISTANCE;
        }

        float base_speed = 0.2; // 회전에서는 속도를 낮춤
        float steering_angle = -(kp_ * offset + ki_ * integral_ + kd_ * (offset - previous_error_));
        steering_angle = std::clamp(steering_angle, -30.0f, 30.0f);

        ackermann_msgs::AckermannDriveStamped msg;
        msg.drive.speed = base_speed;
        msg.drive.steering_angle = steering_angle;
        control_pub_.publish(msg);

        previous_error_ = offset;
    }

    void run() {
        ros::Rate rate(30);
        while (ros::ok()) {
            if (image_received_) {
                auto lane_positions = processImage(cv_image_);
                controlVehicle(lane_positions);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yellow_line_follower_node");
    YellowLineFollower node;
    node.run();
    return 0;
}

