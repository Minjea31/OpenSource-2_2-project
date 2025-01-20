#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <cstring>


#include <opencv2/opencv.hpp>  // OpenCV 헤더 추가
#include <opencv2/imgcodecs.hpp>  // 이미지 코덱 헤더 추가
#include <opencv2/highgui.hpp>  // GUI (이미지 디스플레이 관련)

// 포트와 주소를 상수로 관리
#define SERVER_PORT 65432  // ROS가 서버로 동작할 때 사용하는 포트
#define CLIENT_PORT 65433  // ROS가 클라이언트로 동작할 때 사용하는 포트
#define BUFFER_SIZE 1024

std::string client_ip = "192.168.0.151";  // 클라이언트(MFC) 주소

ros::Publisher pub;

// TCP 서버 클래스 (ROS가 서버 역할)
class TCPServer {
private:
    int server_fd_, client_socket_;
    sockaddr_in address_;

    int setupServer() {
        int opt = 1;
        if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("Socket creation failed");
            return -1;
        }

        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
            perror("Socket options set failed");
            return -1;
        }

        address_.sin_family = AF_INET;
        address_.sin_addr.s_addr = INADDR_ANY;
        address_.sin_port = htons(SERVER_PORT);

        if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0) {
            perror("Socket bind failed");
            return -1;
        }

        if (listen(server_fd_, 3) < 0) {
            perror("Socket listen failed");
            return -1;
        }

        return 0;
    }

    void handleClient() {
        char buffer[BUFFER_SIZE] = {0};
        while (ros::ok()) {
            int valread = read(client_socket_, buffer, BUFFER_SIZE);
            if (valread > 0) {
                try {
                    int received_value = std::stoi(buffer);
                    std_msgs::Int32 msg;
                    msg.data = received_value;
                    pub.publish(msg);  // ROS 토픽으로 데이터 발행
                    ROS_INFO("Received from MFC: %d", received_value);
                } catch (const std::exception &e) {
                    ROS_WARN("Failed to parse received value: %s", e.what());
                }
            } else if (valread == 0) {
                ROS_INFO("Client disconnected");
                break;
            } else {
                perror("Socket read failed");
                break;
            }
        }
        close(client_socket_);
    }

public:
    TCPServer() : server_fd_(-1), client_socket_(-1) {
        memset(&address_, 0, sizeof(address_));
    }

    ~TCPServer() {
        close(server_fd_);
    }

    void start() {
        if (setupServer() != 0) return;

        ROS_INFO("Waiting for MFC connection...");
        socklen_t addrlen = sizeof(address_);
        while (ros::ok()) {
            client_socket_ = accept(server_fd_, (struct sockaddr*)&address_, &addrlen);
            if (client_socket_ < 0) {
                perror("Client connection failed");
                continue;
            }
            ROS_INFO("Client connected");
            handleClient();
        }
    }
};


// TCP 클라이언트 클래스 (ROS가 클라이언트 역할)
class TCPClient {
private:
    std::string server_ip_;
    int server_port_;

public:
    TCPClient(const std::string& ip, int port) : server_ip_(ip), server_port_(port) {}

    void sendData(const cv::Mat& image) {
        int sock = 0;
        sockaddr_in serv_addr;

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            ROS_ERROR("Socket creation failed");
            return;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(server_port_);

        if (inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
            ROS_ERROR("Invalid address or Address not supported");
            close(sock);
            return;
        }

        if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            ROS_ERROR("Connection failed");
            close(sock);
            return;
        }

        // 이미지 데이터를 바이트 스트림으로 변환
        std::vector<uchar> buf;
        bool encoded = cv::imencode(".jpg", image, buf);  // 이미지를 JPG 형식으로 인코딩
        if (!encoded) {
            ROS_ERROR("Failed to encode image");
            close(sock);
            return;
        }

        int image_size = buf.size();

        // 이미지 크기를 먼저 전송 (네트워크에서 바이트 순서 조정)
        int sent_size = send(sock, (char*)&image_size, sizeof(image_size), 0);
        if (sent_size < 0) {
            ROS_ERROR("Failed to send image size");
            close(sock);
            return;
        }

        // 이미지 데이터를 전송
        sent_size = send(sock, (char*)buf.data(), image_size, 0);
        if (sent_size < 0) {
            ROS_ERROR("Failed to send image data");
            close(sock);
            return;
        }

        ROS_INFO("Successfully sent image to MFC, size: %d bytes", image_size);

        close(sock);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_mfc_bridge_node");
    ros::NodeHandle nh;

    pub = nh.advertise<std_msgs::Int32>("ros_int_value", 10);

    // 서버 객체 (ROS가 서버 역할)
    TCPServer server;

    // 클라이언트 객체 (ROS가 클라이언트 역할, MFC로 송신)
    TCPClient client(client_ip, CLIENT_PORT);

    // 카메라 캡처 객체 (OpenCV)
    cv::VideoCapture cap(0);  // 기본 카메라(0번)를 사용

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera");
        return -1;
    }

    // 서버를 스레드로 실행
    std::thread server_thread([&]() {
        server.start();
    });

    ros::Rate rate(10);  // 10Hz로 전송 (이미지를 매 100ms마다 전송)

    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;  // 카메라에서 프레임을 캡처

        if (frame.empty()) {
            ROS_WARN("Empty frame received from camera");
            continue;
        }

        // 클라이언트를 통해 카메라 이미지를 전송
        ROS_INFO("Sending image...");
        client.sendData(frame);

        ros::spinOnce();
        rate.sleep();
    }

    server_thread.join();

    return 0;
}


