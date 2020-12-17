#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#define M_ROWS  2790
#define N_COLS  7

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char** argv){
    ros::init(argc, argv, "largeParkingLot");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cout<<"init code!!"<<endl;
    string filepath = "/media/jixingwu/datasetj/LargeParkingLot/20201214T101548/";


    ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1);
    ros::Publisher pub_image = n.advertise<sensor_msgs::Image>("image", 1);
    ros::Publisher pub_depth = n.advertise<sensor_msgs::Image>("depth", 1);
    // -------------- read position.txt into Eigen Matrix
    ifstream infile;
    infile.open(filepath + "position.txt");
    assert(infile.is_open());
    Eigen::MatrixXd pos_matrix(M_ROWS, N_COLS);
    vector<double> row_vector;
    int index = 0;
    string str;
    while(getline(infile, str)){
        istringstream is(str);
        double t; char ch;
        VectorXd row_matrix(N_COLS); // position.txt有七列
        for (int i = 0; i < N_COLS; ++i) {
            is >> t;
            pos_matrix(index, i) = t;
            is >> ch;
        }
        index++;
    }
    cout<<"pos_matrix rows: "<<pos_matrix.rows()<<endl;
    // ---------------------------------------------------

    for (int row = 0; row < M_ROWS; ++row) {
        Eigen::VectorXd pos_vector(N_COLS);
        pos_vector = pos_matrix.row(row);

        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(pos_vector(0));
        // ----------- pub odometry
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.child_frame_id = "world";
        double yaw_angle = pos_vector(6) * M_PI / 180;
        Quaterniond Q = Quaterniond(cos(yaw_angle/2), 0, 0, sin(yaw_angle/2));
        Vector3d    t = Vector3d(pos_vector(1), pos_vector(2), pos_vector(3));
        odometry.pose.pose.position.x = t.x();
        odometry.pose.pose.position.y = t.y();
        odometry.pose.pose.position.z = t.z();
        odometry.pose.pose.orientation.x = Q.x();
        odometry.pose.pose.orientation.y = Q.y();
        odometry.pose.pose.orientation.z = Q.z();
        odometry.pose.pose.orientation.w = Q.w();
        pub_odometry.publish(odometry);
        // -------------- pub image

        stringstream ss;
        ss << setfill('0') << setw(6) << row;
        string imagePath = "image/" + ss.str() + ".png";
        cv::Mat im_rgb = cv::imread(filepath + imagePath, CV_LOAD_IMAGE_UNCHANGED);
        assert(!im_rgb.empty() && "rgb图片加载失败");
        sensor_msgs::ImagePtr imgRGBMsg = cv_bridge::CvImage(header, "bgr8", im_rgb).toImageMsg();
        pub_image.publish(imgRGBMsg);

        string depthPath = "depth/" + ss.str() + ".png";
        cv::Mat im_depth = cv::imread(filepath + depthPath, CV_LOAD_IMAGE_ANYDEPTH);
        assert(!im_depth.empty() && "depth图片加载失败");
        sensor_msgs::ImagePtr imgDepthMsg = cv_bridge::CvImage(header, "mono8", im_depth).toImageMsg();
        pub_depth.publish(imgDepthMsg);
        printf("process image %d\n", row);
//        cv::imshow("window", im_depth);
//        waitKey(0);
//        return 0;
    }



    return 0;
}
