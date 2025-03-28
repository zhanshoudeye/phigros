//
// Created by baiye on 25-3-9.
//

#include "image_node.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<image_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//初始化订阅者和发布者
image_node::image_node(): Node("image_node") {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/raw_image", 100,
                                                                    std::bind(&image_node::image_callback, this,
                                                                              std::placeholders::_1));
    click_pub_ = this->create_publisher<geometry_msgs::msg::Point32>("/click_position", 10);
}

// 检测蓝色音符并获取其底部坐标保存起来
void image_node::find_blue(cv::Mat &mask, cv::Mat &image, std::vector<cv::Point> &object_points,double angle) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    //查找轮廓
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &contour: contours) {
        double area = cv::contourArea(contour);
        //过滤一些干扰
        if (area > 100) {
            //计算包围矩形和质心
            cv::Rect bounding_box = cv::boundingRect(contour);

            //记录蓝色音符底部坐标
            cv::Point object_point = cv::Point(bounding_box.x+bounding_box.height*std::sin(angle), bounding_box.y + bounding_box.height*std::cos(angle));
            object_points.push_back(object_point);
            //绘制边框
            cv::rectangle(image, bounding_box, cv::Scalar(124, 234, 13), 2);
        }
    }
}

//通过霍夫变换检测直线并找到最长的直线即为判定线
void image_node::find_white(std::vector<cv::Vec4i> &lines, cv::Mat &image, cv::Vec4i &object_line, double &A, double &B,
                            double &C,double &angle) {
    //霍夫变换检测线段
    cv::HoughLinesP(image, lines, 1,CV_PI / 180, 50);
    double max_length = 0,a=0,b=0,c=0,line_angle=0;
    for (auto &line: lines) {
        cv::Point pt1(line[0], line[1]), pt2(line[2], line[3]);
        double dx=pt1.x-pt2.x;
        double dy=pt1.y-pt2.y;


        double length = std::sqrt(dx*dx+dy*dy);
        //筛选最长线段
        if (length > max_length) {
            max_length = length;
            object_line = line;
            a = pt2.y - pt1.y;
            b = pt1.x - pt2.x;
            c = pt2.x * pt1.y - pt2.y * pt1.x;
            double radian =std::atan2(dy,dx);
            angle= radian*(180/CV_PI);
        }

    }
    A=a;
    B=b;
    C=c;
    angle=line_angle;
    //没找到时，打印
    if (lines.empty()) {
        RCLCPP_INFO(this->get_logger(), "empty lines");
    }
}

//根据蓝色音符和判定线的距离发布点击
void image_node::click_blue(std::vector<cv::Point> &object_points, double &A, double &B, double &C) {
    for (auto &blue: object_points) {
        double distance = std::abs(A * blue.x + B * blue.y + C) / std::sqrt(A * A + B * B);
        if (distance <= 10) {
            RCLCPP_INFO(this->get_logger(), "click blue");
            geometry_msgs::msg::Point32 msg;
            msg.x = (float) blue.x;
            msg.y = (float) blue.y;
            msg.z = 0;
            click_pub_->publish(msg);

        }
    }
}

//图像回调函数，处理订阅的图像
void image_node::image_callback(const sensor_msgs::msg::Image msg) {
    cv::Mat image, imageGray, imageEdges, imageHsv;

    //转换ROS图像消息为CV形式
    cv_bridge::CvImagePtr cv_img;
    cv_img = cv_bridge::toCvCopy(msg, msg.encoding);
    cv_img->image.copyTo(image);

    // 图像预处理
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(image, imageHsv, cv::COLOR_BGR2HSV);
    cv::Canny(imageGray, imageEdges, 50, 150);
    //蓝色音符颜色阈值
    cv::Scalar lower_blue = cv::Scalar(0, 100, 150);
    cv::Scalar upper_blue = cv::Scalar(179, 255, 255);
    cv::Mat mask_blue;
    cv::inRange(imageHsv, lower_blue, upper_blue, mask_blue);

    std::vector<cv::Point> blue;
    std::vector<cv::Vec4i> lines;
    cv::Vec4i object_line;
    double A, B, C,angle;
    //检测蓝色音符和判定线
    find_white(lines, imageEdges, object_line, A, B, C, angle);
    find_blue(mask_blue, image, blue, angle);
    //点击蓝色音符
    click_blue(blue, A, B, C);
    // cv::imshow("image", image);
    // cv::imshow("mask", mask_blue);
    // cv::waitKey(1);
}
