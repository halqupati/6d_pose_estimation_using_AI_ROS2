#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

class StatisticalFilter : public rclcpp::Node {
public:
    StatisticalFilter() : Node("statistical_outlier_removal") {
        this->declare_parameter("mean_k", 50);
        this->declare_parameter("stddev_mul_thresh", 1.0);
        mean_k_ = this->get_parameter("mean_k").as_int();
        stddev_mul_thresh_ = this->get_parameter("stddev_mul_thresh").as_double();

        // SensorDataQoS avoids reliability mismatch with gazebo/bridge point-cloud publishers.
        const auto qos = rclcpp::SensorDataQoS();
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input", qos, std::bind(&StatisticalFilter::callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "StatisticalFilter started (mean_k=%d, stddev_mul_thresh=%.3f)",
            mean_k_, stddev_mul_thresh_);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *cloud);

        // Keep input fields generic (x/y/z + optional rgb/intensity/etc.).
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(stddev_mul_thresh_);
        sor.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(*cloud_filtered, output);
        output.header = msg->header;
        pub_->publish(output);
    }

    int mean_k_{50};
    double stddev_mul_thresh_{1.0};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatisticalFilter>());
    rclcpp::shutdown();
    return 0;
}
