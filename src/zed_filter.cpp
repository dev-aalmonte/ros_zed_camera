#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"

// #include "zed_interfaces/srv/set_roi.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZedFilter : public rclcpp::Node {
  public:
    ZedFilter() : Node("zed_filter") {
      // Declaring Parameters
      // this->declare_parameter("MIN_X", -1);
      // this->declare_parameter("MAX_X", 0.75);
      // this->declare_parameter("MIN_Y", -0.2);
      // this->declare_parameter("MAX_Y", 0.2);
      // this->declare_parameter("MIN_Z", 0);
      // this->declare_parameter("MAX_Z", 1.25);

      // roi_client_ = this->create_client<zed_interfaces::srv::SetROI>("/zed/zed_node/set_roi");
      // timer_ = this->create_wall_timer(
      //   1s, 
      //   std::bind(&ZedFilter::timer_callback, this)
      // );
      // timer_callback();

      subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered",
        10,
        std::bind(&ZedFilter::topic_callback, this, _1)
      );
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered/filtered",
        10
      );
    }

  private:
    void timer_callback() {
      RCLCPP_INFO(this->get_logger(), "Changing the ROI...");
      std::string topleft = "[0.25,0.20]";
      std::string topright = "[0.75,0.20]";
      std::string bottomright = "[0.75,0.80]";
      std::string bottomleft = "[0.25,0.80]";
      std::string roi = "[" + topleft + "," + topright + "," + bottomright + "," + bottomleft + "]";
      
      // auto req = std::make_shared<zed_interfaces::srv::SetROI::Request>();
      // req->roi = roi;
    
      // while (!roi_client_->wait_for_service(1s)) {
      //   if (!rclcpp::ok()) {
      //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      //   }
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      // }

      // auto result = roi_client_->async_send_request(req);
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // Convert to PCL
      std::shared_ptr<pcl::PCLPointCloud2> cloud = std::make_shared<pcl::PCLPointCloud2>();
      pcl_conversions::toPCL(*msg, *cloud);

      // Vortex Filter
      pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
      vox.setInputCloud(cloud);
      vox.setLeafSize(0.01, 0.01, 0.01);

      pcl::PCLPointCloud2 cloud_vox;
      vox.filter(cloud_vox);

      // Convert to pcl::PointXYZRGB
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromPCLPointCloud2(cloud_vox, *point_rgb);

      // Filter
      double min_x = -1;
      double max_x = 0.75;
      double min_y = -0.2;
      double max_y = 0.2;
      double min_z = 0;
      double max_z = 1.25;

      pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr conditions(new pcl::ConditionAnd<pcl::PointXYZRGB>());
      conditions->addComparison(
        pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>(
          "z", pcl::ComparisonOps::LE, max_z
        ))
      );
      conditions->addComparison(
        pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>(
          "z", pcl::ComparisonOps::GE, min_z
        ))
      );
      conditions->addComparison(
        pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>(
          "y", pcl::ComparisonOps::LE, max_y
        ))
      );
      conditions->addComparison(
        pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>(
          "y", pcl::ComparisonOps::GE, min_y
        ))
      );
      conditions->addComparison(
        pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>(
          "x", pcl::ComparisonOps::LE, max_x
        ))
      );
      conditions->addComparison(
        pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>(
          "x", pcl::ComparisonOps::GE, min_x
        ))
      );

      pcl::ConditionalRemoval<pcl::PointXYZRGB> conditional_filter;
      conditional_filter.setCondition(conditions);
      conditional_filter.setInputCloud(point_rgb);

      // double min_limit = this->get_parameter("MIN_LIMIT").as_double();
      // double max_limit = this->get_parameter("MAX_LIMIT").as_double();

      // pcl::CropBox<pcl::PointXYZRGB> box;
      // Eigen::Vector4f min_pt (-.5f, -.15f, -.5f, 1.0f);
      // Eigen::Vector4f max_pt (.5f, .15f, .5f, 1.0f);
      // box.setInputCloud(point_rgb);
      // box.setMin(min_pt);
      // box.setMax(max_pt);

      // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      // sor.setInputCloud(point_rgb);
      // sor.setMeanK(20);
      // sor.setStddevMulThresh(2);

      // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
      // ror.setInputCloud(point_rgb);
      // ror.setRadiusSearch(0.1);
      // ror.setMinNeighborsInRadius(200);
      // ror.setNegative(true);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_rgb_out(new pcl::PointCloud<pcl::PointXYZRGB>);
      conditional_filter.filter(*point_rgb_out);
      // ror.filter(*point_rgb_out);

      // Convert to pcl::PointCloud
      pcl::PCLPointCloud2 cloud_filtered;
      pcl::toPCLPointCloud2(*point_rgb_out, cloud_filtered);

      // Convert to msg::PointCloud2
      sensor_msgs::msg::PointCloud2 output;
      pcl_conversions::fromPCL(cloud_filtered, output);

      // Publish
      publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    // rclcpp::Client<zed_interfaces::srv::SetROI>::SharedPtr roi_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedFilter>());
  rclcpp::shutdown();
  return 0;
}