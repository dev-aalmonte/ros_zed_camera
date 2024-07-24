#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "pcl/ModelCoefficients.h"

#include "pcl/search/kdtree.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZedCluster : public rclcpp::Node {
  public:
    ZedCluster() : Node("zed_cluster") {
      // Parameters
      this->declare_parameter("MIN_CLUSTER_SIZE", 100);
      this->declare_parameter("MAX_CLUSTER_SIZE", 5000);

      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered/segmented",
        10,
        std::bind(&ZedCluster::topic_callback, this, _1)
      );
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered/cluster",
        10
      );
      timer_ = this->create_wall_timer(
        1s, 
        std::bind(&ZedCluster::timer_callback, this)
      );
    }

  private:
    void timer_callback() {
      // RCLCPP_INFO(this->get_logger(), "Clusters Count: %d ", maxClusters);
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      std::shared_ptr<pcl::PCLPointCloud2> cloud = std::make_shared<pcl::PCLPointCloud2>();
      pcl::PCLPointCloud2 cloud_clustered;

      pcl_conversions::toPCL(*msg, *cloud);

      // Convert PCLPC2 -> POINTXYZ
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_xyz(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromPCLPointCloud2(*cloud, *point_xyz);

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud(point_xyz);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract;
      int min_cluster_size = this->get_parameter("MIN_CLUSTER_SIZE").as_int();
      int max_cluster_size = this->get_parameter("MAX_CLUSTER_SIZE").as_int();
      extract.setClusterTolerance(0.01);
      extract.setMinClusterSize(min_cluster_size);
      extract.setMaxClusterSize(max_cluster_size);
      extract.setSearchMethod(tree);
      extract.setInputCloud(point_xyz);
      extract.extract(cluster_indices);

      // RCLCPP_INFO(this->get_logger(), "Number of Clusters: %d", (int) cluster_indices.size());
      maxClusters = (int) cluster_indices.size();
      if (maxClusters > 0) {
        for(pcl::PointIndices& cluster : cluster_indices) {
          if (isIrregularShape(&cluster)) {
            continue;
          }
          std::shared_ptr<pcl::PointIndices> inliers;
          inliers = std::make_shared<pcl::PointIndices>(cluster);

          pcl::ExtractIndices<pcl::PCLPointCloud2> extind;
          pcl::PCLPointCloud2 sub_cluster;
          extind.setInputCloud(cloud);
          extind.setIndices(inliers);
          extind.setNegative(false);
          extind.filter(sub_cluster);

          cloud_clustered += sub_cluster;
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_clustered, output);

        publisher_->publish(output);
      }
    }

    bool isIrregularShape(const pcl::PointIndices* cluster) {
      return false;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    int maxClusters = 0;
    int clusterIndex = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedCluster>());
  rclcpp::shutdown();
  return 0;
}