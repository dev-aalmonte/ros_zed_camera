#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "pcl/ModelCoefficients.h"

#include <pcl/point_types.h>
#include "pcl/search/kdtree.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/region_growing_rgb.h"

#include "pcl/filters/model_outlier_removal.h"
#include "pcl/filters/extract_indices.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZedSegmentation : public rclcpp::Node {
  public:
    ZedSegmentation() : Node("zed_segmentation"){
      this->declare_parameter("MODEL_INDEX", 0);
      this->declare_parameter("DISTANCE_THRESHOLD", 0.01);
      this->declare_parameter("MIN_CLUSTER_SIZE", 35);
      this->declare_parameter("MAX_CLUSTER_SIZE", 150);
      this->declare_parameter("DISTANCE_TOLERANCE", 1);
      this->declare_parameter("COLOR_TOLERANCE", 5);

      subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered/filtered", 
        10, 
        std::bind(&ZedSegmentation::topic_callback, this, _1)
      );
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered/segmented",
        10
      );
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // Convert to PCL
      std::shared_ptr<pcl::PCLPointCloud2> cloud = std::make_shared<pcl::PCLPointCloud2>();
      pcl_conversions::toPCL(*msg, *cloud);

      // Convert to pcl::PointXYZRGB
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromPCLPointCloud2(*cloud, *point_rgb);

      // Segmentation
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      // int model_index = this->get_parameter("MODEL_INDEX").as_int();
      // pcl::SacModel model = modelList[model_index];
      // double dt = this->get_parameter("DISTANCE_THRESHOLD").as_double();
      int min_cluster = this->get_parameter("MIN_CLUSTER_SIZE").as_int();
      int max_cluster = this->get_parameter("MAX_CLUSTER_SIZE").as_int();
      int distance_tolerance = this->get_parameter("DISTANCE_TOLERANCE").as_int();
      int color_tolerance = this->get_parameter("COLOR_TOLERANCE").as_int();

      pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
      pcl::IndicesPtr indices (new std::vector<int>);
      pcl::removeNaNFromPointCloud(*point_rgb, *indices);
      
      pcl::RegionGrowingRGB<pcl::PointXYZRGB> rgr;
      rgr.setInputCloud(point_rgb);
      rgr.setIndices(indices);
      rgr.setSearchMethod(tree);
      rgr.setDistanceThreshold(distance_tolerance);
      rgr.setPointColorThreshold(color_tolerance);
      rgr.setRegionColorThreshold(1);
      rgr.setMinClusterSize(min_cluster);
      rgr.setMaxClusterSize(max_cluster);

      std::vector<pcl::PointIndices> clusters;
      rgr.extract(clusters);

      pcl::PCLPointCloud2 cloud_clustered;
      int maxClusters = (int) clusters.size();
      // RCLCPP_INFO(this->get_logger(), "Clusters Count: %d ", maxClusters);
      if (maxClusters > 0) {
        // int counter = 0;
        for(pcl::PointIndices& cluster : clusters) {
          // if(counter % 2 == 0) {
          //   counter++;
          //   continue;
          // } 
          std::shared_ptr<pcl::PointIndices> inliers;
          inliers = std::make_shared<pcl::PointIndices>(cluster);

          pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
          pcl::PCLPointCloud2 sub_cluster;
          extract.setInputCloud(cloud);
          extract.setIndices(inliers);
          extract.setNegative(false);
          extract.filter(sub_cluster);

          cloud_clustered += sub_cluster;
          // counter++;
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_clustered, output);

        publisher_->publish(output);
      }

      // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      // seg.setOptimizeCoefficients(true);
      // seg.setModelType(model);
      // seg.setMethodType(pcl::SAC_RANSAC);
      // seg.setDistanceThreshold(dt);
      // seg.setInputCloud(point_rgb);

      // seg.segment(*inliers, *coefficients);

      // if (inliers->indices.size() == 0) {
      //   PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
      // }
      // else {
      //   // Filtering
      //   pcl::ModelOutlierRemoval<pcl::PointXYZRGB> mor;
      //   mor.setModelType(model);
      //   mor.setModelCoefficients(*coefficients);
      //   mor.setThreshold(0.01);
      //   mor.setInputCloud(point_rgb);
      //   mor.setNegative(true);

      //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_rgb_out(new pcl::PointCloud<pcl::PointXYZRGB>);
      //   mor.filter(*point_rgb_out);

      //   // Convert to pcl::PointCloud
      //   pcl::PCLPointCloud2 cloud_projected;
      //   pcl::toPCLPointCloud2(*point_rgb_out, cloud_projected);

      //   // Convert to msg::PointCloud2
      //   sensor_msgs::msg::PointCloud2 output;
      //   pcl_conversions::fromPCL(cloud_projected, output);

      //   // Publish
      //   publisher_->publish(output);
      // }

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::vector<pcl::SacModel> modelList = {
        pcl::SACMODEL_PLANE,
        pcl::SACMODEL_LINE,
        pcl::SACMODEL_CIRCLE2D,
        pcl::SACMODEL_SPHERE,
        pcl::SACMODEL_PARALLEL_LINE,
        pcl::SACMODEL_PERPENDICULAR_PLANE,
        // pcl::SACMODEL_CYLINDER,
        // pcl::SACMODEL_NORMAL_PLANE,
        // pcl::SACMODEL_CONE,
        // pcl::SACMODEL_NORMAL_SPHERE,
        // pcl::SACMODEL_NORMAL_PARALLEL_PLANE,
        pcl::SACMODEL_PARALLEL_PLANE
      };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedSegmentation>());
  rclcpp::shutdown();
  return 0;
}