#include <chrono>
#include <functional>
#include <memory>
#include <string>

// #include "zed_components/zed_camera_component.hpp"
// #include <GL/glew.h>
// #include <GL/freeglut.h>
// #include "GLViewer.hpp"
// #include "yolo.hpp"
// #include <NvInfer.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

// #include "cv_bridge/cv_bridge.h".

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZedOD : public rclcpp::Node {
  public:
    ZedOD() : Node("zed_od") {
      // Create a service client
      zed_od_client_ = this->create_client<std_srvs::srv::SetBool>("/zed/zed_node/enable_obj_det");
      image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed/zed_node/left/image_rect_color",
        10,
        std::bind(&ZedOD::image_callback, this, _1)
      );

      // TODO: Create a publisher that send information about the object detected
    }

    // --- TEMP DELETE ---
    // auto send_request(bool data) {
    //   // Create Request Object
    //   auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    //   request->data = data;

    //   // Check if service available
    //   while(zed_od_client_->wait_for_service(1s)) {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    //   }

    //   // Send Request
    //   return zed_od_client_->async_send_request(request);
    // }
    // --- END TEMP DELETE ---

  private:
    // TODO: Create a method that grab the image and runs it into the model (Using ZED/Yolo example)
    void image_callback(const sensor_msgs::msg::Image::SharedPtr imageMsg) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Image Received: \n %d x %d", imageMsg->width, imageMsg->height);
      
      // Translate from msg to cv (using cvbridge)
      // u_int32_t width = imageMsg->width;
      // u_int32_t height = imageMsg->height;

      // cv_bridge::CvImagePtr cv_ptr;
      // cv_ptr = cv_bridge::toCvCopy(imageMsg);
      // cv::Mat left_cv = cv_ptr->image;
    }

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr zed_od_client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

int main(int argc, char * argv[])
{
  // --- TEMP DELETE (Dont need, using node instead) ---
  // Camera Parameters
  // sl::Camera zed;
  // sl::InitParameters init_parameters;
  // init_parameters.sdk_verbose = false;
  // init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
  // init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

  // auto camera_state = zed.open(init_parameters);
  // if (camera_state != sl::ERROR_CODE::SUCCESS) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error Opening the Camera");
  //   return EXIT_FAILURE;
  // }
  // else {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera Loaded!");
  // }

  // Tracking
  // zed.enablePositionalTracking();
  // sl::ObjectDetectionParameters detection_params;
  // detection_params.enable_tracking = true;
  // detection_params.enable_segmentation = false;
  // detection_params.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;

  // camera_state = zed.enableObjectDetection(detection_params);
  // if (camera_state != sl::ERROR_CODE::SUCCESS) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error Opening the tracking system");
  //   zed.close();
  //   return EXIT_FAILURE;
  // }
  // else {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tracking and detection enabled!");
  // }

  // Viewer

  // std::string engine_path = "";
  // Yolo detector;

  // auto display_resolution = zed.getCameraInformation().camera_configuration.resolution;
  // sl::Mat left_sl, point_cloud;
  // cv::Mat left_cv;
  // sl::ObjectDetectionRuntimeParameters objectTracker_params_rt;
  // sl::Objects objects;
  // sl::Pose cam_w_pose;
  // cam_w_pose.pose_data.setIdentity();

  // auto camera_config = zed.getCameraInformation().camera_configuration;
  // sl::Resolution pc_resolution(
  //     std::min((int) camera_config.resolution.width, 720), 
  //     std::min((int) camera_config.resolution.height, 404)
  //   );
  // auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;

  // --- TEMP DELETE ---
  // rclcpp::init(argc, argv);
  // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("zed_od");
  // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr zed_od_client_ =
  //   node->create_client<std_srvs::srv::SetBool>("/zed/zed_node/enable_obj_det");

  // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // request->data = true;

  // while (!zed_od_client_->wait_for_service(1s)) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
  //     return 0;
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  // }

  // auto result = zed_od_client_->async_send_request(request);

  // // Wait for the result.
  // if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Call sucessful");
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  // }
  // --- END TEMP DELETE ---
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedOD>());
  rclcpp::shutdown();
  return 0;
}