#include "rclcpp/rclcpp.hpp"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>

using apriltag_msgs::msg::AprilTagDetectionArray;
using std::placeholders::_1;

namespace mrsd
{
  class MotionDecoderNode : public rclcpp::Node
  {
  public:
    MotionDecoderNode() : Node("motion_decoder_node")
    {
      // Initialize the transform broadcaster
      tf_broadcaster_ =
          std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      detections_sub_ = this->create_subscription<AprilTagDetectionArray>("/detections", 10, std::bind(&MotionDecoderNode::detectionsCb, this, _1));
    }

    ~MotionDecoderNode() {}

  private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr detections_sub_;

    /// @brief extract the pose information from the Apriltag detection message
    /// and make it into a Transform.
    /// @param msg
    void detectionsCb(const AprilTagDetectionArray::SharedPtr msg)
    {
      RCLCPP_INFO(get_logger(), "Got detections!");

      // const std::vector<cv::Point3d> objectPoints{
      //     {-tagsize / 2, -tagsize / 2, 0},
      //     {+tagsize / 2, -tagsize / 2, 0},
      //     {+tagsize / 2, +tagsize / 2, 0},
      //     {-tagsize / 2, +tagsize / 2, 0},
      // };

      // const std::vector<cv::Point2d> imagePoints{
      //     {detection->p[0][0], detection->p[0][1]},
      //     {detection->p[1][0], detection->p[1][1]},
      //     {detection->p[2][0], detection->p[2][1]},
      //     {detection->p[3][0], detection->p[3][1]},
      // };

      // cv::Matx33d cameraMatrix;
      // cameraMatrix(0, 0) = intr[0]; // fx
      // cameraMatrix(1, 1) = intr[1]; // fy
      // cameraMatrix(0, 2) = intr[2]; // cx
      // cameraMatrix(1, 2) = intr[3]; // cy

      // cv::Mat rvec, tvec;
      // cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);
    }
  };
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrsd::MotionDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
