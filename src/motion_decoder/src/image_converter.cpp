#include "rclcpp/rclcpp.hpp"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "apriltag_msgs/msg/april_tag_detection.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "image_transport/image_transport.hpp"

using apriltag_msgs::msg::AprilTagDetection;
using apriltag_msgs::msg::AprilTagDetectionArray;
using sensor_msgs::msg::Image;
using std::placeholders::_1;

namespace mrsd
{
  class ImageConverterNode : public rclcpp::Node
  {
  public:
    ImageConverterNode() : Node("image_converter_node")
    {
      detections_sub_ = this->create_subscription<AprilTagDetectionArray>("detections", 10, std::bind(&ImageConverterNode::detectionsCb, this, _1));
      image_sub_ = image_transport::create_subscription(this, "image_rect", std::bind(&ImageConverterNode::imageCb, this, _1), "raw", rmw_qos_profile_default);
      image_pub_ = image_transport::create_publisher(this, "image_rect_annotated", rmw_qos_profile_default);
    }

    ~ImageConverterNode() {}

  private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr detections_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    AprilTagDetectionArray cached_detections_;

    void imageCb(const Image::ConstSharedPtr msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      for (AprilTagDetection detection : cached_detections_.detections)
      {
        for (auto point : detection.corners)
        {
          cv::circle(cv_ptr->image, cv::Point(point.x, point.y), 10, CV_RGB(0, 255, 0));
        }
      }

      // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

      image_pub_.publish(cv_ptr->toImageMsg());
    }

    /// @brief extract the pose information from the Apriltag detection message
    /// and make it into a Transform.
    /// @param msg
    void detectionsCb(const AprilTagDetectionArray::SharedPtr msg)
    {
      RCLCPP_INFO(get_logger(), "Got detections!");
      cached_detections_ = *msg;

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
  rclcpp::spin(std::make_shared<mrsd::ImageConverterNode>());
  rclcpp::shutdown();
  return 0;
}
