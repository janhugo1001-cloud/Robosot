#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("arm_tf_listener")
  {
    // frame
    target_frame_ = this->declare_parameter<std::string>("target_frame", "object_frame");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //0.5秒查TF
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&FrameListener::on_timer, this));
    
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped t;

    try {
      t = tf_buffer_->lookupTransform(
        source_frame_,   // 來源 frame
        target_frame_,   // 目標 frame
        tf2::TimePointZero);

      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double z = t.transform.translation.z;

      double distance = std::sqrt(x * x + y * y + z * z);//距離

      RCLCPP_INFO(
        this->get_logger(),
        "Transform from %s to %s:\n"
        "  Translation: [x=%.3f, y=%.3f, z=%.3f]\n"
        "  Rotation:    [x=%.3f, y=%.3f, z=%.3f, w=%.3f]\n"
        "  Distance:    %.3f m",
        source_frame_.c_str(),
        target_frame_.c_str(),
        x, y, z,
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w,
        distance
      );

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform %s to %s: %s",
        source_frame_.c_str(),
        target_frame_.c_str(),
        ex.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string target_frame_;
  std::string source_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
