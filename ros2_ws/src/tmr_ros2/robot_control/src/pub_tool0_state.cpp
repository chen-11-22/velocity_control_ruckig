#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener")
  {
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("tool0_state", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      300ms, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations


    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between flange and base frames
    // and send velocity commands for base to reach flange
    try {
      t = tf_buffer_->lookupTransform(
        "base", "flange", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base to flange: %s", ex.what());
      return;
    }

    geometry_msgs::msg::Twist msg;

    msg.linear.x = t.transform.translation.x;
    msg.linear.y = t.transform.translation.y;
    msg.linear.z = t.transform.translation.z;

    tf2::Quaternion q(
        t.transform.rotation.x,t.transform.rotation.y,
        t.transform.rotation.z,t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
   
    msg.angular.x = roll;
    msg.angular.y = pitch;
    msg.angular.z = yaw;
    publisher_->publish(msg);
  }

  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}