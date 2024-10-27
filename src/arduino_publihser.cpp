#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <motor_data_msgs_ros2/msg/motor_data_msgs_ros.hpp>

// ROS1 related headers
#include <ros/ros.h>
#include "TfFeedback.h"
#include "Motors.h"

using namespace std::chrono_literals;

/*TODO create cmd_vel subscriber in ros2 and than do inverse kinematics create motor data msg publihser 
  in ros1 and publish all those commands to ros1 topic which is than subscribed by arduino*/
class ArduinoPublisher : public rclcpp::Node
{
public:

  ArduinoPublisher() : Node("arduino_publihser"), logger_(get_logger()) {
    subscription_ = this->create_subscription<motor_data_msgs_ros2::msg::MotorDataMsgsRos>(
      "motors_cmd", 10, std::bind(&ArduinoPublisher::topic_callback, this, std::placeholders::_1));
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "arduino_publihser");
    ros::NodeHandle nh;
    motors_pub = nh.advertise<motors_data_msgs_ros1::Motors>("arduino_pub", 1000);
    tf_sub_=nh.subscribe("odom_data", 1000, &ArduinoPublisher::odom_data_callback,this);
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&ArduinoPublisher::timerCallback, this)
        );
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  void odom_data_callback(const motors_data_msgs_ros1::TfFeedback::ConstPtr& msg){
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    t.transform.rotation=toQuaternion(msg->rot);
    tf_broadcaster_->sendTransform(t);
  }
  void timerCallback() {
      RCLCPP_INFO_STREAM(logger_,"okay action");
 
  }
  geometry_msgs::msg::Quaternion toQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // Roll = 0, Pitch = 0, Yaw = your yaw angle
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();
    return quat_msg;
}
private:

  void topic_callback(const motor_data_msgs_ros2::msg::MotorDataMsgsRos msg){
    motors_data_msgs_ros1::Motors _slam_bot_motors;

    _slam_bot_motors.dir1=msg.dir1;
    _slam_bot_motors.dir2=msg.dir2;
    _slam_bot_motors.dir3=msg.dir3;
    _slam_bot_motors.dir4=msg.dir4;

    _slam_bot_motors.motor1=msg.motor1;
    _slam_bot_motors.motor2=msg.motor2;
    _slam_bot_motors.motor3=msg.motor3;
    _slam_bot_motors.motor4=msg.motor4;

    motors_pub.publish(_slam_bot_motors);
  }
  rclcpp::Subscription<motor_data_msgs_ros2::msg::MotorDataMsgsRos>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  ros::Publisher motors_pub;
  ros::Subscriber tf_sub_;
  motors_data_msgs_ros1::TfFeedback _odom_tf_msg;
  rclcpp::Logger logger_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoPublisher>());
  rclcpp::shutdown();
  return 0;
}