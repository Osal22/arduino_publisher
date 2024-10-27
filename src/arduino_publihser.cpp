#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// ROS1 related headers
#include "String.h"
#include "Motors.h"
#include <motor_data_msgs/msg/motors.hpp>
#include <ros/ros.h>
#include <geometry_msgs/msg/twist.hpp>
#include <motor_data_msgs_ros2/msg/motor_data_msgs_ros.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
using namespace std::chrono_literals;

/*TODO create cmd_vel subscriber in ros2 and than do inverse kinematics create motor data msg publihser 
  in ros1 and publish all those commands to ros1 topic which is than subscribed by arduino*/
class ArduinoPublisher : public rclcpp::Node
{
public:
  ArduinoPublisher() : Node("arduino_publihser") {
    subscription_ = this->create_subscription<motor_data_msgs_ros2::msg::MotorDataMsgsRos>(
      "motors_cmd", 10, std::bind(&ArduinoPublisher::topic_callback, this, std::placeholders::_1));
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "arduino_publihser");
    ros::NodeHandle nh;
    motors_pub = nh.advertise<motors_data_msgs_ros1::Motors>("arduino_pub", 1000);
     timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&ArduinoPublisher::timerCallback, this)
        );
  }
void timerCallback(const ros::TimerEvent&) {
    ROS_INFO("Timer triggered at: %f", ros::Time::now().toSec());
     tf::StampedTransform transform;
    geometry_msgs::msg::TransformStamped transformStamped;
        try {
            // Listening for the transform between the "odom" and "base_link" frames
            listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id = "odom";
            transformStamped.child_frame_id = "base_link";
            
            // Get the translation (x, y, z) from the transform
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();
            
            tf_broadcaster_->sendTransform(transformStamped);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
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
  // ros1 realted timmer
  tf::TransformListener listener;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoPublisher>());
  rclcpp::shutdown();
  return 0;
}