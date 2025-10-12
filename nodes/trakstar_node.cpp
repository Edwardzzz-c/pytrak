/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Author: Sean Seungkook Yun <seungkook.yun@sri.com>
 * Adapted for MyHand by: Ava Chen <ava.chen@columbia.edu> 
 * Adapted for ROS2 by: Cheng Zhang
*/

#include <string>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "trakstar/PointATC3DG.hpp"
#include "trakstar/msg/trakstar_msg.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Visualization
#include <tf2_ros/transform_broadcaster.h>

using namespace trakstar;
using std::string;
using namespace std::chrono_literals;

class TrakstarNode : public rclcpp::Node
{
public:
  TrakstarNode() : Node("trakstar_driver")
  {
    // Declare parameters
    this->declare_parameter<bool>("hemisphere_back", false);
    this->declare_parameter<int>("frequency", 255);
    this->declare_parameter<bool>("range_72inch", false);
    this->declare_parameter<bool>("publish_tf", false);
    
    // Sensor 0 attachment parameters
    this->declare_parameter<double>("pivot_x", 0.0);
    this->declare_parameter<double>("pivot_y", 0.0);
    this->declare_parameter<double>("pivot_z", 0.0);
    this->declare_parameter<double>("attach_roll", 0.0);
    this->declare_parameter<double>("attach_pitch", 0.0);
    this->declare_parameter<double>("attach_yaw", 0.0);
    
    // Sensor 1 attachment parameters
    this->declare_parameter<double>("pivot_x1", 0.0);
    this->declare_parameter<double>("pivot_y1", 0.0);
    this->declare_parameter<double>("pivot_z1", 0.0);
    this->declare_parameter<double>("attach_roll1", 0.0);
    this->declare_parameter<double>("attach_pitch1", 0.0);
    this->declare_parameter<double>("attach_yaw1", 0.0);

    // Get parameters
    hemisphere_back_ = this->get_parameter("hemisphere_back").as_bool();
    frequency_ = this->get_parameter("frequency").as_int();
    range_72inch_ = this->get_parameter("range_72inch").as_bool();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    double px = this->get_parameter("pivot_x").as_double();
    double py = this->get_parameter("pivot_y").as_double();
    double pz = this->get_parameter("pivot_z").as_double();
    double rx = this->get_parameter("attach_roll").as_double();
    double ry = this->get_parameter("attach_pitch").as_double();
    double rz = this->get_parameter("attach_yaw").as_double();
    
    double px1 = this->get_parameter("pivot_x1").as_double();
    double py1 = this->get_parameter("pivot_y1").as_double();
    double pz1 = this->get_parameter("pivot_z1").as_double();
    double rx1 = this->get_parameter("attach_roll1").as_double();
    double ry1 = this->get_parameter("attach_pitch1").as_double();
    double rz1 = this->get_parameter("attach_yaw1").as_double();

    // Initialize hardware
    RCLCPP_INFO(this->get_logger(), "Initializing TRAKSTAR. Please wait....");
    bird_ = std::make_unique<PointATC3DG>();
    
    if(!*bird_) {
      RCLCPP_ERROR(this->get_logger(), "can't open trakstar");
      throw std::runtime_error("Failed to open trakstar");
    }

    RCLCPP_INFO(this->get_logger(), "Frequency: %d", frequency_);
    bird_->setMeasurementRate(static_cast<float>(frequency_));

    RCLCPP_INFO(this->get_logger(), "Initialization Complete.");

    bird_->setSuddenOutputChangeLock(0);
    num_sen_ = bird_->getNumberOfSensors();
    RCLCPP_INFO(this->get_logger(), "Number of trackers: %d", num_sen_);

    if(num_sen_ < 2) {
      RCLCPP_ERROR(this->get_logger(), "at least 2 trackers required");
      throw std::runtime_error("At least 2 trackers required");
    }

    RCLCPP_INFO(this->get_logger(), "Output is set: position/quaternion");
    for(int i = 0; i < num_sen_; i++) {
      bird_->setSensorQuaternion(i);
      if(hemisphere_back_)
        bird_->setSensorHemisphere(i, HEMISPHERE_REAR);
      else
        bird_->setSensorHemisphere(i, HEMISPHERE_FRONT);
    }

    if(range_72inch_)
      bird_->setMaximumRange(true);

    // Setup attachment transforms
    trakstar_attach_pos_ = tf2::Vector3(px, py, pz);
    trakstar_attach_.setEulerYPR(rz, ry, rx);
    
    trakstar_attach_pos1_ = tf2::Vector3(px1, py1, pz1);
    trakstar_attach1_.setEulerYPR(rz1, ry1, rx1);

    // Initialize ROS stuff
    trakstar_pub_ = this->create_publisher<trakstar::msg::TrakstarMsg>("trakstar_msg", 1);
    trakstar_raw_pub_ = this->create_publisher<trakstar::msg::TrakstarMsg>("trakstar_raw_msg", 1);
    
    if(publish_tf_) {
      RCLCPP_INFO(this->get_logger(), "Publishing frame data to TF.");
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Mangle the reported pose into the ROS frame conventions
    ros_to_trakstar_ = tf2::Matrix3x3(-1,  0,  0,
                                       0,  1,  0,
                                       0,  0, -1);

    quat_ = new double[4];

    // Create timer for main loop
    auto timer_period = std::chrono::microseconds(static_cast<int>(1000000.0 / frequency_));
    timer_ = this->create_wall_timer(timer_period, std::bind(&TrakstarNode::timer_callback, this));
  }

  ~TrakstarNode()
  {
    delete[] quat_;
  }

private:
  void timer_callback()
  {
    // Publish data
    auto msg = trakstar::msg::TrakstarMsg();
    msg.header.stamp = this->now();
    msg.n_tracker = num_sen_;

    // Publish raw data
    auto msg_raw = trakstar::msg::TrakstarMsg();
    msg_raw.header.stamp = this->now();
    msg_raw.n_tracker = num_sen_;

    std::vector<geometry_msgs::msg::TransformStamped> transforms(num_sen_);

    for(int i = 0; i < num_sen_; ++i) 
    {
      double dX, dY, dZ;
      bird_->getCoordinatesQuaternion(i, dX, dY, dZ, quat_);
      
      tf2::Vector3 pos(dX, dY, dZ);
      tf2::Quaternion q(-quat_[1], -quat_[2], -quat_[3], quat_[0]);
      tf2::Matrix3x3 mat(q);

      tf2::Transform transform(mat, pos);
      msg_raw.transform[i] = tf2::toMsg(transform);

      mat = ros_to_trakstar_ * mat;

      if(i < 1) {
        mat *= trakstar_attach_;
        pos = ros_to_trakstar_ * pos + mat * trakstar_attach_pos_;
      }
      else {
        mat *= trakstar_attach1_;
        pos = ros_to_trakstar_ * pos + mat * trakstar_attach_pos1_;
      }

      tf2::Transform final_transform(mat, pos);
      msg.transform[i] = tf2::toMsg(final_transform);
    }
    
    trakstar_pub_->publish(msg);
    trakstar_raw_pub_->publish(msg_raw);

    if(tf_broadcaster_)
    {
      std::string frames[4] = {"trakstar0", "trakstar1", "trakstar2", "trakstar3"};
      for(int kk = 0; kk < num_sen_; kk++)
      {
        transforms[kk].header.stamp = msg.header.stamp;
        transforms[kk].header.frame_id = "trakstar_base";
        transforms[kk].child_frame_id = frames[kk];
        transforms[kk].transform = msg.transform[kk];
      }

      tf_broadcaster_->sendTransform(transforms);
    }
  }

  std::unique_ptr<PointATC3DG> bird_;
  int num_sen_;
  bool hemisphere_back_;
  int frequency_;
  bool range_72inch_;
  bool publish_tf_;
  double* quat_;
  
  tf2::Vector3 trakstar_attach_pos_;
  tf2::Matrix3x3 trakstar_attach_;
  tf2::Vector3 trakstar_attach_pos1_;
  tf2::Matrix3x3 trakstar_attach1_;
  tf2::Matrix3x3 ros_to_trakstar_;
  
  rclcpp::Publisher<trakstar::msg::TrakstarMsg>::SharedPtr trakstar_pub_;
  rclcpp::Publisher<trakstar::msg::TrakstarMsg>::SharedPtr trakstar_raw_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<TrakstarNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("trakstar_driver"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }
  
  rclcpp::shutdown();
  return 0;
}
