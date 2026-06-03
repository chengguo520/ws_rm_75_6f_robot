#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

extern "C" {
#include "rm_base.h"
}

namespace
{

constexpr int kRm75DeviceMode = ARM_75;
constexpr int kForceAxisCount = 6;

// Vendor callback for transparent-transmission APIs. This read-only node does
// not use transparent transmission, but RM_API_Init expects a callback pointer
// in this SDK version, so we keep a quiet callback here.
void vendorCallback(CallbackData)
{
}

class Rm75ForceReadOnlyNode
{
public:
  Rm75ForceReadOnlyNode()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("arm_ip", arm_ip_, "192.168.1.18");
    private_nh_.param("arm_port", arm_port_, 8080);
    private_nh_.param("recv_timeout_ms", recv_timeout_ms_, 5000);
    private_nh_.param("rate_hz", rate_hz_, 10.0);
    private_nh_.param("zero_on_start", zero_on_start_, false);
    private_nh_.param<std::string>("frame_id", frame_id_, "rm75_tool_force_sensor");

    if (rate_hz_ > 20.0)
    {
      ROS_WARN("Get_Force_Data should not be queried faster than 20 Hz. Clamping %.2f Hz to 20 Hz.", rate_hz_);
      rate_hz_ = 20.0;
    }
    if (rate_hz_ <= 0.0)
    {
      ROS_WARN("Invalid rate_hz %.2f. Using 10 Hz.", rate_hz_);
      rate_hz_ = 10.0;
    }

    wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/rm75_force_sensor/wrench", 10);
    zero_srv_ = nh_.advertiseService("/rm75_force_sensor/zero", &Rm75ForceReadOnlyNode::zeroForceSensor, this);
    stop_hybrid_srv_ = nh_.advertiseService(
        "/rm75_force_control/stop_hybrid", &Rm75ForceReadOnlyNode::stopHybridControl, this);
  }

  ~Rm75ForceReadOnlyNode()
  {
    closeConnection();
  }

  bool connect()
  {
    ROS_INFO("Initializing RealMan API for ARM_75.");
    int ret = RM_API_Init(vendorCallback);
    if (ret != 0)
    {
      ROS_ERROR("RM_API_Init failed: %d", ret);
      return false;
    }
    api_initialized_ = true;

    ROS_INFO("Connecting to RM75 controller at %s:%d.", arm_ip_.c_str(), arm_port_);
    char ip_buffer[64] = {0};
    std::strncpy(ip_buffer, arm_ip_.c_str(), sizeof(ip_buffer) - 1);
    socket_handle_ = Arm_Socket_Start(ip_buffer, arm_port_, kRm75DeviceMode, recv_timeout_ms_);
    if (socket_handle_ < 0)
    {
      ROS_ERROR("Arm_Socket_Start failed, return value: %d", socket_handle_);
      closeConnection();
      return false;
    }

    ret = Arm_Sockrt_State(socket_handle_);
    if (ret != 0)
    {
      ROS_WARN("Arm_Sockrt_State returned %d. Continuing only if force reads succeed.", ret);
    }

    if (zero_on_start_)
    {
      ROS_WARN("zero_on_start=true: clearing six-axis force bias while the arm should be still and unloaded.");
      ret = Clear_Force_Data(socket_handle_, RM_BLOCK);
      if (ret != 0)
      {
        ROS_ERROR("Clear_Force_Data failed on startup: %d", ret);
        return false;
      }
    }

    return true;
  }

  void spin()
  {
    ros::Rate rate(rate_hz_);
    while (ros::ok())
    {
      publishForceOnce();
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  bool zeroForceSensor(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    if (!isConnected(response))
    {
      return true;
    }

    // This changes the sensor bias only; it does not command robot motion. Use
    // it with the tool still, unloaded, and not touching the environment.
    int ret = Clear_Force_Data(socket_handle_, RM_BLOCK);
    response.success = (ret == 0);
    response.message = response.success ? "Clear_Force_Data succeeded." : "Clear_Force_Data failed: " + std::to_string(ret);
    return true;
  }

  bool stopHybridControl(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    if (!isConnected(response))
    {
      return true;
    }

    // Safety service: if a previous test left ordinary force-position hybrid
    // control active, this asks the controller to exit that mode. It does not
    // start force control and does not send any trajectory.
    int ret = Stop_Force_Postion(socket_handle_, RM_BLOCK);
    response.success = (ret == 0);
    response.message = response.success ? "Stop_Force_Postion succeeded." : "Stop_Force_Postion failed: " + std::to_string(ret);
    return true;
  }

  bool isConnected(std_srvs::Trigger::Response& response) const
  {
    if (socket_handle_ >= 0)
    {
      return true;
    }
    response.success = false;
    response.message = "RM75 controller is not connected.";
    return false;
  }

  void publishForceOnce()
  {
    if (socket_handle_ < 0)
    {
      return;
    }

    float force[kForceAxisCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int ret = Get_Force_Data(socket_handle_, force);
    if (ret != 0)
    {
      ROS_WARN_THROTTLE(2.0, "Get_Force_Data failed: %d", ret);
      return;
    }

    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.wrench.force.x = force[0];
    msg.wrench.force.y = force[1];
    msg.wrench.force.z = force[2];
    msg.wrench.torque.x = force[3];
    msg.wrench.torque.y = force[4];
    msg.wrench.torque.z = force[5];
    wrench_pub_.publish(msg);

    ROS_INFO_THROTTLE(1.0,
                      "force[N]=[%.3f, %.3f, %.3f] torque[Nm]=[%.3f, %.3f, %.3f]",
                      force[0], force[1], force[2], force[3], force[4], force[5]);
  }

  void closeConnection()
  {
    if (socket_handle_ >= 0)
    {
      Arm_Socket_Close(socket_handle_);
      socket_handle_ = -1;
    }
    if (api_initialized_)
    {
      RM_API_UnInit();
      api_initialized_ = false;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher wrench_pub_;
  ros::ServiceServer zero_srv_;
  ros::ServiceServer stop_hybrid_srv_;

  std::string arm_ip_;
  std::string frame_id_;
  int arm_port_ = 8080;
  int recv_timeout_ms_ = 5000;
  double rate_hz_ = 10.0;
  bool zero_on_start_ = false;

  bool api_initialized_ = false;
  SOCKHANDLE socket_handle_ = -1;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm75_force_read_only_node");

  Rm75ForceReadOnlyNode node;
  if (!node.connect())
  {
    return EXIT_FAILURE;
  }

  node.spin();
  return EXIT_SUCCESS;
}
