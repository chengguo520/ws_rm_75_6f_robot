#include <cstdlib>
#include <cstring>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

extern "C" {
#include "rm_base.h"
}

namespace
{

constexpr int kRm75DeviceMode = ARM_75;

void vendorCallback(CallbackData)
{
}

class Rm75DragTeachNode
{
public:
  Rm75DragTeachNode()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("arm_ip", arm_ip_, "192.168.1.18");
    private_nh_.param("arm_port", arm_port_, 8080);
    private_nh_.param("recv_timeout_ms", recv_timeout_ms_, 5000);
    private_nh_.param("multi_drag_mode", multi_drag_mode_, 1);
    private_nh_.param("use_multi_drag", use_multi_drag_, true);
    private_nh_.param("fallback_to_basic_drag", fallback_to_basic_drag_, true);
    private_nh_.param("zero_force_before_start", zero_force_before_start_, true);
    private_nh_.param("stop_hybrid_before_start", stop_hybrid_before_start_, true);
    private_nh_.param("auto_start", auto_start_, false);
    private_nh_.param("auto_stop_after_sec", auto_stop_after_sec_, 0.0);

    if (multi_drag_mode_ < 0 || multi_drag_mode_ > 3)
    {
      ROS_WARN("Invalid multi_drag_mode=%d. Using mode=1: six-axis force, position-only drag.", multi_drag_mode_);
      multi_drag_mode_ = 1;
    }

    start_srv_ = nh_.advertiseService("/rm75_drag_teach/start", &Rm75DragTeachNode::startDragTeach, this);
    stop_srv_ = nh_.advertiseService("/rm75_drag_teach/stop", &Rm75DragTeachNode::stopDragTeach, this);
  }

  ~Rm75DragTeachNode()
  {
    if (drag_active_)
    {
      ROS_WARN("Node is exiting while drag teach is active. Calling Stop_Drag_Teach.");
      Stop_Drag_Teach(socket_handle_, RM_BLOCK);
      drag_active_ = false;
    }
    closeConnection();
  }

  bool connect()
  {
    ROS_INFO("Initializing RealMan API for RM75 drag teach.");
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
      ROS_WARN("Arm_Sockrt_State returned %d. Continuing only if drag teach commands succeed.", ret);
    }

    ROS_INFO("Ready. Call /rm75_drag_teach/start to enter drag teach, /rm75_drag_teach/stop to exit.");
    if (use_multi_drag_)
    {
      ROS_INFO("Default multi-drag mode=%d: %s", multi_drag_mode_, modeDescription(multi_drag_mode_).c_str());
    }
    else
    {
      ROS_INFO("Default drag mode: basic Start_Drag_Teach current-loop drag.");
    }
    return true;
  }

  void spin()
  {
    if (auto_start_)
    {
      std_srvs::Trigger::Request req;
      std_srvs::Trigger::Response res;
      startDragTeach(req, res);
      ROS_INFO("auto_start result: success=%s message='%s'", res.success ? "true" : "false", res.message.c_str());
      if (!res.success)
      {
        ros::shutdown();
        return;
      }
      if (auto_stop_after_sec_ > 0.0)
      {
        auto_stop_timer_ = nh_.createTimer(
            ros::Duration(auto_stop_after_sec_), &Rm75DragTeachNode::autoStopTimerCallback, this, true);
      }
    }

    ros::spin();
  }

private:
  bool startDragTeach(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    if (!isConnected(response))
    {
      return true;
    }
    if (drag_active_)
    {
      response.success = true;
      response.message = "Drag teach is already active.";
      return true;
    }

    if (stop_hybrid_before_start_)
    {
      // Defensive cleanup: ordinary force-position hybrid control and
      // transparent force-position mode are separate modes. Stopping them before
      // drag teach avoids mixing motion/control modes on the real robot.
      Stop_Force_Postion(socket_handle_, RM_BLOCK);
      Stop_Force_Postion_Move(socket_handle_, RM_BLOCK);
    }

    if (zero_force_before_start_)
    {
      ROS_WARN("Clearing six-axis force bias before drag teach. Tool should be still and not touching the environment.");
      int zero_ret = Clear_Force_Data(socket_handle_, RM_BLOCK);
      if (zero_ret != 0)
      {
        response.success = false;
        response.message = "Clear_Force_Data failed before drag teach: " + std::to_string(zero_ret);
        return true;
      }
    }

    int ret = 0;
    if (use_multi_drag_)
    {
      ROS_WARN("Entering multi drag teach mode=%d (%s). Keep one hand near the emergency stop.",
               multi_drag_mode_, modeDescription(multi_drag_mode_).c_str());
      ret = Start_Multi_Drag_Teach(socket_handle_, multi_drag_mode_, RM_BLOCK);
      if (ret != 0 && fallback_to_basic_drag_)
      {
        ROS_WARN("Start_Multi_Drag_Teach failed with %d. Trying basic Start_Drag_Teach fallback.", ret);
        ret = Start_Drag_Teach(socket_handle_, RM_BLOCK);
        active_mode_label_ = "basic Start_Drag_Teach fallback";
      }
      else
      {
        active_mode_label_ = "multi drag mode " + std::to_string(multi_drag_mode_);
      }
    }
    else
    {
      ROS_WARN("Entering basic Start_Drag_Teach mode. Keep one hand near the emergency stop.");
      ret = Start_Drag_Teach(socket_handle_, RM_BLOCK);
      active_mode_label_ = "basic Start_Drag_Teach";
    }

    if (ret != 0)
    {
      response.success = false;
      response.message = "Drag teach start failed: " + std::to_string(ret);
      return true;
    }

    drag_active_ = true;
    response.success = true;
    response.message = "Drag teach started: " + active_mode_label_;
    return true;
  }

  bool stopDragTeach(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    if (!isConnected(response))
    {
      return true;
    }

    int ret = Stop_Drag_Teach(socket_handle_, RM_BLOCK);
    drag_active_ = false;
    response.success = (ret == 0);
    response.message = response.success ? "Drag teach stopped." : "Stop_Drag_Teach failed: " + std::to_string(ret);
    return true;
  }

  void autoStopTimerCallback(const ros::TimerEvent&)
  {
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    stopDragTeach(req, res);
    ROS_INFO("auto_stop result: success=%s message='%s'", res.success ? "true" : "false", res.message.c_str());
    ros::shutdown();
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

  std::string modeDescription(int mode) const
  {
    switch (mode)
    {
      case 0:
        return "current-loop drag teach";
      case 1:
        return "six-axis force drag teach, position only";
      case 2:
        return "six-axis force drag teach, orientation only";
      case 3:
        return "six-axis force drag teach, position and orientation";
      default:
        return "unknown";
    }
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
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
  ros::Timer auto_stop_timer_;

  std::string arm_ip_;
  int arm_port_ = 8080;
  int recv_timeout_ms_ = 5000;
  int multi_drag_mode_ = 1;
  bool use_multi_drag_ = true;
  bool fallback_to_basic_drag_ = true;
  bool zero_force_before_start_ = true;
  bool stop_hybrid_before_start_ = true;
  bool auto_start_ = false;
  double auto_stop_after_sec_ = 0.0;

  bool api_initialized_ = false;
  bool drag_active_ = false;
  std::string active_mode_label_;
  SOCKHANDLE socket_handle_ = -1;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm75_drag_teach_node");

  Rm75DragTeachNode node;
  if (!node.connect())
  {
    return EXIT_FAILURE;
  }

  node.spin();
  return EXIT_SUCCESS;
}
