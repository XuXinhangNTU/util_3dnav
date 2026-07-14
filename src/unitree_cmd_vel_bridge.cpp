#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <mutex>
#include <memory>
#include <string>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace
{
constexpr const char *kHighStateTopic = "rt/lf/sportmodestate";

void highStateCallback(const void *message)
{
  const auto state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
  (void)state;
}
} // namespace

class UnitreeCmdVelBridge
{
public:
  explicit UnitreeCmdVelBridge(ros::NodeHandle &nh) : nh_(nh)
  {
    nh_.param<std::string>("network_interface", network_interface_, "enp3s0");
    nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    nh_.param<double>("timeout_sec", timeout_sec_, 0.5);
    nh_.param<double>("control_frequency", control_frequency_, 200.0);

    if (control_frequency_ <= 0.0)
    {
      ROS_WARN("[unitree_cmd_vel_bridge] invalid control_frequency %.3f, using 200 Hz",
               control_frequency_);
      control_frequency_ = 200.0;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

    state_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
            kHighStateTopic));
    state_subscriber_->InitChannel(highStateCallback);

    sport_client_.reset(new unitree::robot::go2::SportClient(false));
    sport_client_->SetTimeout(20.0f);
    sport_client_->Init();

    last_cmd_time_ = ros::Time::now();
    cmd_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &UnitreeCmdVelBridge::cmdVelCallback, this);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_),
                                    &UnitreeCmdVelBridge::controlTimerCallback, this);

    ROS_INFO("[unitree_cmd_vel_bridge] network_interface=%s cmd_vel_topic=%s timeout=%.3fs frequency=%.1fHz",
             network_interface_.c_str(), cmd_vel_topic_.c_str(), timeout_sec_, control_frequency_);
  }

  ~UnitreeCmdVelBridge()
  {
    sendStop();
  }

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    has_cmd_ = true;
  }

  void controlTimerCallback(const ros::TimerEvent &)
  {
    geometry_msgs::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double elapsed = (ros::Time::now() - last_cmd_time_).toSec();
      if (!has_cmd_ || elapsed > timeout_sec_)
      {
        cmd = geometry_msgs::Twist();
      }
      else
      {
        cmd = latest_cmd_;
      }
    }

    const int result = sport_client_->Move(cmd.linear.x, cmd.linear.y, cmd.angular.z);
    if (result < 0)
    {
      ROS_WARN_THROTTLE(1.0, "[unitree_cmd_vel_bridge] SportClient.Move failed: %d", result);
    }
  }

  void sendStop()
  {
    if (sport_client_)
    {
      sport_client_->Move(0.0, 0.0, 0.0);
      sport_client_->StopMove();
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Timer control_timer_;

  std::mutex mutex_;
  geometry_msgs::Twist latest_cmd_;
  ros::Time last_cmd_time_;
  bool has_cmd_ = false;

  std::string network_interface_;
  std::string cmd_vel_topic_;
  double timeout_sec_ = 0.5;
  double control_frequency_ = 200.0;

  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>
      state_subscriber_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unitree_cmd_vel_bridge");
  ros::NodeHandle nh("~");

  UnitreeCmdVelBridge bridge(nh);
  ros::spin();

  return 0;
}
