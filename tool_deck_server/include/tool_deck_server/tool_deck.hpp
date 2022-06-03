#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf2_ros/transform_broadcaster.h>
#include <XmlRpcValue.h>

#include <algorithm>

namespace tool_deck_server
{
class Tool
{
public:
  Tool(std::string name, bool active = false);
  ~Tool();

  const std::string get_name();
  void set_frame(const std::string& frame);
  const std::string get_frame();
  void activate();
  void deactivate();
  void set_rest_pose(const geometry_msgs::Pose& pose);
  void set_active_pose(const geometry_msgs::Pose& pose);
  void set_current_pose(const geometry_msgs::Pose& pose);
  const geometry_msgs::Pose get_active_pose();
  const geometry_msgs::Pose* get_current_pose();
  const bool is_active();

protected:
  std::string name_;
  std::string frame_;
  bool active_;
  geometry_msgs::Pose rest_pose_;
  geometry_msgs::Pose active_pose_;
  geometry_msgs::Pose* current_pose_;
};

class InteractiveTool : public Tool
{
public:
  InteractiveTool(std::string name, bool active = false);
  ~InteractiveTool();

  const double get_linear_feedrate()
  {
    return linear_feedrate_;
  }

  void set_linear_feedrate(const double& rate)
  {
    linear_feedrate_ = rate;
  }

  const double get_angular_feedrate()
  {
    return angular_feedrate_;
  }

  void set_angular_feedrate(const double& rate)
  {
    angular_feedrate_ = rate;
  }

  visualization_msgs::InteractiveMarker get_marker();
  visualization_msgs::InteractiveMarker get_menu();

  static const double threshold_minimum_control(const double& control, const double& threshold)
  {
    // take absolute value comparison only
    if (std::abs(control) < std::abs(threshold))
    {
      return 0.0;
    }
    return control;
  }

  static const double bound_control(const double& control, const double& bound)
  {
    // take absolute value of boundary only
    double b = std::abs(bound);
    return (std::max(std::min(control, b), -b));
  }

  static const double filter_linear_control_to_velocity(const double& feedrate, const double& control, const double& dt)
  {
    // some control limits
    double min = 0.1, max = 0.5;

    // filter control input
    double c = bound_control(threshold_minimum_control(control, min), max);

    // create a realistic step with dt
    return (c / max) * feedrate * dt;
  }

  static const double filter_angular_control_to_velocity(const double& feedrate, const double& control,
                                                         const double& dt)
  {
    // some control limits
    double min = 0.2, max = 0.5;

    // filter control input
    double c = bound_control(threshold_minimum_control(control, min), max);

    // create a realistic step with dt
    return (c / max) * (feedrate * (M_PI / 180.0)) * dt;
  }

private:
  double linear_feedrate_;
  double angular_feedrate_;
  visualization_msgs::InteractiveMarker marker_;  // interactive marker for drag control
  visualization_msgs::InteractiveMarker menu_;    // interactive marker for menu
};

class ToolDeck
{
public:
  ToolDeck(geometry_msgs::Pose pose);
  ~ToolDeck();

  const geometry_msgs::Pose* get_pose();
  void set_pose(const geometry_msgs::Pose& pose);
  void add_tool(std::string name, InteractiveTool tool);
  void remove_tool(std::string name);
  InteractiveTool* get_tool_ptr(std::string name);
  void set_current_tool_pose(std::string name, const geometry_msgs::Pose& pose);
  const bool in_deck(std::string name);
  void activate(std::string name);
  void deactivate(std::string name);
  // void set_feedrate(std::string name, const double& rate);
  // void get_feedrate(std::string name, const double& rate);

protected:
  geometry_msgs::Pose deck_pose_;
  std::map<std::string, InteractiveTool> tool_list_;
};

class ToolDeckServer : public ToolDeck
{
public:
  ToolDeckServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ToolDeckServer();

  double cast_xml_to_double(XmlRpc::XmlRpcValue xml_value);

  void tf_timer_callback(const ros::TimerEvent& event);

  void marker_feedback_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void menu_feedback_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // menu entry handlers
  void menu_save(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void menu_set_frame(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void menu_activate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void menu_deactivate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void menu_nearest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  template <int feedrate>
  void change_linear_feed(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    InteractiveTool* tool_ptr = get_tool_ptr(feedback->header.frame_id);
    tool_ptr->set_linear_feedrate(((double)feedrate) / 1000.0);
  }

  template <int feedrate>
  void change_angular_feedrate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    InteractiveTool* tool_ptr = get_tool_ptr(feedback->header.frame_id);
    tool_ptr->set_angular_feedrate(((double)feedrate) / 10.0);
  }

  // runtime configuration persistence
  void save_current_tool_states();
  void load_persistent_tool_states();

private:
  double loop_hz_;  // update rate
  // reference frames
  std::string base_frame_;
  std::string deck_frame_;
  std::string tool_frame_;
  std::string persistence_dir_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;  // tf broadcaster
  ros::Timer tf_timer_;                           // tf timer
  // tool change services
  // interactive elements
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;  // interactive marker server
  interactive_markers::MenuHandler menu_handler_;                                // menu handler
  std::map<int, std::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)>>
      menu_callbacks_;  // menu callback map
};

}  // namespace tool_deck_server
