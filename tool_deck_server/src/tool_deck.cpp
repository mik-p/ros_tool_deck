
#include "tool_deck_server/tool_deck.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <cmath>

#include <boost/filesystem.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace tool_deck_server
{
// class Tool
Tool::Tool(std::string name, bool active)
  : name_(name), frame_(name), active_(active), active_pose_(geometry_msgs::Pose()), rest_pose_(geometry_msgs::Pose())
{
  // set quaternions valid
  rest_pose_.orientation.w = 1.0;
  active_pose_.orientation.w = 1.0;

  if (active)
  {
    current_pose_ = &active_pose_;
  }
  else
  {
    current_pose_ = &rest_pose_;
  }
}

Tool::~Tool()
{
}

const std::string Tool::get_name()
{
  return name_;
}

void Tool::set_frame(const std::string& frame)
{
  frame_ = frame;
}

const std::string Tool::get_frame()
{
  return frame_;
}

void Tool::activate()
{
  active_ = true;
  current_pose_ = &active_pose_;
}

void Tool::deactivate()
{
  active_ = false;
  current_pose_ = &rest_pose_;
}

void Tool::set_rest_pose(const geometry_msgs::Pose& pose)
{
  rest_pose_ = pose;
}

void Tool::set_active_pose(const geometry_msgs::Pose& pose)
{
  active_pose_ = pose;
}

void Tool::set_current_pose(const geometry_msgs::Pose& pose)
{
  if (active_)
  {
    set_active_pose(pose);
  }
  else
  {
    set_rest_pose(pose);
  }
}

const geometry_msgs::Pose Tool::get_active_pose()
{
  return active_pose_;
}

const geometry_msgs::Pose* Tool::get_current_pose()
{
  if (active_)
  {
    current_pose_ = &active_pose_;
  }
  else
  {
    current_pose_ = &rest_pose_;
  }

  return current_pose_;
}

const bool Tool::is_active()
{
  return active_;
}

// class InteractiveTool
InteractiveTool::InteractiveTool(std::string name, bool active)
  : linear_feedrate_(0.5)
  , angular_feedrate_(10)
  , Tool(name, active)
  , marker_(visualization_msgs::InteractiveMarker())
  , menu_(visualization_msgs::InteractiveMarker())
{
  // drag control marker setup
  marker_.name = name_;
  marker_.description = name_ + " Pose";
  marker_.header.frame_id = name_;
  marker_.scale = 0.15;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker_.controls.push_back(control);

  // menu marker setup
  menu_.name = name_ + "_menu";
  menu_.description = name_ + " Menu";
  menu_.header.frame_id = name_;
  menu_.scale = 0.1;
  menu_.pose.position.y = 0.2;

  visualization_msgs::InteractiveMarkerControl menu_control;
  menu_control.always_visible = true;

  menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menu_control.name = "menu";

  // make a box to aim at
  visualization_msgs::Marker box;

  box.type = visualization_msgs::Marker::CUBE;
  box.scale.x = 0.025;
  box.scale.y = 0.025;
  box.scale.z = 0.025;
  box.color.r = 0.6;
  box.color.g = 0.0;
  box.color.b = 0.6;
  box.color.a = 1.0;

  menu_control.markers.push_back(box);

  menu_.controls.push_back(menu_control);
}

InteractiveTool::~InteractiveTool()
{
}

visualization_msgs::InteractiveMarker InteractiveTool::get_marker()
{
  return marker_;
}

visualization_msgs::InteractiveMarker InteractiveTool::get_menu()
{
  return menu_;
}

// class ToolDeck
ToolDeck::ToolDeck(geometry_msgs::Pose pose) : deck_pose_(pose)
{
}

ToolDeck::~ToolDeck()
{
}

const geometry_msgs::Pose* ToolDeck::get_pose()
{
  return &deck_pose_;
}

void ToolDeck::set_pose(const geometry_msgs::Pose& pose)
{
  deck_pose_ = pose;
}

void ToolDeck::add_tool(std::string name, InteractiveTool tool)
{
  tool_list_.insert({ name, tool });
}

void ToolDeck::remove_tool(std::string name)
{
  tool_list_.erase(name);
}

InteractiveTool* ToolDeck::get_tool_ptr(std::string name)
{
  auto tool_list_iter = tool_list_.find(name);
  if (tool_list_iter == tool_list_.end())
  {
    return nullptr;
  }
  return &tool_list_iter->second;
}

void ToolDeck::set_current_tool_pose(std::string name, const geometry_msgs::Pose& pose)
{
  InteractiveTool* tool_ptr = get_tool_ptr(name);
  if (!(tool_ptr == nullptr))
  {
    tool_ptr->set_current_pose(pose);
  }
}

const bool ToolDeck::in_deck(std::string name)
{
  auto tool_list_iter = tool_list_.find(name);
  if (tool_list_iter == tool_list_.end())
  {
    return false;
  }
  return true;
}

void ToolDeck::activate(std::string name)
{
  InteractiveTool* tool_ptr = get_tool_ptr(name);
  if (!(tool_ptr == nullptr))
  {
    tool_ptr->activate();
  }
}

void ToolDeck::deactivate(std::string name)
{
  InteractiveTool* tool_ptr = get_tool_ptr(name);
  if (!(tool_ptr == nullptr))
  {
    tool_ptr->deactivate();
  }
}

// class ToolDeckServer
ToolDeckServer::ToolDeckServer(ros::NodeHandle& nh, ros::NodeHandle& pnh) : ToolDeck(geometry_msgs::Pose())
{
  // get params
  pnh.param("base_frame", base_frame_, std::string("base_link"));
  pnh.param("deck_frame", deck_frame_, std::string("tool_deck"));
  pnh.param("tool_frame", tool_frame_, std::string("tool_ref1"));
  pnh.param("update_rate", loop_hz_, loop_hz_);
  pnh.param("persistence_dir", persistence_dir_, std::string("~/.ros/"));

  // get deck pose
  XmlRpc::XmlRpcValue deck_pose;
  if (!pnh.getParam("deck_pose", deck_pose))
  {
    ROS_WARN_STREAM("missing deck pose: set to default pose");
    deck_pose_.orientation.w = 1.0;
  }
  else
  {
    geometry_msgs::Pose pose;

    pose.position.x = cast_xml_to_double(deck_pose[0][0]);
    pose.position.y = cast_xml_to_double(deck_pose[0][1]);
    pose.position.z = cast_xml_to_double(deck_pose[0][2]);

    tf2::Quaternion q;
    q.setRPY(cast_xml_to_double(deck_pose[1][0]), cast_xml_to_double(deck_pose[1][1]),
             cast_xml_to_double(deck_pose[1][2]));

    pose.orientation.x = q[0];
    pose.orientation.y = q[1];
    pose.orientation.z = q[2];
    pose.orientation.w = q[3];

    set_pose(pose);
  }

  // collect tools
  std::vector<std::string> tools;
  if (!pnh.getParam("tools", tools))
  {
    ROS_WARN_STREAM("no tools found: shutting...");
    ros::shutdown();
    return;
  }

  for (const std::string& tool_name : tools)
  {
    // get tool details from params
    bool is_active = false;
    pnh.param(tool_name + "/active", is_active, is_active);
    std::string frame = tool_frame_;
    pnh.param(tool_name + "/frame", frame, frame);

    // create new tool
    InteractiveTool new_tool(tool_name, is_active);

    // set the tool frame from param
    new_tool.set_frame(frame);

    // get poses from params
    XmlRpc::XmlRpcValue rest_pose;
    XmlRpc::XmlRpcValue active_pose;

    if (!pnh.getParam(tool_name + "/rest_pose", rest_pose))
    {
      ROS_WARN_STREAM("missing rest pose for " << tool_name << ": set to default pose");
    }
    else
    {
      geometry_msgs::Pose pose;

      pose.position.x = cast_xml_to_double(rest_pose[0][0]);
      pose.position.y = cast_xml_to_double(rest_pose[0][1]);
      pose.position.z = cast_xml_to_double(rest_pose[0][2]);

      tf2::Quaternion q;
      q.setRPY(cast_xml_to_double(rest_pose[1][0]), cast_xml_to_double(rest_pose[1][1]),
               cast_xml_to_double(rest_pose[1][2]));

      pose.orientation.x = q[0];
      pose.orientation.y = q[1];
      pose.orientation.z = q[2];
      pose.orientation.w = q[3];

      new_tool.set_rest_pose(pose);
    }

    if (!pnh.getParam(tool_name + "/active_pose", active_pose))
    {
      ROS_WARN_STREAM("missing active pose for " << tool_name << ": set to default pose");
    }
    else
    {
      geometry_msgs::Pose pose;

      pose.position.x = cast_xml_to_double(active_pose[0][0]);
      pose.position.y = cast_xml_to_double(active_pose[0][1]);
      pose.position.z = cast_xml_to_double(active_pose[0][2]);

      tf2::Quaternion q;
      q.setRPY(cast_xml_to_double(active_pose[1][0]), cast_xml_to_double(active_pose[1][1]),
               cast_xml_to_double(active_pose[1][2]));

      pose.orientation.x = q[0];
      pose.orientation.y = q[1];
      pose.orientation.z = q[2];
      pose.orientation.w = q[3];

      new_tool.set_active_pose(pose);
    }

    add_tool(tool_name, new_tool);
  }

  // setup tf timer callback
  tf_timer_ = pnh.createTimer(ros::Duration(1.0 / loop_hz_), &ToolDeckServer::tf_timer_callback, this);

  // setup tool services

  // setup marker server
  marker_server_.reset(new interactive_markers::InteractiveMarkerServer("tool_deck_controls", "", false));
  ros::Duration(0.1).sleep();

  // setup menu handler with menu elements
  menu_handler_.insert("save poses", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert("set frame", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert("activate", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert("deactivate", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert("move to nearest", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  interactive_markers::MenuHandler::EntryHandle feed_sub_menu_handle = menu_handler_.insert("drag feedrate");
  menu_handler_.insert(feed_sub_menu_handle, "50cm", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert(feed_sub_menu_handle, "10cm", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert(feed_sub_menu_handle, "1cm", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert(feed_sub_menu_handle, "1mm", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  interactive_markers::MenuHandler::EntryHandle rotate_sub_menu_handle = menu_handler_.insert("rotate feedrate");
  menu_handler_.insert(rotate_sub_menu_handle, "10deg", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert(rotate_sub_menu_handle, "1deg", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
  menu_handler_.insert(rotate_sub_menu_handle, "0.1deg", boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));

  // setup menu element callback functions
  int idx = 1;
  // save
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::menu_save, this, std::placeholders::_1) });
  // set frame
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::menu_set_frame, this, std::placeholders::_1) });
  // activate
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::menu_activate, this, std::placeholders::_1) });
  // deactivate
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::menu_deactivate, this, std::placeholders::_1) });
  // move to nearest
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::menu_nearest, this, std::placeholders::_1) });
  // feed 50cm
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::change_linear_feed<500>, this, std::placeholders::_1) });
  // feed 10cm
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::change_linear_feed<100>, this, std::placeholders::_1) });
  // feed 1cm
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::change_linear_feed<10>, this, std::placeholders::_1) });
  // feed 1mm
  menu_callbacks_.insert({ idx++, std::bind(&ToolDeckServer::change_linear_feed<1>, this, std::placeholders::_1) });
  // feed 10deg
  menu_callbacks_.insert(
      { idx++, std::bind(&ToolDeckServer::change_angular_feedrate<100>, this, std::placeholders::_1) });
  //  feed 1deg
  menu_callbacks_.insert(
      { idx++, std::bind(&ToolDeckServer::change_angular_feedrate<10>, this, std::placeholders::_1) });
  // feed 0.11deg
  menu_callbacks_.insert(
      { idx++, std::bind(&ToolDeckServer::change_angular_feedrate<1>, this, std::placeholders::_1) });

  // add tool controls
  for (auto& it : tool_list_)
  {
    // drag controls
    visualization_msgs::InteractiveMarker int_marker = it.second.get_marker();
    marker_server_->insert(int_marker);
    marker_server_->setCallback(int_marker.name, boost::bind(&ToolDeckServer::marker_feedback_cb, this, _1));

    // menu
    visualization_msgs::InteractiveMarker menu = it.second.get_menu();
    marker_server_->insert(menu);
    marker_server_->setCallback(menu.name, boost::bind(&ToolDeckServer::menu_feedback_cb, this, _1));
    menu_handler_.apply(*marker_server_, menu.name);
  }

  // apply changes
  marker_server_->applyChanges();

  // load state persistence
  load_persistent_tool_states();
}

ToolDeckServer::~ToolDeckServer()
{
  save_current_tool_states();
  marker_server_.reset();
}

double ToolDeckServer::cast_xml_to_double(XmlRpc::XmlRpcValue xml_value)
{
  double value = 0.0;

  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    value = static_cast<double>(xml_value);
  }
  else if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    value = (double)static_cast<int>(xml_value);
  }

  return value;
}

void ToolDeckServer::tf_timer_callback(const ros::TimerEvent& event)
{
  std::vector<geometry_msgs::TransformStamped> tf;

  // update deck frame transform
  geometry_msgs::TransformStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = base_frame_;
  msg.child_frame_id = deck_frame_;

  const geometry_msgs::Pose* deck_pose_ptr = get_pose();

  msg.transform.translation.x = deck_pose_ptr->position.x;
  msg.transform.translation.y = deck_pose_ptr->position.y;
  msg.transform.translation.z = deck_pose_ptr->position.z;
  msg.transform.rotation.x = deck_pose_ptr->orientation.x;
  msg.transform.rotation.y = deck_pose_ptr->orientation.y;
  msg.transform.rotation.z = deck_pose_ptr->orientation.z;
  msg.transform.rotation.w = deck_pose_ptr->orientation.w;

  tf.push_back(msg);

  // update transform for each tool
  for (auto& it : tool_list_)
  {
    geometry_msgs::TransformStamped msg;

    // current time and tool frame
    msg.header.stamp = ros::Time::now();
    msg.child_frame_id = it.second.get_name();

    // current frame of reference
    if (it.second.is_active())
    {
      msg.header.frame_id = it.second.get_frame();
    }
    else
    {
      msg.header.frame_id = deck_frame_;
    }

    // get current transfrom pose
    const geometry_msgs::Pose* tool_pose_ptr = it.second.get_current_pose();

    msg.transform.translation.x = tool_pose_ptr->position.x;
    msg.transform.translation.y = tool_pose_ptr->position.y;
    msg.transform.translation.z = tool_pose_ptr->position.z;
    msg.transform.rotation.x = tool_pose_ptr->orientation.x;
    msg.transform.rotation.y = tool_pose_ptr->orientation.y;
    msg.transform.rotation.z = tool_pose_ptr->orientation.z;
    msg.transform.rotation.w = tool_pose_ptr->orientation.w;

    // append current tool transform to tf
    tf.push_back(msg);
  }

  // publish tf
  tf_broadcaster_.sendTransform(tf);
}

void ToolDeckServer::marker_feedback_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    // find a suitable timestep
    double dt = (ros::Time::now() - feedback->header.stamp).toSec();

    // collect euler orientation description
    double r, p, y;
    tf2::getEulerYPR(feedback->pose.orientation, y, p, r);

    // get current tool by name and associated feedrates
    InteractiveTool* current_tool_ptr = get_tool_ptr(feedback->marker_name);
    const double lin_feed = current_tool_ptr->get_linear_feedrate();
    const double ang_feed = current_tool_ptr->get_angular_feedrate();

    // filter by control type
    double independant_dimension_pose_control[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // make all controls zero
                                                                                      // unless otherwise

    if (feedback->control_name == "move_x")
    {
      independant_dimension_pose_control[0] =
          InteractiveTool::filter_linear_control_to_velocity(lin_feed, feedback->pose.position.x, dt);
    }
    if (feedback->control_name == "move_y")
    {
      independant_dimension_pose_control[1] =
          InteractiveTool::filter_linear_control_to_velocity(lin_feed, feedback->pose.position.y, dt);
    }
    if (feedback->control_name == "move_z")
    {
      independant_dimension_pose_control[2] =
          InteractiveTool::filter_linear_control_to_velocity(lin_feed, feedback->pose.position.z, dt);
    }
    if (feedback->control_name == "rotate_x")
    {
      independant_dimension_pose_control[3] = InteractiveTool::filter_angular_control_to_velocity(ang_feed, r, dt);
    }
    if (feedback->control_name == "rotate_y")
    {
      independant_dimension_pose_control[4] = InteractiveTool::filter_angular_control_to_velocity(ang_feed, p, dt);
    }
    if (feedback->control_name == "rotate_z")
    {
      independant_dimension_pose_control[5] = InteractiveTool::filter_angular_control_to_velocity(ang_feed, y, dt);
    }

    // this is the delta-pose that we want to work with via velocity control (mulitply by dt)
    geometry_msgs::Pose dPose;
    dPose.position.x = independant_dimension_pose_control[0];
    dPose.position.y = independant_dimension_pose_control[1];
    dPose.position.z = independant_dimension_pose_control[2];
    tf2::Quaternion dq;
    dq.setRPY(independant_dimension_pose_control[3], independant_dimension_pose_control[4],
              independant_dimension_pose_control[5]);
    dPose.orientation.x = dq[0];
    dPose.orientation.y = dq[1];
    dPose.orientation.z = dq[2];
    dPose.orientation.w = dq[3];

    // this is the frame dPose will transform into
    geometry_msgs::TransformStamped tool_frame_transform;

    // current time and tool frame
    tool_frame_transform.header.stamp = ros::Time::now();
    tool_frame_transform.child_frame_id = current_tool_ptr->get_name();

    // current frame of reference
    if (current_tool_ptr->is_active())
    {
      tool_frame_transform.header.frame_id = tool_frame_;
    }
    else
    {
      tool_frame_transform.header.frame_id = deck_frame_;
    }

    // set current transfrom pose
    const geometry_msgs::Pose* tool_pose_ptr = current_tool_ptr->get_current_pose();
    tool_frame_transform.transform.translation.x = tool_pose_ptr->position.x;
    tool_frame_transform.transform.translation.y = tool_pose_ptr->position.y;
    tool_frame_transform.transform.translation.z = tool_pose_ptr->position.z;
    tool_frame_transform.transform.rotation.x = tool_pose_ptr->orientation.x;
    tool_frame_transform.transform.rotation.y = tool_pose_ptr->orientation.y;
    tool_frame_transform.transform.rotation.z = tool_pose_ptr->orientation.z;
    tool_frame_transform.transform.rotation.w = tool_pose_ptr->orientation.w;

    // the new pose inside the tool reference frame
    geometry_msgs::Pose new_pose;
    tf2::doTransform(dPose, new_pose, tool_frame_transform);

    // normalise the quaternion vector
    tf2::Quaternion q;
    tf2::convert(new_pose.orientation, q);
    q.normalize();
    new_pose.orientation.x = q[0];
    new_pose.orientation.y = q[1];
    new_pose.orientation.z = q[2];
    new_pose.orientation.w = q[3];

    // apply new pose to current tool pose
    set_current_tool_pose(feedback->marker_name, new_pose);

    // # reset marker pose
    marker_server_->setPose(feedback->marker_name, geometry_msgs::Pose());
  }

  marker_server_->applyChanges();
}

void ToolDeckServer::menu_feedback_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // ROS_DEBUG_STREAM("feedback event: " << (int)feedback->event_type);

  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    // do selected callback
    auto menu_cb_iter = menu_callbacks_.find(feedback->menu_entry_id);
    if (!(menu_cb_iter == menu_callbacks_.end()))
    {
      menu_cb_iter->second(feedback);
    }
  }

  marker_server_->applyChanges();
}

void ToolDeckServer::menu_save(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  save_current_tool_states();
}

void ToolDeckServer::menu_set_frame(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
}

void ToolDeckServer::menu_activate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // activate the tool named in feedback
  activate(feedback->header.frame_id);
}

void ToolDeckServer::menu_deactivate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // deactivate the tool named in feedback
  deactivate(feedback->header.frame_id);
}

void ToolDeckServer::menu_nearest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // get current pose
  InteractiveTool* tool_ptr = get_tool_ptr(feedback->header.frame_id);
  const geometry_msgs::Pose* pose_ptr = tool_ptr->get_current_pose();

  // round to nearest whole values
  geometry_msgs::Pose round_pose;
  double r, p, y;
  tf2::Quaternion q;

  // nearest millimetre
  round_pose.position.x = std::round(pose_ptr->position.x * 1000.0) / 1000.0;
  round_pose.position.y = std::round(pose_ptr->position.y * 1000.0) / 1000.0;
  round_pose.position.z = std::round(pose_ptr->position.z * 1000.0) / 1000.0;

  // nearest degree
  // tf2::getEulerYPR(pose_ptr->orientation, y, p, r);
  // r = std::round(r * (180.0 / M_PI));
  // p = std::round(p * (180.0 / M_PI));
  // y = std::round(y * (180.0 / M_PI));
  // q.setRPY(r, p, y);
  // q.normalize();
  // round_pose.orientation.x = q[0];
  // round_pose.orientation.y = q[1];
  // round_pose.orientation.z = q[2];
  // round_pose.orientation.w = q[3];

  round_pose.orientation = pose_ptr->orientation;

  // set pose to rounded value
  set_current_tool_pose(feedback->header.frame_id, round_pose);
}

void ToolDeckServer::save_current_tool_states()
{
  // create yaml node
  YAML::Node state_persistence;

  for (auto& it : tool_list_)
  {
    // format each member data of tool in yaml
    YAML::Node tool_node;

    // save the tool frame
    tool_node["frame"] = it.second.get_frame();

    // is active or not
    tool_node["active"] = it.second.is_active();

    if (it.second.is_active())
    {
      // the tool active pose yaml
      YAML::Node pose_node;
      const geometry_msgs::Pose pose = it.second.get_active_pose();
      pose_node["position"].push_back(pose.position.x);
      pose_node["position"].push_back(pose.position.y);
      pose_node["position"].push_back(pose.position.z);
      pose_node["orientation"].push_back(pose.orientation.x);
      pose_node["orientation"].push_back(pose.orientation.y);
      pose_node["orientation"].push_back(pose.orientation.z);
      pose_node["orientation"].push_back(pose.orientation.w);

      // add to tool level yaml
      tool_node["active_pose"] = pose_node;
    }

    // add to top level yaml
    state_persistence[it.second.get_name()] = tool_node;
  }

  // write to disk at specified location
  YAML::Emitter persistence_emitter;
  persistence_emitter << state_persistence;

  // file location - {persistence_dir}/<node_name>_tool_states.yaml
  // std::string f_name_rel = persistence_dir_ + ros::this_node::getName() + "_tool_states.yaml";
  // boost::filesystem::path f_name_path = persistence_dir_;
  // ROS_WARN_STREAM(f_name_path);

  // f_name_path = boost::filesystem::system_complete(f_name_path);
  // ROS_WARN_STREAM(f_name_path);

  // ROS_WARN_STREAM(f_name);

  // f_name = boost::filesystem::absolute(f_name_path.c_str() + f_name).c_str();  // defaults to ~/.ros/<file>
  // ROS_WARN_STREAM(f_name);

  std::string f_name = ros::this_node::getName() + "_tool_states.yaml";
  f_name = boost::filesystem::current_path().c_str() + f_name;

  std::ofstream persistence_file(f_name);
  if (persistence_file.is_open())
  {
    ROS_INFO_STREAM("saving tool state file: " << f_name);
    persistence_file << persistence_emitter.c_str();
    persistence_file.close();
  }
  else
  {
    ROS_ERROR_STREAM("unable to open persistence file: " << f_name);
  }
}

void ToolDeckServer::load_persistent_tool_states()
{
  std::string f_name = ros::this_node::getName() + "_tool_states.yaml";
  f_name = boost::filesystem::current_path().c_str() + f_name;

  if (boost::filesystem::exists(f_name))
  {
    ROS_INFO_STREAM("loading persistent tool states: " << f_name);

    YAML::Node state_persistence = YAML::LoadFile(f_name);

    // iterate through loaded tools
    for (auto& it : tool_list_)
    {
      if (state_persistence[it.second.get_name()])
      {
        // was the tool active
        if (state_persistence[it.second.get_name()]["active"].as<bool>())
        {
          // get frame
          it.second.set_frame(state_persistence[it.second.get_name()]["frame"].as<std::string>());

          // get stored active pose
          geometry_msgs::Pose new_active_pose;
          new_active_pose.position.x =
              state_persistence[it.second.get_name()]["active_pose"]["position"][0].as<double>();
          new_active_pose.position.y =
              state_persistence[it.second.get_name()]["active_pose"]["position"][1].as<double>();
          new_active_pose.position.z =
              state_persistence[it.second.get_name()]["active_pose"]["position"][2].as<double>();
          new_active_pose.orientation.x =
              state_persistence[it.second.get_name()]["active_pose"]["orientation"][0].as<double>();
          new_active_pose.orientation.y =
              state_persistence[it.second.get_name()]["active_pose"]["orientation"][1].as<double>();
          new_active_pose.orientation.z =
              state_persistence[it.second.get_name()]["active_pose"]["orientation"][2].as<double>();
          new_active_pose.orientation.w =
              state_persistence[it.second.get_name()]["active_pose"]["orientation"][3].as<double>();

          // set tool active pose
          it.second.set_active_pose(new_active_pose);
          it.second.activate();
        }
      }
    }
  }
}

}  // namespace tool_deck_server
