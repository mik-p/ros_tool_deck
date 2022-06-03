
#include <ros/ros.h>
#include <tool_deck_server/tool_deck.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tool_deck_server");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tool_deck_server::ToolDeckServer tdserv(nh, pnh);

  ros::spin();

  return 0;
}
