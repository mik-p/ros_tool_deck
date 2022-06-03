# ros_tool_deck
An implementation of a 'tool changer' type system that publishes transforms for a list of tools in ROS. This addresses limitations in dynamically changing a URDF for the purposes of swapping used tools during runtime

## use case
1. A robotic manipulator may be performing a machining operation needing two different drill bits. The tools can be activated/de-activated and moved into the appropriate URDF/TF frame as needed.

1. A new sensor might be under testing and to avoid redesigning the robot URDF, it could be added dynamically later.

1. A user may need to use different conflicting features on a robot requiring the activation and deactivation of capabilities (used with ROS capabilities interface)

1. Hierarchically connecting 2 robots together? For example creating a landing pad for an aerial drone on a robotic ground vehicle (this would probably be a bad idea).
