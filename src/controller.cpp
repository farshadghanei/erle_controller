#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "controller.h"

//ROS node name
std::string NODE_NAME="controller_node";

int main(int argc, char **argv)
{
    ROS_INFO("ErleController has started!");

    ros::init(argc, argv, "erle_controller");
    ros::NodeHandle n;
    double frequency;
    std::string worldFrame;
    std::string frame;

    ROS_INFO("loading initial parameters (frequency and frames)");
    n.getParam(NODE_NAME+"/frequency", frequency);
    n.getParam(NODE_NAME+"/worldFrame", worldFrame);
    n.getParam(NODE_NAME+"/frame", frame);

    ROS_INFO("Setting MAVROS stream rate...");
    MAVROS_setStreamRate setStreamRate;

    ROS_INFO("Running the controller at frequency: %f", frequency);
    drone::Controller controller(worldFrame, frame, n);
    controller.run(frequency);

    return 0;
}

