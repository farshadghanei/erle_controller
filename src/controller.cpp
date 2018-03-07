#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "controller.h"

int main(int argc, char **argv)
{
    ROS_INFO("ErleController has started!");
    int rate = 10;

    ros::init(argc, argv, "erle_controller");
    ros::NodeHandle n;
    double frequency;
    std::string worldFrame;
    std::string frame;

    ROS_INFO("loading initial parameters (frequency and frames)");

    n.param<double>("frequency", frequency, 50.0);
    n.getParam("controller_node/worldFrame", worldFrame);
    n.getParam("controller_node/frame", frame);

    drone::Controller controller(worldFrame, frame, n);
    ROS_INFO("Running the controller at frequency: %f", frequency);

    ROS_INFO("Setting MAVROS stream rate...");
    MAVROS_setStreamRate setStreamRate;

    controller.run(frequency);

/*    drone::initializeGoal();

    ros::Rate r(rate);
    tf::Transform currentPose, targetPose;
    currentPose = drone::getPose();
    targetPose = drone::getGoal();
*/

    return 0;
}


namespace drone {

void initializeGoal() {
    goalPose.setOrigin(tf::Vector3(0.0,0.0,4.0));
}

void setGoal(tf::Transform target) {
    goalPose.setOrigin(target.getOrigin());
}
    
tf::Transform getGoal() {
    return goalPose;
}

/* Reading the pose from Vicon */
tf::Transform getPose() {
    tf::Transform pose;
    pose.setOrigin(tf::Vector3(0.0,0.0,0.0));
    return pose;
}

void sendCommand(DroneCommand cmd) {
    ROS_INFO("Command being sent to drone (R,P,Y,Th): (%f,%f,%f,%f)"
                ,cmd.roll, cmd.pitch, cmd.yaw, cmd.thrust);
    return;
}



}
