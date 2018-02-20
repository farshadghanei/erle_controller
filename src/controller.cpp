#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "controller.h"

//#include <cstdlib>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/CommandTOL.h>
//#include <mavros_msgs/SetMode.h>

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
    n.param<std::string>("worldFrame", worldFrame, "/world");
    n.param<std::string>("frame", frame, "/frame");

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

    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
/*    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    ROS_INFO("is valid %d", cl.isValid());
    if(cl.call(srv_setMode)){
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.mode_sent);
    }else{
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.mode_sent);
//        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);

        ROS_ERROR("Failed SetMode");
        return -1;
    }

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 10;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
    sleep(10);

    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 10;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }
*/
/*
    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }
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
