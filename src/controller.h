#ifndef _CONTROLLER_H_
#define _CONTROLLER_H

#include "pid.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_setstreamrate.h"

//ROS node name
extern std::string NODE_NAME;

namespace drone {


/* A function to get parameter from node */
double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

/***** Controller Class ******/
class Controller
{
public:
    /* Constructor with initialization */
    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_listener()
        , m_pidX(
            get(n, NODE_NAME+"/PIDs/X/kp"),
            get(n, NODE_NAME+"/PIDs/X/kd"),
            get(n, NODE_NAME+"/PIDs/X/ki"),
            get(n, NODE_NAME+"/PIDs/X/minOutput"),
            get(n, NODE_NAME+"/PIDs/X/maxOutput"),
            get(n, NODE_NAME+"/PIDs/X/integratorMin"),
            get(n, NODE_NAME+"/PIDs/X/integratorMax"),
            get(n, NODE_NAME+"/PIDs/X/integratorSpeedThreshold"),
            "x")
        , m_pidY(
            get(n, NODE_NAME+"/PIDs/Y/kp"),
            get(n, NODE_NAME+"/PIDs/Y/kd"),
            get(n, NODE_NAME+"/PIDs/Y/ki"),
            get(n, NODE_NAME+"/PIDs/Y/minOutput"),
            get(n, NODE_NAME+"/PIDs/Y/maxOutput"),
            get(n, NODE_NAME+"/PIDs/Y/integratorMin"),
            get(n, NODE_NAME+"/PIDs/Y/integratorMax"),
            get(n, NODE_NAME+"/PIDs/Y/integratorSpeedThreshold"),
            "y")
        , m_pidZ(
            get(n, NODE_NAME+"/PIDs/Z/kp"),
            get(n, NODE_NAME+"/PIDs/Z/kd"),
            get(n, NODE_NAME+"/PIDs/Z/ki"),
            get(n, NODE_NAME+"/PIDs/Z/minOutput"),
            get(n, NODE_NAME+"/PIDs/Z/maxOutput"),
            get(n, NODE_NAME+"/PIDs/Z/integratorMin"),
            get(n, NODE_NAME+"/PIDs/Z/integratorMax"),
            get(n, NODE_NAME+"/PIDs/Z/integratorSpeedThreshold"),
            "z")
        , m_pidYaw(
            get(n, NODE_NAME+"/PIDs/Yaw/kp"),
            get(n, NODE_NAME+"/PIDs/Yaw/kd"),
            get(n, NODE_NAME+"/PIDs/Yaw/ki"),
            get(n, NODE_NAME+"/PIDs/Yaw/minOutput"),
            get(n, NODE_NAME+"/PIDs/Yaw/maxOutput"),
            get(n, NODE_NAME+"/PIDs/Yaw/integratorMin"),
            get(n, NODE_NAME+"/PIDs/Yaw/integratorMax"),
            get(n, NODE_NAME+"/PIDs/Yaw/integratorSpeedThreshold"),
            "yaw")
        , m_RC_roll_min(get(n, NODE_NAME+"/RC/roll/min"))
        , m_RC_roll_mid(get(n, NODE_NAME+"/RC/roll/mid"))
        , m_RC_roll_max(get(n, NODE_NAME+"/RC/roll/max"))
        , m_RC_pitch_min(get(n, NODE_NAME+"/RC/pitch/min"))
        , m_RC_pitch_mid(get(n, NODE_NAME+"/RC/pitch/mid"))
        , m_RC_pitch_max(get(n, NODE_NAME+"/RC/pitch/max"))
        , m_RC_yaw_min(get(n, NODE_NAME+"/RC/yaw/min"))
        , m_RC_yaw_mid(get(n, NODE_NAME+"/RC/yaw/mid"))
        , m_RC_yaw_max(get(n, NODE_NAME+"/RC/yaw/max"))
        , m_RC_thrust_min(get(n, NODE_NAME+"/RC/thrust/min"))
        , m_RC_thrust_mid(get(n, NODE_NAME+"/RC/thrust/mid"))
        , m_RC_thrust_max(get(n, NODE_NAME+"/RC/thrust/max"))
        , m_RC_thrust_take_off_step(get(n, NODE_NAME+"/RC/thrust/take_off_step"))
        , m_state(Idle)
        , m_pose_worldFrame()
        , m_goal_worldFrame()
        , m_goal_bodyFrame()
        , m_transform()
        , m_subscribeGoal()
        , m_serviceArm()
        , m_serviceDisarm()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_serviceStop()
        , m_thrust(0)
        , m_startZ(0)
    {
        ROS_INFO("controller object created");
        print_RC_params();
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        ROS_INFO("transform_listener result: %s",m_listener.allFramesAsString().c_str());
        m_pubRC = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override",10);
        m_subscribeGoal = nh.subscribe("erle/goal", 1, &Controller::goalChanged, this);
        m_serviceTakeoff = nh.advertiseService("erle/takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("erle/land", &Controller::land, this);
        m_serviceArm = nh.advertiseService("erle/arm", &Controller::arm, this);
        m_serviceDisarm = nh.advertiseService("erle/disarm", &Controller::disarm, this);
        m_serviceStop = nh.advertiseService("erle/stop", &Controller::stop, this);
        ROS_INFO("Services advertised: erle/arm, disarm, takeoff, land, stop");

        m_rc_override.channels[0] = m_RC_pitch_mid;
        m_rc_override.channels[2] = m_RC_thrust_min;

        rc_out();   
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        while(ros::ok()) {
        ros::spinOnce();
        }
    }

private:
    /* abstraction for publishing the output */
    void publish_output(geometry_msgs::Twist msg) {
        //if (msg.linear.x + msg.linear.y + msg.linear.z + msg.angular.z > 0.0001) {
    //  ROS_INFO("Controller values (x,y,z,Yaw): (%f,%f,%f,%f)",msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
    //}
        //rc_override.channels[0] = m_RC_midRoll + msg.linear.x;
        //rc_override.channels[1] = m_RC_midPitch + msg.linear.y;
        m_rc_override.channels[2] = m_RC_thrust_min + msg.linear.z;
        //rc_override.channels[3] = m_RC_midYaw + msg.angular.z;
        rc_out();
    }

    /* setting hover/goal position  manually */
    bool set_goal_height(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Request& res) {

        m_goal_worldFrame.pose.position.z = 1;
        m_goal_worldFrame.pose.orientation.w = 1.0;
        ROS_INFO("set_goal_height requested. setting it to %f", m_goal_worldFrame.pose.position.z);
        return true;
    }
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_INFO("New goal has been published in topic:\n\t\
            Poition       (x, y, z): (%.2f, %.2f, %.2f)\n\t\
            Orientation   (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            msg->pose.position.x, 
            msg->pose.position.y, 
            msg->pose.position.z, 
            msg->pose.orientation.x, 
            msg->pose.orientation.y, 
            msg->pose.orientation.z, 
            msg->pose.orientation.w);
        m_goal_worldFrame = *msg;
    }

    /* arm ErleCopter */
    bool arm(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res) {

        bool response;
        if (m_state==Idle) {
            ROS_INFO("Arm requested...");
            mavros_msgs::CommandBool armValue;
            armValue.request.value = true;
            if (ros::service::call("/mavros/cmd/arming", armValue)) {
                ROS_INFO("send armValue successful.");
                response = true;
                m_state = Armed;
            } else {
                ROS_INFO("send armValue failed");
                response = false;
            }
        } else {
            ROS_INFO("state is not idle, cannot call arm.");
            response = false;
        }
        return response;
    }

    /* disarm ErleCopter */
    bool disarm(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res) {

        bool response;
        if (m_state == Armed) {
            ROS_INFO("Disarm requested...");
            mavros_msgs::CommandBool armValue;
            armValue.request.value = false;
            if (ros::service::call("/mavros/cmd/arming", armValue)) {
                ROS_INFO("send disarmValue successful.");
                response = true;
                m_state = Idle;
            } else {
                ROS_INFO("send disarmValue failed");
                response = false;
            }
        } else {
            ROS_INFO("state is not armed, cannot call disarm.");
            response = false;
        }
        return response;
    }

    /* stop and disarm ErleCopter giving control to RC*/
    bool stop(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res) {

        bool response = true;

        ROS_INFO("Stop requested...");

        ROS_INFO("rollback RC channels!");
        // all sticks in the middle, thrust to minimum
        m_rc_override.channels[0]=m_RC_roll_mid;
        m_rc_override.channels[1]=m_RC_pitch_mid;
        m_rc_override.channels[2]=m_RC_thrust_min;
        m_rc_override.channels[3]=m_RC_yaw_mid;
        rc_out();

        release_channels(0xFF);
        rc_out();
        //response = disarm();

        return response;
    }

    /* publishing the RC values to override topic */    
    void rc_out(void) {
        m_pubRC.publish(m_rc_override);
    }

    /* release override of channels back to the RC, 
     use flags to indicate which channels, rest will be untouched */
    void release_channels(int flags) {
        for (int i=0;i<8;i++) {
            if (flags & 1) {
                 m_rc_override.channels[i] = 0;
            }
            flags >> 1;
        }
        rc_out();
    }

    /* takeoff service */
    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res) {

        bool response;
        if (m_state == Armed) {
            ROS_INFO("Takeoff requested...");
            
            m_goal_worldFrame.pose.position.z = 2.0;
            m_goal_worldFrame.pose.orientation.w = 1.0;
            m_startZ = m_transform.getOrigin().z();
            ROS_INFO("the original height is: %f", m_startZ);
            m_state = TakingOff;
            response = true;
        } else {
            ROS_INFO("state is not armed, cannot call takeoff.");
            response = false;
        }
        return response;
    }

    /* landing service */
    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    /* updating the local transform values */
    void updateTransform(void) {
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), m_transform);
        m_pose_worldFrame.pose.position = m_transform.getOrigin();
        m_pose_worldFrame.pose.orientation = m_transform.getRotation();

        m_goal_worldFrame.header.stamp = m_transform.stamp_;
        m_goal_worldFrame.header.frame_id = m_worldFrame;
        m_listener.transformPose(m_frame, m_goal_worldFrame, m_goal_bodyFrame);
        //ROS_INFO("updated transform:");  
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    /* repeating each dt */
    void iteration(const ros::TimerEvent& e)
    {
    if (!ros::ok()) {
        release_channels(0xFF);
        ros::shutdown();
    }
//        ROS_INFO("entered iteration at time: %f", e.current_real.toSec());
        float dt = e.current_real.toSec() - e.last_real.toSec();

        updateTransform();

        switch(m_state) {
            case Armed:
            {
                geometry_msgs::Twist msg;
                msg.linear.z = 100;     //just a little thrust to prevent disarming
                publish_output(msg);
            }
            break;
            case TakingOff: 
            {
                if (m_pose_worldFrame.pose.position.z > m_startZ + 0.15 || m_thrust > m_RC_thrust_max - m_RC_thrust_min) {
                    pidReset();
                    // ROS_INFO("Value of m_pidZ.ki = %f",m_pidZ.ki());
                    m_RC_thrust_min = 1400; // += m_thrust;
                    m_pidZ.setIntegral(0);//m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    ROS_INFO("Shifting to automatic mode, m_thrust=%f, current_z=%f", m_thrust, m_transform.getOrigin().z());
                    m_thrust = 0;
                } else {
                    m_thrust += m_RC_thrust_take_off_step * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    publish_output(msg);
                }

            }
            break;
            case Landing:
            {
                m_goal_worldFrame.pose.position.z -= 0.20 * dt; //m_startZ + 0.1;
                if (m_pose_worldFrame.pose.position.z <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    msg.linear.z = 1100 - m_RC_thrust_min;
                    publish_output(msg);
                    ROS_INFO("I am done landing, going to idle");
                }
            }
            // intentional fall-thru
            case Automatic:
            {
                if (iterationCounter==0) {
                    ROS_INFO("Relative target: %f", m_goal_bodyFrame.pose.position.z);
                    //ROS_INFO("targetDrone");
                    //ROS_INFO("%f, %f, %f",targetDrone.pose.position.x, targetDrone.pose.position.y, targetDrone.pose.position.z);
                    //ROS_INFO("%f, %f, %f, %f",targetDrone.pose.orientation.x, targetDrone.pose.orientation.y, 
                    //  targetDrone.pose.orientation.z,targetDrone.pose.orientation.w);
                }
                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        m_goal_bodyFrame.pose.orientation.x,
                        m_goal_bodyFrame.pose.orientation.y,
                        m_goal_bodyFrame.pose.orientation.z,
                        m_goal_bodyFrame.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);
        
                geometry_msgs::Twist msg;
                //msg.linear.x = m_pidX.update(0.0, targetDrone.pose.position.x);
                //msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, m_goal_bodyFrame.pose.position.z);
                //msg.angular.z = m_pidYaw.update(0.0, yaw);
                if (iterationCounter==0) {
                    //ROS_INFO("Target Roll, Pitch, Yaw: (%f,%f,%f)", roll, pitch, yaw);
                    //ROS_INFO("m_pidX, m_pidY, m_pidZ, m_pidYaw: (%f,%f,%f, %f)", 
                    //  msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
                    ROS_INFO("m_pidZ=%f",msg.linear.z);
                }
                publish_output(msg);
            }
            break;
            case Idle:
            {
            }
            break;
        }
        if (iterationCounter++ > 100) {
            //ROS_INFO("Controller values (x,y,z,Yaw): (%d,%d,%d,%d)", rc_override.channels[0], rc_override.channels[1], rc_override.channels[2], rc_override.channels[3]);
            ROS_INFO("RC_override: %d",m_rc_override.channels[2]);
            iterationCounter=0;
        }
        rc_out();
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
        Armed = 4,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubRC;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_pose_worldFrame;
    geometry_msgs::PoseStamped m_goal_worldFrame;
    geometry_msgs::PoseStamped m_goal_bodyFrame;
    tf::StampedTransform m_transform;
    mavros_msgs::OverrideRCIn m_rc_override;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    ros::ServiceServer m_serviceArm;
    ros::ServiceServer m_serviceDisarm;
    ros::ServiceServer m_serviceStop;
    float m_thrust;
    float m_startZ;
    int iterationCounter;
    float m_RC_roll_min;
    float m_RC_roll_mid;
    float m_RC_roll_max;
    float m_RC_pitch_min;
    float m_RC_pitch_mid;
    float m_RC_pitch_max;
    float m_RC_yaw_min;
    float m_RC_yaw_mid;
    float m_RC_yaw_max;
    float m_RC_thrust_min;
    float m_RC_thrust_mid;
    float m_RC_thrust_max;
    float m_RC_thrust_take_off_step;

/* just printing all parameters */
    void print_RC_params(void) {
        ROS_INFO("Loaded RC parameters:\n\
\tm_RC_roll_min=%f\n\
\tm_RC_roll_mid=%f\n\
\tm_RC_roll_max=%f\n\
\tm_RC_pitch_min=%f\n\
\tm_RC_pitch_mid=%f\n\
\tm_RC_pitch_max=%f\n\
\tm_RC_yaw_min=%f\n\
\tm_RC_yaw_mid=%f\n\
\tm_RC_yaw_max=%f\n\
\tm_RC_thrust_min=%f\n\
\tm_RC_thrust_mid=%f\n\
\tm_RC_thrust_max=%f\n\
\tm_RC_thrust_take_off_step=%f\n\
", 
     m_RC_roll_min,
     m_RC_roll_mid,
     m_RC_roll_max,
     m_RC_pitch_min,
     m_RC_pitch_mid,
     m_RC_pitch_max,
     m_RC_yaw_min,
     m_RC_yaw_mid,
     m_RC_yaw_max,
     m_RC_thrust_min,
     m_RC_thrust_mid,
     m_RC_thrust_max,
     m_RC_thrust_take_off_step);
    }

};

}

#endif /* _CONTROLLER_H */
