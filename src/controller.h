#ifndef _CONTROLLER_H_
#define _CONTROLLER_H

#include "pid.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_setstreamrate.h"


/* A function to get parameter from node */
double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

namespace drone {

tf::Transform goalPose;

typedef struct DroneCommand {
    double roll;
    double pitch;
    double yaw;
    double thrust;
} DroneCommand;

void initializeGoal();
void setGoal(tf::Transform);
tf::Transform getGoal();
tf::Transform getPose();

void sendCommand(DroneCommand);

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
        , m_pubNav()
        , m_listener()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_serviceHeight()
        , m_thrust(0)
        , m_startZ(0)
    {
        ROS_INFO("controller object created");
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
	ROS_INFO("transform_listener result: %s",m_listener.allFramesAsString().c_str());
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_pubRC = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override",10);
        ROS_INFO("cmd_vel topic advertised");
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        ROS_INFO("goal topic subscribed");
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
        ROS_INFO("takeoff and land services advertised");
        //FARSHAD
        m_serviceHeight = nh.advertiseService("set_goal_height", &Controller::set_goal_height, this);
        }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    /* abstraction for publishing the output */
    void publish_output(geometry_msgs::Twist msg) {
        if (msg.linear.x + msg.linear.y + msg.linear.z + msg.angular.z > 0.0001) {
		ROS_INFO("Controller values (x,y,z,Yaw): (%f,%f,%f,%f)",msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
	}
        m_pubNav.publish(msg);
        mavros_msgs::OverrideRCIn rc_override;
        rc_override.channels[0] = msg.linear.x;
        rc_override.channels[1] = msg.linear.y;
        rc_override.channels[2] = msg.linear.z;
        rc_override.channels[3] = msg.angular.z;
        rc_override.channels[4] = 1100;
        rc_override.channels[5] = 1100;
        rc_override.channels[6] = 1100;
        rc_override.channels[7] = 1100;
        m_pubRC.publish(rc_override);
    }

    /* setting hover/goal position  manually */
    bool set_goal_height(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Request& res)
    {
        ROS_INFO("set_goal_height requested. setting it to 1.0");
        m_goal.pose.position.z = 1.0;
        return true;
    }
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    /* takeoff service */
    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
	//FARSHAD
	getTransform(m_worldFrame, m_frame, transform);
        //m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();
        ROS_INFO("the original height is: %f", m_startZ);

        return true;
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

    /* obtaining the transform */
    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
	ROS_INFO("getTransform: from \"%s\" to \"%s\" frame.",	sourceFrame.c_str(), targetFrame.c_str());	
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
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
//        ROS_INFO("entered iteration at time: %f", e.current_real.toSec());
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
		//FARSHAD
		getTransform(m_worldFrame, m_frame, transform);
                //m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                    ROS_INFO("Shifting to automatic mode");
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    publish_output(msg);
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
		//FARSHAD
		getTransform(m_worldFrame, m_frame, transform);
                //m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    publish_output(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
		//FARSHAD
		getTransform(m_worldFrame, m_frame, transform);
                //m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;
		ROS_INFO("targetWorld");
		ROS_INFO("%f, %f, %f",targetWorld.pose.position.x, targetWorld.pose.position.y, targetWorld.pose.position.z);
		ROS_INFO("%f, %f, %f, %f",targetWorld.pose.orientation.x, targetWorld.pose.orientation.y, 
				targetWorld.pose.orientation.z,targetWorld.pose.orientation.w);
                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);
		ROS_INFO("reached label_2");

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

		ROS_INFO("reached label_3");
                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0.0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                publish_output(msg);
            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                publish_output(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    ros::Publisher m_pubRC;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    ros::ServiceServer m_serviceHeight;
    float m_thrust;
    float m_startZ;
};

}

#endif /* _CONTROLLER_H */
