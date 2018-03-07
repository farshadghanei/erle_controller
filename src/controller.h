#ifndef _CONTROLLER_H_
#define _CONTROLLER_H

#include "pid.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_setstreamrate.h"


namespace drone {


/* A function to get parameter from node */
double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

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
            get(n, "/controller_node/PIDs/X/kp"),
            get(n, "/controller_node/PIDs/X/kd"),
            get(n, "/controller_node/PIDs/X/ki"),
            get(n, "/controller_node/PIDs/X/minOutput"),
            get(n, "/controller_node/PIDs/X/maxOutput"),
            get(n, "/controller_node/PIDs/X/integratorMin"),
            get(n, "/controller_node/PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "/controller_node/PIDs/Y/kp"),
            get(n, "/controller_node/PIDs/Y/kd"),
            get(n, "/controller_node/PIDs/Y/ki"),
            get(n, "/controller_node/PIDs/Y/minOutput"),
            get(n, "/controller_node/PIDs/Y/maxOutput"),
            get(n, "/controller_node/PIDs/Y/integratorMin"),
            get(n, "/controller_node/PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "/controller_node/PIDs/Z/kp"),
            get(n, "/controller_node/PIDs/Z/kd"),
            get(n, "/controller_node/PIDs/Z/ki"),
            get(n, "/controller_node/PIDs/Z/minOutput"),
            get(n, "/controller_node/PIDs/Z/maxOutput"),
            get(n, "/controller_node/PIDs/Z/integratorMin"),
            get(n, "/controller_node/PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "/controller_node/PIDs/Yaw/kp"),
            get(n, "/controller_node/PIDs/Yaw/kd"),
            get(n, "/controller_node/PIDs/Yaw/ki"),
            get(n, "/controller_node/PIDs/Yaw/minOutput"),
            get(n, "/controller_node/PIDs/Yaw/maxOutput"),
            get(n, "/controller_node/PIDs/Yaw/integratorMin"),
            get(n, "/controller_node/PIDs/Yaw/integratorMax"),
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
	m_RC_midOutput = get(n, "/controller_node/Params/RC_midOutput");
	m_RC_minOutput = get(n, "/controller_node/Params/RC_minOutput");
	m_RC_maxOutput = get(n, "/controller_node/Params/RC_maxOutput");
	m_RC_minThrust = get(n, "/controller_node/Params/RC_minThrust");
	m_RC_midRoll = get(n, "/controller_node/Params/RC_midRoll");
	m_RC_midPitch = get(n, "/controller_node/Params/RC_midPitch");
	m_RC_midYaw = get(n, "/controller_node/Params/RC_midYaw");
	m_RC_step = get(n, "/controller_node/Params/RC_step");
	ROS_INFO("RC (min, mid, max, thrust) = (%f, %f, %f, %f)", m_RC_minOutput, m_RC_midOutput, m_RC_maxOutput, m_RC_minThrust);
	ROS_INFO("RC (roll, pitch, yaw) = (%f, %f, %f)", m_RC_midRoll, m_RC_midPitch, m_RC_midYaw);
        //rc_override.channels[0] = m_RC_midRoll;
        //rc_override.channels[1] = m_RC_midPitch;
        rc_override.channels[2] = m_RC_minThrust;
        //rc_override.channels[3] = m_RC_midYaw;
        //rc_override.channels[4] = m_RC_minOutput;
        //rc_override.channels[5] = m_RC_minOutput;
        //rc_override.channels[6] = m_RC_midOutput;
        //rc_override.channels[7] = m_RC_minOutput;
        m_pubRC.publish(rc_override);
	//ROS_INFO("PIDZ ki = %f",m_pidZ.ki());
	mavros_msgs::CommandBool armValue;
	armValue.request.value = true;
	if (ros::service::call("/mavros/cmd/arming", armValue)) {
		ROS_INFO("send armValue successful.");
	} else {
		ROS_INFO("send armValue failed");
	}
        m_pubRC.publish(rc_override);
		
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
        //if (msg.linear.x + msg.linear.y + msg.linear.z + msg.angular.z > 0.0001) {
	//	ROS_INFO("Controller values (x,y,z,Yaw): (%f,%f,%f,%f)",msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
	//}
        //rc_override.channels[0] = m_RC_midRoll + msg.linear.x;
        //rc_override.channels[1] = m_RC_midPitch + msg.linear.y;
        rc_override.channels[2] = m_RC_minThrust + msg.linear.z;
        //rc_override.channels[3] = m_RC_midYaw + msg.angular.z;
        m_pubNav.publish(msg);
    }

    /* setting hover/goal position  manually */
    bool set_goal_height(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Request& res)
    {
        m_goal.pose.position.z = 1;
        m_goal.pose.orientation.w = 1.0;
        ROS_INFO("set_goal_height requested. setting it to %f", m_goal.pose.position.z);
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
	
	m_goal.pose.position.z = 2.0;
        m_goal.pose.orientation.w = 1.0;
        tf::StampedTransform transform;
	getTransform(m_worldFrame, m_frame, transform);
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
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
	//ROS_INFO("getTransform: from \"%s\" to \"%s\" frame.",	sourceFrame.c_str(), targetFrame.c_str());	
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
	if (!ros::ok()) {
		ros::shutdown();
	}
//        ROS_INFO("entered iteration at time: %f", e.current_real.toSec());
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
		getTransform(m_worldFrame, m_frame, transform);
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.15 || m_thrust > m_RC_maxOutput - m_RC_minThrust)
                {
                    pidReset();
		   // ROS_INFO("Value of m_pidZ.ki = %f",m_pidZ.ki());
		    m_RC_minThrust = 1400; // += m_thrust;
                    m_pidZ.setIntegral(0);//m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    ROS_INFO("Shifting to automatic mode, m_thrust=%f, current_z=%f", m_thrust, transform.getOrigin().z());
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += m_RC_step * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    publish_output(msg);
                }

            }
            break;
        case Landing:
            {
		m_goal.pose.position.z -= 0.20 * dt; //m_startZ + 0.1;
                tf::StampedTransform transform;
		getTransform(m_worldFrame, m_frame, transform);
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
		    msg.linear.z = 1100 - m_RC_minThrust;
                    publish_output(msg);
		    ROS_INFO("I am done landing, going to idle");
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
		getTransform(m_worldFrame, m_frame, transform);
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;
		if (iterationCounter==0) {
			//ROS_INFO("targetWorld");
			//ROS_INFO("%f, %f, %f",targetWorld.pose.position.x, targetWorld.pose.position.y, targetWorld.pose.position.z);
			//ROS_INFO("%f, %f, %f, %f",targetWorld.pose.orientation.x, targetWorld.pose.orientation.y, 
			//	targetWorld.pose.orientation.z,targetWorld.pose.orientation.w);
		}
                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);
		if (iterationCounter==0) {
			ROS_INFO("Relative target: %f", targetDrone.pose.position.z);
			//ROS_INFO("targetDrone");
			//ROS_INFO("%f, %f, %f",targetDrone.pose.position.x, targetDrone.pose.position.y, targetDrone.pose.position.z);
			//ROS_INFO("%f, %f, %f, %f",targetDrone.pose.orientation.x, targetDrone.pose.orientation.y, 
			//	targetDrone.pose.orientation.z,targetDrone.pose.orientation.w);
		}

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);
		
                geometry_msgs::Twist msg;
                //msg.linear.x = m_pidX.update(0.0, targetDrone.pose.position.x);
                //msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                //msg.angular.z = m_pidYaw.update(0.0, yaw);
		if (iterationCounter==0) {
			//ROS_INFO("Target Roll, Pitch, Yaw: (%f,%f,%f)", roll, pitch, yaw);
			//ROS_INFO("m_pidX, m_pidY, m_pidZ, m_pidYaw: (%f,%f,%f, %f)", 
			//	msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
			ROS_INFO("m_pidZ=%f",msg.linear.z);
		}
                publish_output(msg);
            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
		msg.linear.z = 1100 - m_RC_minThrust;
                publish_output(msg);
            }
            break;
        }
	if (iterationCounter++ > 100) {
		//ROS_INFO("Controller values (x,y,z,Yaw): (%d,%d,%d,%d)", rc_override.channels[0], rc_override.channels[1], rc_override.channels[2], rc_override.channels[3]);
		ROS_INFO("RC_override: %d",rc_override.channels[2]);
		iterationCounter=0;
	}
        m_pubRC.publish(rc_override);
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
    mavros_msgs::OverrideRCIn rc_override;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    ros::ServiceServer m_serviceHeight;
    float m_thrust;
    float m_startZ;
    int iterationCounter;
    float m_RC_midOutput;
    float m_RC_minOutput;
    float m_RC_maxOutput;
    float m_RC_minThrust;
    float m_RC_midRoll;
    float m_RC_midPitch;
    float m_RC_midYaw;
    float m_RC_step;
};

}

#endif /* _CONTROLLER_H */
