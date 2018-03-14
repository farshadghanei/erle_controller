/*This is based on the work from: https://github.com/whoenig/crazyflie_ros  */

#ifndef _PID_H_
#define _PID_H_

#include <ros/ros.h>

class PID
{
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        float speedThreshold,
        const std::string& name)
        : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_i_on_off(true)
        , m_speedThreshold(speedThreshold)
        , m_previousTime(ros::Time::now()) {
	ROS_INFO("PID %s initialized with: \n\t\t\t\
                (kp=%.2f kd=%.2f ki=%.2f) \n\t\t\t\
                (minOutput=%.2f maxOutput=%.2f) \n\t\t\t\
                (integratorMin=%.2f integratorMax=%.2f)"
                , name.c_str(), m_kp, m_kd, m_ki
                , m_minOutput, m_maxOutput
                , m_integratorMin, m_integratorMax);
    }

    /* reset the internal state of PID */
    void reset() {
        m_integral = 0;
        m_previousError = 0;
        m_previousTime = ros::Time::now();
    }

    /* start accumulating integral */
    void enableIntegral(void) {
        m_i_on_off = true;
    }

    /* stop accumulating integral */
    void disableIntegral(void) {
        m_i_on_off = false;
    }

    /* manually set integral for biasing*/
    void setIntegral(float integral) {
        m_integral = integral;
    }

    float integral(void) {
        return m_integral;;
    }

    float kp() const {
        return m_kp;
    }

    float ki() const {
        return m_ki;
    }

    float kd() const {
        return m_kd;
    }

    /* update the output of PID based on updated error and time */
    float update(float value, float targetValue) {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = targetValue - value;
        float p = m_kp * error;
        float d = 0;
        if (dt > 0) {
            d = m_kd * (error - m_previousError) / dt;
        }
        if (d < m_speedThreshold) {
            m_i_on_off = true;
        } else {
            m_i_on_off = false;
        }
        if (m_i_on_off) {
            m_integral += error * dt;
        } else {
            m_integral = 0;
        }
        float i = m_ki * m_integral;
        i = std::max(std::min(i, m_integratorMax), m_integratorMin);
        float output = p + d + i;
        m_previousError = error;
        m_previousTime = time;
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_kp;         //proportional coefficient
    float m_kd;         //differential coefficient
    float m_ki;         //integral coefficient
    float m_minOutput;      // minimum total output (biased around 0)
    float m_maxOutput;      // maximum total output (biased around 0)
    float m_integratorMin;  // minimum integral component (biased around 0)
    float m_integratorMax;  // maximum integral component (biased around 0)
    float m_integral;       // integral component uncapped
    float m_previousError;
    ros::Time m_previousTime;
    bool m_i_on_off;        // whether integral component should be accounted
    float m_speedThreshold; // speed under which the integrator works
};

#endif /* _PID_H_ */
