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
        , m_previousTime(ros::Time::now()) {
	ROS_INFO("PID %s initialized with min_op=%f max_op=%f", name.c_str(), m_minOutput, m_maxOutput);
    }

    void reset() {
        m_integral = 0;
        m_previousError = 0;
        m_previousTime = ros::Time::now();
    }

    void enableIntegral(void) {
        m_i_on_off = true;
    }

    void disableIntegral(void) {
        m_i_on_off = false;
    }

    void setIntegral(float integral) {
        m_integral = integral;
    }

    float getIntegral(void) {
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

    float update(float value, float targetValue) {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = targetValue - value;
        if (m_i_on_off) {
            m_integral += error * dt;
        } else {
            m_integral = 0;
        }
        float p = m_kp * error;
        float d = 0;
        if (dt > 0) {
            d = m_kd * (error - m_previousError) / dt;
        }
        float i = m_ki * m_integral;
        i = std::max(std::min(i, m_integratorMax), m_integratorMin);
        float output = p + d + i;
        m_previousError = error;
        m_previousTime = time;
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    ros::Time m_previousTime;
    bool m_i_on_off;
};

#endif /* _PID_H_ */
