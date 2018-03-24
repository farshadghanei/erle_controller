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
        float i_approximnity,
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
        , m_i_force_off(false)
        , m_i_approximity(i_approximnity)
        , m_previousTime(ros::Time::now())
        , m_name(name) {
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
        m_i_force_off = false;
    }

    /* stop accumulating integral */
    void disableIntegral(void) {
        m_i_force_off = true;
    }

    /* manually set integral for biasing*/
    void setIntegral(float integral) {
        m_integral = integral;
    }

    float integral(void) {
        return m_integral;;
    }

    float p() {
        return m_out_p;
    }

    float i() {
        return m_out_i;
    }

    float d() {
        return m_out_d;
    }

    /* update the output of PID based on updated error and time */
    float update(float value, float targetValue) {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = targetValue - value;
        float speed = (error - m_previousError) / dt;
        m_out_p = m_kp * error;
        m_out_d = 0;
        if (dt > 0) {
            m_out_d = m_kd * speed;
        }

        if (!m_i_force_off && error < m_i_approximity) {
            m_i_on_off = true;
            m_integral += error * dt;
        } else {
            m_i_on_off = false;
            m_integral = 0;
        }
        m_out_i = m_ki * m_integral;
        m_out_i = std::max(std::min(m_out_i, m_integratorMax), m_integratorMin);
        float output = m_out_p + m_out_d + m_out_i;

        ROS_INFO("[PID_update_%s]: time(%.2f), prev_time(%.2f), dt(%.2f), error(%.2f), prev_error(%.2f), speed(%.2f), output_uncapped(%.2f), m_out_p(%.2f), m_out_d(%.2f), m_out_i(%.2f), kp(%.2f), kd(%.2f), ki(%.2f), i_on_off(%d), m_integral(%.2f)",
               m_name.c_str(), time.toSec(), m_previousTime.toSec(), dt, error, m_previousError, speed, output, m_out_p, m_out_d, m_out_i, m_kp, m_kd, m_ki, m_i_on_off, m_integral);

        m_previousError = error;
        m_previousTime = time;
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_out_p;         //proportional output
    float m_out_d;         //differential output
    float m_out_i;         //integral output
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
    bool m_i_force_off;        // whether integral component should be accounted, set externally
    bool m_i_on_off;        // whether integral component should be accounted, set internally based on speed
    float m_i_approximity; // approximity around target in which the integrator works
    std::string m_name;
};

#endif /* _PID_H_ */
