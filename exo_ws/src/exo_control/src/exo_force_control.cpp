#include <exo_control/exo_force_control.h>

namespace ExoControllers{
    /**
     * Force control class
     *
     * @param L2 input lenght.
     */
    ForceControl::ForceControl(double L2)
    {
        ROS_INFO_STREAM("Force Controller Created");
        
        std::string ns="~force_ctrl";
        std::stringstream s;
        s.str("");
        s<<ns<<"/kp";
        ros::param::get(s.str(),m_kp);
        ROS_WARN_STREAM("force m_kp: \n"<<m_kp);

        m_L2 = L2;
        m_startFlag = false;
        m_tao = 0;
    }

    ForceControl::~ForceControl()
    {
    }
    /**
     * Force control class init.
     *
     * @param w_des desired wrench.
     */
    bool ForceControl::init(double W_des)
    {
        m_W_des = W_des;
        m_startFlag = false;        
        return true;
    }
    /**
     * Force control update.
     *
     * @param Ws input wrench.
     * @return torque after update.
     */
    double ForceControl::update(double Ws)
    {
        if(!m_startFlag)
        {
            m_startFlag = true;
        }

        m_tao = m_L2 * m_kp * (Ws - m_W_des); // compute torque from wrench.

        return m_tao;
    }

}
