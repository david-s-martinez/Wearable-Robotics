#include <exo_control/exo_force_control.h>

namespace ExoControllers{

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

    bool ForceControl::init(double W_des)
    {
        m_W_des = W_des;
        m_startFlag = false;        
        return true;
    }

    double ForceControl::update(double Ws)
    {
        if(!m_startFlag)
        {
            m_startFlag = true;
        }

        m_tao = m_L2 * Ws; // to DO

        return m_tao;
    }

}
