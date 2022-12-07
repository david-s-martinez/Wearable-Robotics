#include <exo_control/exo_pos_control.h>

namespace ExoControllers{

    PosControl::PosControl(double L1, double L2, double m2, double b1, double k1, 
                            double theta1, double gx, double gy)    
    {
        ROS_INFO_STREAM("Position Controller Created");
        
        std:string ns = "~pos_ctrl";
        std::stringstream s; 
        
        s.str("");
        s<<ns<<"/kp";
        ros::param::get(s.str(),m_kp);
        ROS_WARN_STREAM("pos m_kp: \n"<<m_kp);

        s.str("");
        s<<ns<<"/ki";
        ros::param::get(s.str(),m_ki);
        ROS_WARN_STREAM("pos m_ki: \n"<<m_ki);

        s.str("");
        s<<ns<<"/kd";
        ros::param::get(s.str(),m_kd);
        ROS_WARN_STREAM("pos m_kd: \n"<<m_kd);

        m_L1 = L1;
        m_L2 = L2;
        m_m2 = m2;
        m_b1 = b1;
        m_k1 = k1;
        m_theta1 = theta1;
        m_gx = gx;
        m_gy = gy;

        m_q_des = 0.0;
        m_qd_des = 0.0;
        m_qdd_des = 0.0;
        m_kp = 0.0;
        m_kd = 0.0;
        m_ki = 0.0;
        m_tao = 0.0;
        m_taor = 0.0;
        m_startFlag = false;
        m_deltaQ = 0.0;
        m_timeStart = 0.0;
        m_qStart = Vector3d::Zero(); 
        m_qEnd = Vector3d::Zero(); 
        m_timeEnd = 0.0;
    }

    PosControl::~PosControl()
    {
    }

    // 3. Trajectory generation
    void PosControl::trajGen(double qi, double qf, double qpi,
                             double qpf, double qppi, double qppf,
                             double t0, double tf, double tc,
                             bool startFlag)
    {
      VectorXd static aq1(6);
      VectorXd static q(6);
      MatrixXd static T(6,6);
      VectorXd p(6);
      VectorXd v(6);
      VectorXd a(6);

      if(!startFlag){
        q << qi,qf,qpi,qpf,qppi,qppf;

        T <<     1, t0, pow(t0,2), pow(t0,3),   pow(t0,4),    pow(t0,5),
                 1, tf, ... //toDO! 


        aq1 = T.colPivHouseholderQr().solve(q);
      }

      if(tc<=tf)
      {
      p << 1, tc, pow(tc,2), pow(tc,3), pow(tc,4), pow(tc,5);
      v << //to Do
      a << // to Do
      m_q_des =  p.transpose()*aq1;
      m_qd_des = //to Do
      m_qdd_des = // to Do 
      }
      else{ 
        m_q_des = //to Do
        m_qd_des = qpf;
        m_qdd_des = //to Do
      }
    }

    // 4. regressor
    double PosControl::YrTheta(double q1, double qd1, double qd1r, double qdd1r)
    {
        MatrixXd Yr(1,//to DO);
        MatrixXd Theta(//to DO,1);

        Yr(0,0) = qdd1r;
        Yr(0,1) = //to Do! 

        Theta(0,0) = m_I233 ... // to Do

        MatrixXd taor = Yr*Theta; 
        return taor(0,0);
    }

    bool PosControl::init(Vector3d qEnd, double timeEnd)
    {
        m_qEnd = qEnd;
        m_timeEnd = timeEnd;

        m_startFlag = false;

        return true;
    }

    // 5. PD controller 
    double PosControl::update(double delta_t, double q1, double qd1, double qdd1)
    {
        if(!m_startFlag)
        {
            m_timeStart = ros::Time::now().toSec();
            m_qStart << q1,qd1,qdd1;
            trajGen(m_qStart[0],m_qEnd[0],m_qStart[1],m_qEnd[1], //gen traj to initpos
                m_qStart[2],m_qEnd[2],0.0,m_timeEnd,0.0,m_startFlag);
            m_startFlag = true;
        }
        else
        {
            trajGen(m_qStart[0],m_qEnd[0],m_qStart[1],m_qEnd[1], //gen traj to initpos
                m_qStart[2],m_qEnd[2],0.0,m_timeEnd,ros::Time::now().toSec()-m_timeStart,m_startFlag);
        }

        double deltaQ = q1 - m_q_des;
        double deltaQd = // to Do! 

        double qd1r = // to Do!
        double qdd1r = qdd1 - m_kp*deltaQd; 

        double Sq = // to DO! 
        m_tao = // to Do! 

        return m_tao;
    }

}