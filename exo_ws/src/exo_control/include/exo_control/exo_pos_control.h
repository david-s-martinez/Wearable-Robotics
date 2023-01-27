#ifndef EXO_ROBOT_POSCONTROL_H
#define EXO_ROBOT_POSCONTROL_H

#include <ros/ros.h>
#include <Eigen/Dense>

using namespace Eigen;
using std::string;

namespace ExoControllers{

  class PosControl{
      private:
        ros::NodeHandle n;
        double m_q_des;
        double m_qd_des;
        double m_qdd_des;
        double m_kp;
        double m_kd;
        double m_ki;
        double m_tao;
        double m_I233;
        double m_L1;
        double m_L2;
        double m_m2;
        double m_b1; 
        double m_k1;
        double m_theta1;
        double m_gx;
        double m_gy;
        double m_taor;
        bool m_startFlag;
        double m_deltaQ;
        double m_timeStart;
        Vector3d m_qStart;
        Vector3d m_qEnd;
        double m_timeEnd;

      public:
        PosControl(double L1, double L2, double m2, double b1, double k1, double theta1, double gx, double gy);
        ~PosControl();

        void trajGen(double qi, double qf, double qpi,
                             double qpf, double qppi, double qppf,
                             double t0, double tf, double tc,
                             bool startFlag);

        double YrTheta(double q1, double qd1, double qd1r, double qdd1r);
        
        bool init(Vector3d qEnd, double timeEnd);
        bool update_target(Vector3d qEnd);
        double update(double delta_t, double q1, double qd1, double qdd1);

  };

}

#endif