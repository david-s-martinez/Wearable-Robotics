#include "ros/ros.h"
#include <iostream>
#include <cmath>
double deg2rad(double degree){
    return (degree * 3.14159265359/180);
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "exo_control",
            ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(200);

    double delta_t = 1/(double)200; 

    // load your params
    double L1;
    std::string ns="~L1";
    std::stringstream s;
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L1);
    double L2;
    ns="~L2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L2);
    double m2;
    ns="~m2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m2);
    double b1;
    ns="~b1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),b1);
    double k1;
    ns="~k1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),k1);
    double theta1;
    ns="~theta1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),theta1);
    double I233;
    ns="~I233";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I233);
    double g;
    ns="~g";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),g);
    // torque tau
    double tau = 0; 
    
    //init params 
    double q1 = deg2rad(90); 
    double qd1 = 0;
    double qdd1 = 0; 

    // static g 
    double gx = g;
    double gy = 0;
    double gz = 0;

    double m_matrix; 
    double c_matrix;
    double g_matrix;
    double b_matrix;

    while(ros::ok())
    {        
        m_matrix = I233 + ((pow(L2,2) * m2)/4);
        c_matrix = -1.5*L1*L2*m2*sin(q1)*qd1;
        g_matrix = (-L2*gx*m2*sin(q1))/2 + (L2*gy*m2*cos(q1))/2 - k1*(theta1-q1);
        b_matrix = b1*qd1;
        
        // calculate qdd1 and integrate 
        qdd1=(tao - b_matrix*qd1 - g_matrix - c_matrix*qd1)/m_matrix;
        qd1 = delta_t *qdd1 + qd1;
        q1 = delta_t *qd1 + q1;

        // ROS_WARN_STREAM("qdd1"<<qdd1);
        // ROS_WARN_STREAM("qd1"<<qd1);
        ROS_WARN_STREAM("q1"<<q1);

        ros::spinOnce();
        r.sleep();
    }
    
    return 0; 

}
