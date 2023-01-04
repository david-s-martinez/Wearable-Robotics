#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <tum_ics_skin_descr/Patch/TfMarkerDataPatches.h>

#include <tum_ics_skin_msgs/SkinCellDataArray.h>
#include <exo_control/exo_pos_control.h>
#include <exo_control/exo_force_control.h>
#include <iostream>
#include <cmath>
#include <typeinfo>

// class ExoControl
// {
// public:
//     ExoControl(ros::NodeHandle n){
//         ros::Subscriber skin_sub = n.subscribe("patch1", 10,&ExoControl::chatterCallback,this);
    
//     }
//     double* getMessageData()
//   {
//     return acc_arr;
//   }
//     double acc_arr[4];

//     void chatterCallback(const tum_ics_skin_msgs::SkinCellDataArray msg)
//     {
//         // double acc_arr[4];
//         int id = msg.data[0].cellId;
//         // std::cout << id << "\n";
        
//         if(msg.data[0].cellId == 10){
//             for(int i = 0; i <= 3; i++){
//                 if(i==0){
//                     acc_arr[i] =msg.data[0].cellId;
//                 }
//                 else{

//                     acc_arr[i] =msg.data[0].acc[i-1];
//                 } 
//                 // std::cout << "%f",acc_arr[i] ;
//                 std::cout << acc_arr[i]<< ",";

//             }

//         std::cout <<"\n";
//         }
        
//         // std::cout << msg<< "\n";
// }
// };
double deg2rad(double degree){
    return (degree * 3.14159265359/180);
}
double control_q1(double q1, double force,double force1 ){

    if(q1>0){
            if (force>force1){
                q1--;
            }

        }
        

    if(q1<180 ){
        if(force1>force){
            
            q1++;
        }
    }
    return q1;
}
double rad2deg(double radian)
{
    double pi = 3.14159;
    return(radian * (180 / pi));
}
double acc_arr[4];
double acc_arr1[4];
void chatterCallback(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    // get id from sent data
    int id = msg.data[0].cellId;
    // double acc_arr[4];
    // if cell is the one we want
    
    if(msg.data[0].cellId == 10){
        // for(int i = 0; i <= 3; i++){
        for(int i = 0; i <= 1; i++){
            if(i==0){
                acc_arr[i] =msg.data[0].cellId;
            }
            else{

                // acc_arr[i] =msg.data[0].force[i-1];
                acc_arr[i] =msg.data[0].prox[i-1];
            } 

        }
    }
}
void chatterCallback1(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    // get id from sent data
    int id = msg.data[0].cellId;
    // if cell is the one we want
    if(msg.data[0].cellId == 6){
        
        // for(int i = 0; i <= 3; i++){
        for(int i = 0; i <= 1; i++){
            if(i==0){
                acc_arr1[i] =msg.data[0].cellId;
            }
            else{

                // acc_arr1[i] =msg.data[0].force[i-1];
                acc_arr1[i] =msg.data[0].prox[i-1];
            } 

        }
    }
}
int main( int argc, char** argv )
{
  
    ros::init(argc, argv, "servo",ros::init_options::AnonymousName);
            
    ros::NodeHandle n;
    // ExoControl exo(n);
    ros::Subscriber skin_sub = n.subscribe("patch1", 10,chatterCallback);
    ros::Subscriber skin_sub1 = n.subscribe("patch2", 10,chatterCallback1);

    ros::Publisher servo_pub = n.advertise<std_msgs::Float32>("servo", 10);
    ros::Rate loop_rate(200);
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
    double q1 = deg2rad(0); 
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
    
    //load force control
    ExoControllers::PosControl posControl(L1, L2, m2, b1, k1, theta1, gx, gy);
    Vector3d qEnd;
    qEnd << deg2rad(180),0.0,0.0;
    double timeEnd = 1;
    posControl.init(qEnd,timeEnd);

    //load force control
    ExoControllers::ForceControl forceControl(L2);
    double W_des = 0;
    double Ws = 0;
    forceControl.init(W_des);
    
    std_msgs::Float32 q1_;
    q1_.data = 0;
    servo_pub.publish(q1_);
    ros::spinOnce();
    loop_rate.sleep();
    
    while(ros::ok())
    {   
        // double* message_data = exo.getMessageData();
        // std::cout << message_data[1]<< "\n";
        
        // keeping angular displacement values in desired range
        if(q1 > deg2rad(180.0)){
            q1 = deg2rad(180.0);
            qd1 = 0.0;
            qdd1 = 0.0;
        }
        else if(q1 < 0.0 ){
            q1 = 0.0;
            qd1 = 0.0;
            qdd1 = 0.0;
            
        }

        m_matrix = I233 + ((pow(L2,2) * m2)/4);
        // c_matrix = -1.5*L1*L2*m2*sin(q1)*qd1; wrong
        c_matrix = 0;
        g_matrix = (-L2*gx*m2*sin(q1))/2 + (L2*gy*m2*cos(q1))/2 - k1*(theta1-q1);
        b_matrix = b1;

        double force = acc_arr[1];
        double force1 = acc_arr1[1];
        
        // std::cout << force << "\n";

        // Proportional force control
        // q1 = control_q1(q1,force,force1);
        

        // Force control update
        
        Ws = (force1 - force) / 0.9;
        
        //weird stuff:
        // if(Ws > 0.3 || Ws < -0.3){
        //     tau = forceControl.update(Ws) + g_matrix;

        // }
        // else if (Ws < 0.3 && Ws > -0.3 ){
        //     Ws = 0.0;
        //     // tau = 0.0;
        // }

        // std::cout << Ws << "\n";
        //call force control update
        tau = forceControl.update(Ws) + g_matrix;

        //call pos control update
        // tau = posControl.update(delta_t,q1,qd1,qdd1);

        // calculate qdd1 and integrate 
        qdd1=(tau- b_matrix*qd1 - g_matrix - c_matrix*qd1)/m_matrix;
        qd1 = delta_t *qdd1 + qd1;
        q1 = delta_t *qd1 + q1;


        std::cout << rad2deg(q1)<< "\n";
        std_msgs::Float32 q1_;
        q1_.data = rad2deg(q1);

        if (isnan(q1)&& rad2deg(q1) <= 0.0 ){
        std::cout << "nan found";
        break;
        }
        servo_pub.publish(q1_);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
//      while(ros::ok())
//     {   
        
//         double force = acc_arr[1];
//         double force1 = acc_arr1[1];
//         q1 = control_q1(q1,force,force1);
//         std::cout << q1<< "\n";
//         std_msgs::Float32 q1_;
//         q1_.data = q1;

//         if (isnan(q1)&& rad2deg(q1) <= 0.0 ){
//         std::cout << "nan found";
//         break;
//         }
//         servo_pub.publish(q1_);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
//     return 0; 

}
