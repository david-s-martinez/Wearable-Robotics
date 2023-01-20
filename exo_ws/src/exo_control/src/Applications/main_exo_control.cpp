#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"


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
//     float* getMessageData()
//   {
//     return skin_arr;
//   }
//     float skin_arr[4];

//     void chatterCallback(const tum_ics_skin_msgs::SkinCellDataArray msg)
//     {
//         // float skin_arr[4];
//         int id = msg.data[0].cellId;
//         // std::cout << id << "\n";
        
//         if(msg.data[0].cellId == 10){
//             for(int i = 0; i <= 3; i++){
//                 if(i==0){
//                     skin_arr[i] =msg.data[0].cellId;
//                 }
//                 else{

//                     skin_arr[i] =msg.data[0].acc[i-1];
//                 } 
//                 // std::cout << "%f",skin_arr[i] ;
//                 std::cout << skin_arr[i]<< ",";

//             }

//         std::cout <<"\n";
//         }
        
//         // std::cout << msg<< "\n";
// }
// };
float deg2rad(float degree){
    return (degree * 3.14159265359/180);
}
float control_q1(float q1, float force,float force1 ){

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
    return deg2rad(q1);
}
float rad2deg(float radian)
{
    float pi = 3.14159;
    return(radian * (180 / pi));
}
float skin_arr[4];
float skin_arr1[4];
float skin_arr2[4];
float skin_arr3[4];
void chatterCallback(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    // get id from sent data
    int id = msg.data[0].cellId;
    // float skin_arr[4];
    // if cell is the one we want
    
    if(msg.data[0].cellId == 10){
        // for(int i = 0; i <= 3; i++){
        for(int i = 0; i <= 1; i++){
            if(i==0){
                skin_arr[i] =msg.data[0].cellId;
            }
            else{

                // skin_arr[i] =msg.data[0].force[i-1];
                skin_arr[i] =msg.data[0].prox[i-1];
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
                skin_arr1[i] =msg.data[0].cellId;
            }
            else{

                // skin_arr1[i] =msg.data[0].force[i-1];
                skin_arr1[i] =msg.data[0].prox[i-1];
            } 

        }
    }
}

void chatterCallback2(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    // get id from sent data
    int id = msg.data[0].cellId;
    // if cell is the one we want
    if(msg.data[0].cellId == 12){
        
        // for(int i = 0; i <= 3; i++){
        for(int i = 0; i <= 1; i++){
            if(i==0){
                skin_arr2[i] =msg.data[0].cellId;
            }
            else{

                // skin_arr1[i] =msg.data[0].force[i-1];
                skin_arr2[i] =msg.data[0].prox[i-1];
            } 

        }
    }
}

void chatterCallback3(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    // get id from sent data
    int id = msg.data[0].cellId;
    // if cell is the one we want
    if(msg.data[0].cellId == 14){
        
        // for(int i = 0; i <= 3; i++){
        for(int i = 0; i <= 1; i++){
            if(i==0){
                skin_arr3[i] =msg.data[0].cellId;
            }
            else{

                // skin_arr1[i] =msg.data[0].force[i-1];
                skin_arr3[i] =msg.data[0].prox[i-1];
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
    ros::Subscriber skin_sub2 = n.subscribe("patch3", 10,chatterCallback2);
    ros::Subscriber skin_sub3 = n.subscribe("patch4", 10,chatterCallback3);

    ros::Publisher servo_pub = n.advertise<std_msgs::Float32MultiArray>("servo", 10);
    ros::Publisher q_state_pub = n.advertise<std_msgs::Float64>("q_state", 10);

    ros::Rate loop_rate(200);
    float delta_t = 1/(float)200; 

    // load your params
    float L1;
    std::string ns="~L1";
    std::stringstream s;
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L1);
    float L2;
    ns="~L2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L2);
    float m2;
    ns="~m2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m2);
    float b1;
    ns="~b1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),b1);
    float k1;
    ns="~k1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),k1);
    float theta1;
    ns="~theta1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),theta1);
    float I233;
    ns="~I233";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I233);
    float g;
    ns="~g";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),g);
    // torque tau
    float tau1 = 0; 
    
    float tau2 = 0;

    //init params 
    float q1 = deg2rad(45);
    float qd1 = 0;
    float qdd1 = 0;

    float q2 = deg2rad(0); 
    float qd2 = 0;
    float qdd2 = 0; 

    // static g 
    float gx = g;
    float gy = 0;
    float gz = 0;

    float m_matrix; 
    float c_matrix;
    float g_matrix1;
    float g_matrix2;
    float b_matrix;
    
    //load force control
    ExoControllers::PosControl posControl(L1, L2, m2, b1, k1, theta1, gx, gy);
    Vector3d qEnd;
    qEnd << deg2rad(120),0.0,0.0;
    float timeEnd = 1;
    posControl.init(qEnd,timeEnd);

    //load force control
    ExoControllers::ForceControl forceControl(L2);
    float W_des = 0;
    float Ws1 = 0;

    float Ws2 = 0;
    forceControl.init(W_des);
    // Mode selection
    int mode;
    std::cout << "Enter 1 for pos_ctrl, 2 for force_ctrl, 3 for proport_ctrl, else default: ";
    std::cin >> mode;
    // send exo home
    if (mode == 2 || mode == 3 ){
        q1 = 0;

    }
    else {
        std::cout << "Enter start angle in deg: ";
        float start_q1 ;
        std::cin >> start_q1;
        q1 = deg2rad(start_q1);
    }
    
    std_msgs::Float32MultiArray q_array;
    std_msgs::Float64 q_result;
    q_array.data.clear();
    q_array.data.push_back(rad2deg(q1));
    q_array.data.push_back(rad2deg(q2));
    servo_pub.publish(q_array);

    ros::spinOnce();
    loop_rate.sleep();
    while(ros::ok())
    {   
        // float* message_data = exo.getMessageData();
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
        if(q2 > deg2rad(180.0)){
            q2 = deg2rad(180.0);
            qd2 = 0.0;
            qdd2 = 0.0;
        }
        else if(q2 < 0.0 ){
            q2 = 0.0;
            qd2 = 0.0;
            qdd2 = 0.0;
            
        }

        m_matrix = I233 + ((pow(L2,2) * m2)/4);
        c_matrix = 0;
        g_matrix1 = (-L2*gx*m2*sin(q1))/2 + (L2*gy*m2*cos(q1))/2 - k1*(theta1-q1);
        g_matrix2 = (-L2*gx*m2*sin(q2))/2 + (L2*gy*m2*cos(q2))/2 - k1*(theta1-q2);
        b_matrix = b1;

        float force = skin_arr[1];
        float force1 = skin_arr1[1];
        float force2 = skin_arr2[1];
        float force3 = skin_arr3[1];
        
        if (mode == 1){
            // call pos control update
            tau1 = posControl.update(delta_t,q1,qd1,qdd1);

        }
        else if (mode == 2){
            // Force control update
            //call force control update
            Ws1 = (force1 - force) / 0.9;
            Ws2 = (force2 - force3) / 0.9;
            
            // std::cout << Ws1 << "\n";
            tau1 = forceControl.update(Ws1) + g_matrix1;

            tau2 = forceControl.update(Ws2) + g_matrix2;
        }

        if (mode == 3){
            // Proportional force control
            q1 = control_q1(rad2deg(q1),force,force1);
            q2 = control_q1(rad2deg(q2),force2,force3);


        }
        else{
            // calculate qdd1 and integrate 
            qdd1=(tau1- b_matrix*qd1 - g_matrix1 - c_matrix*qd1)/m_matrix;
            qd1 = delta_t *qdd1 + qd1;
            // Default q1 update 
            q1 = delta_t *qd1 + q1;

            qdd2=(tau2- b_matrix*qd2 - g_matrix2 - c_matrix*qd2)/m_matrix;
            qd2 = delta_t *qdd2 + qd2;
            // Default q1 update 
            q2 = delta_t *qd2 + q2;
        }

        std::cout << rad2deg(q1)<< ", "<< rad2deg(q2)<< "\n";

        q_array.data.clear();
        q_array.data.push_back(rad2deg(q1));
        q_array.data.push_back(rad2deg(q2));

        if (isnan(q1)&& rad2deg(q1) <= 0.0 ){
        std::cout << "nan found";
        break;
        }
        if (isnan(q2)&& rad2deg(q2) <= 0.0 ){
        std::cout << "nan found";
        break;
        }
        double q_state = rad2deg(q1);
        q_result.data = q_state;
        servo_pub.publish(q_array);
        q_state_pub.publish(q_result);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
