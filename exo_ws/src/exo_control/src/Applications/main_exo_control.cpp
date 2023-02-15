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
#include <unistd.h>

/**
 * Converts from degrees to rad.
 *
 * @param degree Value to be converted.
 * @return Value in radians.
 */
float deg2rad(float degree){
    return (degree * 3.14159265359/180);
}
/**
 * Converts from rad to deg.
 *
 * @param radian Value to be converted.
 * @return Value in degrees.
 */
float rad2deg(float radian)
{
    float pi = 3.14159;
    return(radian * (180 / pi));
}
/**
 * Intention detection: Increments / decrements angle based on input forces and increase factor.
 *
 * @param q_actual input angle.
 * @param f1 first force value.
 * @param f2 second force value.
 * @param inc_f increase factor.
 * @return angle value after update.
 */
float detect_intent(float q_actual, float f1, float f2, float up_lim = 180.0,float low_lim = 0.0, float inc_f = 0.3){

    if(q_actual > low_lim){
        if(f1>f2){
            q_actual = q_actual - inc_f;
        }
    }
    
    if(q_actual < up_lim){
        if(f2>f1){
            q_actual = q_actual + inc_f;
        }
    }
    return deg2rad(q_actual);
}

// Arrays to include skin sensor data
float skin_arr[4];
float skin_arr1[4];
float skin_arr2[4];
float skin_arr3[4];
float skin_arr4[4];
/**
 * Extracts data from cell ids.
 * 
 * @param msg Skin cell data array message.
 */
void chatterCallback(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    bool mode = false;
    // get id from sent data
    int id = msg.data[0].cellId;
    // float skin_arr[4];
    // if cell is the one we want
    if(mode){

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
    else{

        if(msg.data[0].cellId == 10){
            for(int i = 0; i <= 3; i++){
            // for(int i = 0; i <= 1; i++){
                if(i==0){
                    skin_arr[i] =msg.data[0].cellId;
                }
                else{

                    skin_arr[i] =msg.data[0].force[i-1];
                    // skin_arr[i] =msg.data[0].prox[i-1];
                } 

            }
        }
    }
}
/**
 * Extracts data from cell ids.
 * 
 * @param msg Skin cell data array message.
 */
void chatterCallback1(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    
    bool mode = false;
    // get id from sent data
    int id = msg.data[0].cellId;
    // if cell is the one we want
    if(mode){
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
    else{
        if(msg.data[0].cellId == 6){
        
            for(int i = 0; i <= 3; i++){
            // for(int i = 0; i <= 1; i++){
                if(i==0){
                    skin_arr1[i] =msg.data[0].cellId;
                }
                else{

                    skin_arr1[i] =msg.data[0].force[i-1];
                    // skin_arr1[i] =msg.data[0].prox[i-1];
                } 

            }
        }
    }
    
}
/**
 * Extracts data from cell ids.
 * 
 * @param msg Skin cell data array message.
 */
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
/**
 * Extracts data from cell ids.
 * 
 * @param msg Skin cell data array message.
 */
void chatterCallback3(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    // get id from sent data
    int id = msg.data[0].cellId;
    // if cell is the one we want
    if(msg.data[0].cellId == 15){
        
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
/**
 * Extracts data from cell ids.
 * 
 * @param msg Skin cell data array message.
 */
void chatterCallback4(const tum_ics_skin_msgs::SkinCellDataArray msg)
{
    // get id from sent data
    int id = msg.data[0].cellId;
    // if cell is the one we want
    if(msg.data[0].cellId == 1){
        
        for(int i = 0; i <= 3; i++){
        // for(int i = 0; i <= 1; i++){
            if(i==0){
                skin_arr4[i] =msg.data[0].cellId;
            }
            else{

                // skin_arr1[i] =msg.data[0].force[i-1];
                skin_arr4[i] =msg.data[0].acc[i-1];
            } 

        }
    }
}
/**
 * Main function for exo control.
 * 
 */
int main( int argc, char** argv )
{
    // Init ros.
    ros::init(argc, argv, "servo",ros::init_options::AnonymousName);
    ros::NodeHandle n;

    //  Init subscribers and publishers.
    ros::Subscriber skin_sub = n.subscribe("patch1", 10,chatterCallback);
    ros::Subscriber skin_sub1 = n.subscribe("patch2", 10,chatterCallback1);
    ros::Subscriber skin_sub2 = n.subscribe("patch3", 10,chatterCallback2);
    ros::Subscriber skin_sub3 = n.subscribe("patch4", 10,chatterCallback3);
    ros::Subscriber skin_sub4 = n.subscribe("patch5", 10,chatterCallback4);

    ros::Publisher servo_pub = n.advertise<std_msgs::Float32MultiArray>("servo", 10);
    ros::Publisher q_state_pub = n.advertise<std_msgs::Float64>("q_state", 10);

    ros::Rate loop_rate(200);
    float delta_t = 1/(float)200; 

    // Loat params from yaml file.
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
    float L3;
    ns="~L3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L3);
    float m2;
    ns="~m2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m2);
    float m3;
    ns="~m3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m3);
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
    float I333;
    ns="~I333";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I333);
    float g;
    ns="~g";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),g);

    // Torque init.
    float tau1 = 0; 
    float tau2 = 0;

    // Init angle params. 
    float q1 = deg2rad(45);
    float qd1 = 0;
    float qdd1 = 0;

    float q2 = deg2rad(0); 
    float qd2 = 0;
    float qdd2 = 0; 

    // Gravity init. 
    float gx = g;
    float gy = 0;
    float gz = 0;

    // Matrices init.
    float m_matrix1;
    float m_matrix2; 
    float c_matrix;
    float g_matrix1;
    float g_matrix2;
    float b_matrix;
    
    // Load pos control.
    ExoControllers::PosControl posControl1(L1, L2, m2, b1, k1, theta1, gx, gy);
    ExoControllers::PosControl posControl2(L2, L3, m3, b1, k1, theta1, gx, gy);
    float end_q1 = 0;
    float target_q1 = 0;
    Vector3d qEnd;
    float timeEnd = 1;

    // Load force control.
    ExoControllers::ForceControl forceControl1(L2);
    ExoControllers::ForceControl forceControl2(L3);
    float W_des = 0;
    float Ws1 = 0;
    float Ws2 = 0;
    
    forceControl1.init(W_des);
    forceControl2.init(W_des);

    float force_norm = 0.9;
    float force_norm1 = 0.05;
    float force_norm2 = 0.02;
    
    // Mode selection.
    int mode;
    std::cout << "Enter 1 for pos_ctrl, 2 for force_ctrl,3 for force + pos , 4 for proport_ctrl, else dynamic model without input torques: ";
    std::cin >> mode;

    // Starting angle init.
    if (mode == 2 || mode == 4 ){
        q1 = 0;
    }
    else {
        std::cout << "Enter start angle in deg: ";
        float start_q1 ;
        std::cin >> start_q1;
        q1 = deg2rad(start_q1);
        q2 = deg2rad(start_q1);
        // Set target position for modes with position control.
        if (mode == 3 || mode == 1){
            std::cout << "Enter target angle in deg: ";
            // Read input target pos.
            std::cin >> end_q1;
            target_q1 = deg2rad(end_q1);
            qEnd << target_q1,0.0,0.0;
        }
    }
    if(mode==1){
        posControl2.init(qEnd,timeEnd);
    }
    posControl1.init(qEnd,timeEnd);
    
    
    // Send exo home.
    std_msgs::Float32MultiArray q_array;
    std_msgs::Float64 q_result;
    q_array.data.clear();
    q_array.data.push_back(rad2deg(q1));
    q_array.data.push_back(rad2deg(q2));
    servo_pub.publish(q_array);

    ros::spinOnce();
    loop_rate.sleep();
    // Sleeps for 3 second.
    sleep(3);

    // Main Loop.
    while(ros::ok())
    {   
        // Keeping angular displacement values in desired range
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
        if(target_q1 > deg2rad(120.0)){
            target_q1 = deg2rad(120.0);
            
        }
        else if(target_q1 < deg2rad(15.0) ){
            target_q1 = deg2rad(15.0) ;
            
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
        // Compute m,c,g,b matrices.
        float acc_1 = skin_arr4[2];
        float acc_2 = skin_arr4[3];

        // std::cout << acc_z<< "acc "<< acc_y << "\n";
        m_matrix1 = I233 + ((pow(L2,2) * m2)/4);
        m_matrix2 = I333 + ((pow(L3,2) * m3)/4);
        c_matrix = 0;
        // gy = acc_1/0.145;
        // gz = acc_2/0.145;
        g_matrix1 = (-L2*gx*m2*sin(q1))/2 + (L2*gy*m2*cos(q1))/2 - k1*(theta1-q1);
        g_matrix2 = (-L3*gz*m3*cos(q2))/2 + (L3*gx*m3*cos(q1)*sin(q2))/2 - (L3*gy*m3*sin(q1)*sin(q2))/2 - k1*(theta1-q2);
        b_matrix = b1;

        // Get force values from arrays.
        float force = skin_arr[1];
        float force1 = skin_arr1[1];
        float force2 = skin_arr2[1];
        float force3 = skin_arr3[1];
        
        // Call pos_control update if mode 1.
        if (mode == 1){
            // Just elbow:
            // tau1 = posControl1.update(delta_t,q1,qd1,qdd1);
            // Just wrist:
            tau2 = posControl2.update(delta_t,q2,qd2,qdd2);
        }
        // Force control if mode 2.
        else if (mode == 2){
            //Call force control update.
            Ws1 = (force1 - force) / force_norm1;
            Ws2 = (force2 - force3) / force_norm;
            
            // std::cout << Ws1 << "\n";
            tau1 = forceControl1.update(Ws1) + g_matrix1;

            tau2 = forceControl2.update(Ws2) + g_matrix2;
        }
        // Force control + pos control if mode 3 (just elbow for presentation).
        else if (mode == 3){

            // Call force control update.S
            Ws1 = (force1/ force_norm2 - force/ force_norm1) ;
            
            // std::cout << Ws1 << "\n";
            float tau1_force = forceControl1.update(Ws1) + g_matrix1;
            
            // Update target position.
            target_q1 = detect_intent(rad2deg(target_q1),force/ 2,force1/ 2);

            // std::cout << rad2deg(target_q1) << "\n";

            qEnd << target_q1,0.0,0.0;
            posControl1.update_target(qEnd);

            // Call pos control update.
            float tau1_pos = posControl1.update(delta_t,q1,qd1,qdd1);

            // Calculate final torque.
            tau1 = 2.0*tau1_force + tau1_pos;
        }

        // Use angle update proportional to force if mode 4 (no dynamic model).
        if (mode == 4){
            q1 = detect_intent(rad2deg(q1),force/ force_norm,force1/ force_norm);
            q2 = detect_intent(rad2deg(q2),force2/ force_norm,force3/ force_norm);
        }
        // Use Dynamic model for angle update.
        else{
            // Calculate qdd and integrate.
            qdd1=(tau1- b_matrix*qd1 - g_matrix1 - c_matrix*qd1)/m_matrix1;
            qd1 = delta_t *qdd1 + qd1;
            qdd2=(tau2- b_matrix*qd2 - g_matrix2 - c_matrix*qd2)/m_matrix1;
            qd2 = delta_t *qdd2 + qd2;

            // Default q1 update 
            q1 = delta_t *qd1 + q1;

            // Default q2 update 
            q2 = delta_t *qd2 + q2;
        }

        // Print actual angle values.
        std::cout << rad2deg(q1)<< ", "<< rad2deg(q2)<< "\n";

        // Add q1 and q1 to array for ESP32.
        q_array.data.clear();
        q_array.data.push_back(rad2deg(q1));
        q_array.data.push_back(rad2deg(q2));

        // Stop if overshoot.
        if (isnan(q1)&& rad2deg(q1) <= 0.0 ){
        std::cout << "nan found";
        break;
        }
        if (isnan(q2)&& rad2deg(q2) <= 0.0 ){
        std::cout << "nan found";
        break;
        }

        // Publish q state and angle array.
        double q_state = rad2deg(q1)*0.5 + 10.0;

        std::cout << q_state<<"\n";
        q_result.data = q_state;
        servo_pub.publish(q_array);
        q_state_pub.publish(q_result);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
