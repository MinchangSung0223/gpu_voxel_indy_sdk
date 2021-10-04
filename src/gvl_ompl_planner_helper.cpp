// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*

#include "gvl_ompl_planner_helper.h"
#include <Eigen/Dense>
#include <signal.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
//#include "Poco/Net/Net.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include "include/spline.h"
//#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <thread>

#include <stdio.h>
#include <iostream>
#define IC_PERFORMANCE_MONITOR
#include <icl_core_config/Config.h>
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <vector>

using namespace gpu_voxels;
namespace bfs = boost::filesystem;
#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
#define RED BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (10 % 249) )
#define PURPLE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (150 % 249) )
#define BLUE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (200 % 249))
#define YELLOW BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (1 % 249))
using namespace NRMKIndy::Service::DCP;
using namespace std;
IndyDCPConnector connector("192.168.0.7", ROBOT_INDY7);

double task_goal_values[7] = {0,0,0,1,3.0,2.0,1.35}; 
double task_joint_values[6] = {-0.21868971,-0.46094136,-1.019970203,0,-1.622457735,-0.195476836}; 

Vector3ui map_dimensions(250,250,300);
bool cmdMove=false;
void printCatesianKDLFrame(KDL::Frame frame,char* str ){
    std::cout<<"======="<<str<<"=======\n\n"<<endl;
    for(int i =0;i<4;i++){
        for(int j=0;j<4;j++)
            std::cout<<frame(i,j)<<"\t";
        std::cout<<"\n"<<std::endl;
    }
}
std::vector<std::array<double,JOINTNUM>> changeTrajectoryType(std::vector<KDL::JntArray> q_list){
    std::vector<std::array<double,JOINTNUM>> ret;
    ret.clear();
    for(int i=0;i<q_list.size();i++){

        KDL::JntArray temp = q_list.at(i);
        std::array<double,JOINTNUM> temp2;
        for(int j=0;j<jointnum;j++){
            temp2.at(j) = temp(j);
        }
        ret.push_back(temp2);
    }
    return ret;
}
std::vector<KDL::JntArray> changeTrajectoryType(std::vector<std::array<double,JOINTNUM>> q_list){
    std::cout<<"ChangeTrajectoryType 2"<<std::endl;
   std::vector<KDL::JntArray> ret;
    ret.clear();
    for(int i=0;i<q_list.size();i++){

        std::array<double,JOINTNUM> temp = q_list.at(i);
        std::cout<<"ChangeTrajectoryType 2 - "<<i<<std::endl;

        KDL::JntArray temp2(jointnum);
        for(int j=0;j<jointnum;j++){
            temp2(j) = temp.at(j);
        }
        ret.push_back(temp2);
    }
    return ret;
}
std::vector<std::array<double,JOINTNUM>> splineJointTrajectory(std::vector<std::array<double,JOINTNUM>> q_list,double Tf, double dt, int deriv_num) {
   int N = q_list.size();
   if(N<3){
      q_list.push_back(q_list.at(N-1));
      N=N+1;
   }

   std::vector<double> Tlist;

   for(int j=0;j<N;j++){
    Tlist.push_back(double(Tf/(N-1)*j/1.0));
   }        
   std::vector<std::vector<double>> all_spline_thetalist;
   std::vector<std::vector<double>> all_spline_dthetalist;
    
   for(int j=0;j<JOINTNUM;j++){
       std::vector<double> thetalist;
       for(int i = 0;i<N;i++){
           std::array<double,JOINTNUM> temp=q_list.at(i);
           thetalist.push_back(temp[j]);
       }
       tk::spline s(Tlist,thetalist,tk::spline::cspline,false, tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
           std::vector<double> spline_thetalist;
           std::vector<double> spline_dthetalist;

       for(double t=0+dt;t<=Tf;){
        spline_thetalist.push_back(s(t));
        spline_dthetalist.push_back(s.deriv(1,t));
        t = t+dt;
       }
        all_spline_thetalist.push_back(spline_thetalist);
        all_spline_dthetalist.push_back(spline_dthetalist);
   }
   std::vector<std::array<double,JOINTNUM>>  spline_q_list;
   std::vector<std::array<double,JOINTNUM>>  spline_dq_list;

   for(int i=0;i<all_spline_thetalist.at(0).size()+1;i++){
           std::array<double,JOINTNUM> temp;
       for(int j=0;j<JOINTNUM;j++){
         std::vector<double>temp_ = all_spline_thetalist.at(j);
         temp[j]=temp_[i];
       }
       spline_q_list.push_back(temp);
   }

   for(int i=0;i<all_spline_dthetalist.at(0).size()+1;i++){
           std::array<double,JOINTNUM> temp;
       for(int j=0;j<JOINTNUM;j++){
         std::vector<double>temp_ = all_spline_dthetalist.at(j);
         temp[j]=temp_[i];
       }
       spline_dq_list.push_back(temp);
   }
   if (deriv_num==0){
       return spline_q_list;

   }
   else if (deriv_num==1){
       return spline_dq_list;
   }

}

std::vector<KDL::JntArray>  GvlOmplPlannerHelper::doTaskPlanning(double goal_values[7],KDL::JntArray start_values,ob::PathPtr path){
    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");
    auto space(std::make_shared<ob::RealVectorStateSpace>(jointnum));
    ob::RealVectorBounds bounds(jointnum);
      for(int j = 0;j<jointnum;j++){
          bounds.setLow(j,q_min(j));
          bounds.setHigh(j,q_max(j));
      }


    std::vector<KDL::JntArray> q_list;
    q_list.clear();
    space->setBounds(bounds);

      this->si_->setStateValidityChecker(this->getptr());
      this->si_->setMotionValidator(this->getptr());
      this->si_->setup();

    og::PathSimplifier simp(this->si_);

    KDL::JntArray q_start(jointnum);  
    KDL::JntArray q_result(jointnum);  
    
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    KDL::Frame cartesian_pos;
    KDL::Frame cartesian_pos_result;
    
    KDL::Frame goal_pose( KDL::Rotation::Quaternion(goal_values[0],goal_values[1],goal_values[2],goal_values[3]),KDL::Vector(goal_values[4],goal_values[5],goal_values[6]));

    fk_solver.JntToCart(q_start, cartesian_pos);

    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.1);

    for(int i=0;i<jointnum;i++){
        q_start(i) = start_values(i);
    }

    //bool ret = iksolver1.CartToJnt(q_start,goal_pose,q_result);
    //std::cout<<"ik ret : "<<ret<<std::endl;
  
    


    fk_solver.JntToCart(q_result, cartesian_pos_result);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    /*
    if(ret==false){
        for(int i = 0;i<jointnum;i++){
        start[i] = q_start(i);
        goal[i] = task_joint_values[i];
         }

    }else{

    for(int i = 0;i<jointnum;i++){
            start[i] = q_start(i);
            goal[i] = q_result(i);
        }
    }
    */

    for(int i = 0;i<jointnum;i++){
        start[i] = q_start(i);
        goal[i] = task_joint_values[i];
         }
    //std::system("clear");
    LOGGING_INFO(Gpu_voxels, "PDEF \n" << endl);
    auto pdef(std::make_shared<ob::ProblemDefinition>(this->si_));
    pdef->setStartAndGoalStates(start, goal);
    LOGGING_INFO(Gpu_voxels, "PLANNER \n" << endl);

    //auto planner(std::make_shared<og::KPIECE1>(this->si_));

    auto planner(std::make_shared<og::AITstar>(this->si_));

    
    planner->setProblemDefinition(pdef);
    planner->setup();
    int succs = 0;
    
    LOGGING_INFO(Gpu_voxels, "WHILE \n" << endl);
    float solveTime = 0.5;
    int no_succs_count = 0;
     while(succs<1)
    {
        double sum = 0.0;
        for(int k = 0;k<jointnum;k++){
            sum += sqrt((start[k]-goal[k])*(start[k]-goal[k]));
        }
        if(sum< 0.01){
            LOGGING_INFO(Gpu_voxels, "COMPLETE MOTION PLANNING \n" << endl);
            break;
        }
        try{
            planner->clear();
            LOGGING_INFO(Gpu_voxels,"SUCCESS : " <<succs<<", START PLANNING \n" << endl);

            const std::function< bool()> ptc;

            ob::PlannerStatus  solved = planner->ob::Planner::solve(solveTime);
            LOGGING_INFO(Gpu_voxels, "end PLANNING \n" << endl);

            PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");


            if (solved)
            {
                ++succs;
                path = pdef->getSolutionPath();
                std::cout << "Found solution:" << std::endl;
                path->print(std::cout);
                simp.simplifyMax(*(path->as<og::PathGeometric>()));


            }else{
                std::cout << "No solution could be found" << std::endl;
                no_succs_count++;
                solveTime +=0.1; 
                if(no_succs_count>5)return q_list;
            }

            PERF_MON_SUMMARY_PREFIX_INFO("planning");
            std::cout << "END OMPL" << std::endl;
           }
        catch(int expn){
            std::cout << "ERRORROROROROR" << std::endl;
             return q_list;
        }

    }

            std::cout << "ENDdddddd OMPL" << std::endl;


    og::PathGeometric* solution= path->as<og::PathGeometric>();

    solution->interpolate(interpolate_num);
    int step_count = solution->getStateCount();
    std::cout<<step_count<<std::endl;
    std::cout << "ENDdddddd OMPL" << std::endl;

    for(int i=0;i<step_count;i++){
        const double *values = solution->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
        double *temp_values = (double*)values;
        KDL::JntArray temp_joints_value(jointnum);
        for(int j =0;j<jointnum;j++)
            temp_joints_value(j)=temp_values[j];    
        q_list.push_back(temp_joints_value);
    
     }
    std::cout << "END OMPL22" << std::endl;

    return q_list;


}




void rosjointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    gvl->clearMap("myRobotMap");
    gvl->clearMap("myRobotMapBitVoxel");
    
    gvl->clearMap("myRobotCollisionMap");
    gvl->clearMap("myRobotCollisionMapBitVoxel");
    
    
    for(size_t i = 0; i < msg->name.size(); i++)
    {
        myRobotJointValues[joint_names[i]] = msg->position[i];
        joint_states(i) = msg->position[i];
        
    }
    gvl->setRobotConfiguration("myUrdfRobot",myRobotJointValues);
    gvl->setRobotConfiguration("myUrdfCollisionRobot",myRobotJointValues);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMapBitVoxel",YELLOW);
    //LOGGING_INFO(Gpu_voxels, "ROS JointState " << endl);

    gvl->insertRobotIntoMap("myUrdfCollisionRobot","myRobotCollisionMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfCollisionRobot", "myRobotCollisionMapBitVoxel", BLUE);

}

void rosDesiredPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    
    LOGGING_INFO(Gpu_voxels,msg->position.x<< endl);
     task_goal_values[0] = msg->orientation.x;
     task_goal_values[1] = msg->orientation.y;
     task_goal_values[2] = msg->orientation.z;
     task_goal_values[3] = msg->orientation.w;
     task_goal_values[4] = msg->position.x;
     task_goal_values[5] = msg->position.y;
     task_goal_values[6] = msg->position.z;
    new_pose_received=true;

}



int toggle = 1;


double calcErr(KDL::JntArray q1, double* q2){
    double sum=0;
    double err = 9999;
    for(int i = 0;i<jointnum;i++){
        sum+=(q1(i)-q2[i])*(q1(i)-q2[i]);
    }
    err = sqrt(sum);
    return err;
}


void rosMovingFlagCallback(const std_msgs::Bool::ConstPtr& msg){
    isMoving = msg->data;
    std::cout<<"roscallbackISMOVING :  "<<isMoving << endl;
    try{
        if(joint_trajectory.size()>0 ){
            std::cout<<"rrrrrrrrrrrrroscallbackISMOVING :  "<<isMoving << endl;

            KDL::JntArray temp_q = joint_trajectory.at(0);
            KDL::JntArray temp_qdot = vel_trajectory.at(0);
            J send_q;
            J send_qdot;
            send_q.at(0) = temp_q(0);
            send_q.at(1) = temp_q(1);
            send_q.at(2) = temp_q(2);
            send_q.at(3) = temp_q(3);
            send_q.at(4) = temp_q(4);
            send_q.at(5) = temp_q(5);

            
            send_qdot.at(0) = temp_qdot(0);
            send_qdot.at(1) = temp_qdot(1);
            send_qdot.at(2) = temp_qdot(2);
            send_qdot.at(3) = temp_qdot(3);
            send_qdot.at(4) = temp_qdot(4);
            send_qdot.at(5) = temp_qdot(5);

            
            _tcpclient.set(send_q,send_qdot);

            reverse(joint_trajectory.begin(), joint_trajectory.end());
            reverse(vel_trajectory.begin(), vel_trajectory.end());
            
            joint_trajectory.pop_back();
            vel_trajectory.pop_back();


            reverse(joint_trajectory.begin(), joint_trajectory.end());
            reverse(vel_trajectory.begin(), vel_trajectory.end());

        }else if(joint_trajectory.size()==0){
            double task_joint_values1[6];
             //task_joint_values1[0] = -0.33248515;
             //task_joint_values1[1] = -0.6935937;
             //task_joint_values1[2] = -1.06691955;
             //task_joint_values1[3] = 0.00349066;
             //task_joint_values1[4] = -1.37915889;
             //task_joint_values1[5] = -0.33039076; 

             task_joint_values1[0] = -0.49305541;
             task_joint_values1[1] = -0.33353235;
             task_joint_values1[2] = -0.72431149;
             task_joint_values1[3] = 0.41626094;
             task_joint_values1[4] = -1.22940967;
             task_joint_values1[5] =  -0.45012032; 


            double task_joint_values2[6];

            //task_joint_values2[0] = 0.7686428426666667;
            //task_joint_values2[1] = -0.5401792911111111;
            //task_joint_values2[2] = -1.3383181920000002;
            //task_joint_values2[3] = -0.0005235986666666667;
            //task_joint_values2[4] = -1.257858530222222;
            //task_joint_values2[5] = 0.7726570991111112; 

            task_joint_values2[0] =  -0.69690983;
            task_joint_values2[1] = -0.71837737;
            task_joint_values2[2] =  -1.37758809;
            task_joint_values2[3] =  1.2163197 ;
            task_joint_values2[4] = -0.58782677;
            task_joint_values2[5] = -1.60936777; 

            
            double val1 = calcErr(joint_states,task_joint_values1);
            double val2 = calcErr(joint_states,task_joint_values2);
            if(val1<0.01 ||val2<0.01){
                if(val1>= val2){
                    for(int i =0;i<jointnum;i++)
                        task_joint_values[i] = task_joint_values1[i];
                }
                else
                    for(int i =0;i<jointnum;i++)
                        task_joint_values[i] = task_joint_values2[i];
            }

            new_pose_received=true;


        }
    }
    catch(int e){


    }
    

}


void roscallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
    std::vector<Vector3f> point_data;
    point_data.resize(msg->points.size());
    for (uint32_t i = 0; i < msg->points.size(); i++)
    {
    	point_data[i].x = msg->points[i].x;
    	point_data[i].y = msg->points[i].y;
    	point_data[i].z = msg->points[i].z;
    }
    my_point_cloud.update(point_data);
    my_point_cloud.transformSelf(&tf);
    Matrix4f base_tf=Matrix4f(1,0,0,base_x,0,1,0,base_y,0,0,1,base_z,0,0,0,1);
    my_point_cloud.transformSelf(&base_tf);



    new_data_received=true;
}

Eigen::Matrix4f GvlOmplPlannerHelper::loadBaseToCam(Eigen::Matrix4f TBaseToCamera){

    Eigen::Matrix3f Rx  = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Ry  = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rz  = Eigen::Matrix3f::Identity();
    float roll = cam_roll;//-PI/2.0;
    float pitch = cam_pitch;
    float yaw = cam_yaw;//-PI/2.0;

    Rx(1,1) = cos(roll);
    Rx(1,2) = -sin(roll);
    Rx(2,1) = sin(roll);
    Rx(2,2) = cos(roll);
    Ry(0,0) = cos(pitch);
    Ry(0,2) = sin(pitch);
    Ry(2,0) = -sin(pitch);
    Ry(2,2) = cos(pitch);
    Rz(0,0) = cos(yaw);
    Rz(0,1) = -sin(yaw);
    Rz(1,0) = sin(yaw);
    Rz(1,1) = cos(yaw);

    Eigen::Matrix3f R = Rz*Ry*Rx;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f TBaseToCameraTemp = TBaseToCamera;
    T(0,0)=R(0,0);
    T(0,1)=R(0,1);
    T(0,2)=R(0,2);
    T(1,0)=R(1,0);
    T(1,1)=R(1,1);
    T(1,2)=R(1,2);
    T(2,0)=R(2,0);
    T(2,1)=R(2,1);
    T(2,2)=R(2,2);
    TBaseToCamera(0,3)=0;
    TBaseToCamera(1,3)=0;
    TBaseToCamera(2,3)=0;
    
    TBaseToCamera = TBaseToCamera*T;
    TBaseToCamera(0,3) = TBaseToCameraTemp(0,3);
    TBaseToCamera(1,3) = TBaseToCameraTemp(1,3);
    TBaseToCamera(2,3) = TBaseToCameraTemp(2,3);
    
    return TBaseToCamera;
}


void pubJointState(double *jointValue,ros::Publisher *pub_joint ){
    sensor_msgs::JointState jointState;
    for(int j =0;j<jointnum;j++)
        jointState.name.push_back(joint_names[j]);

    
    for(int i = 0;i<jointnum;i++)
        jointState.position.push_back(jointValue[i]);
    
    jointState.header.stamp=ros::Time::now();
    pub_joint->publish(jointState);

}

double calcErr(KDL::JntArray q1,KDL::JntArray q2){
    double sum=0;
    double err = 9999;
    for(int i = 0;i<jointnum;i++){
        sum+=(q1(i)-q2(i))*(q1(i)-q2(i));
    }
    err = sqrt(sum);
    return err;
}
void check_state(){


}
void go_joint(std::array<double, JOINTNUM> target){
    std::vector<std::array<double, JOINTNUM>> q_list;
    q_list.clear();
    std::array<double, JOINTNUM> q_start = {joint_states(0), joint_states(1), joint_states(2), joint_states(3), joint_states(4), joint_states(5)};
    std::array<double, JOINTNUM> q_goal = {target.at(0), target.at(1), target.at(2), target.at(3), target.at(4), target.at(5)};
    q_list.push_back(q_start);
    q_list.push_back(q_goal);
    q_list.push_back(q_goal);

    std::vector<std::array<double, JOINTNUM>> home_trajectory = splineJointTrajectory(q_list,15,0.1, 1);
    std::vector<std::array<double, JOINTNUM>> home_vel_trajectory = splineJointTrajectory(q_list,15,0.1, 0);
    
    for(int j= 0;j<home_trajectory.size();j++){
        J tempq = home_trajectory.at(j);
        J tempqdot = home_trajectory.at(j);
        
        _tcpclient.set(tempq,tempqdot);
    }


}
void go_home(){
    std::vector<std::array<double, JOINTNUM>> q_list;
    q_list.clear();
    std::array<double, JOINTNUM> q_start = {joint_states(0), joint_states(1), joint_states(2), joint_states(3), joint_states(4), joint_states(5)};
    std::array<double, JOINTNUM> q_goal = {0,-0.262,-1.5707,0.0,-1.309,0};
    q_list.push_back(q_start);
    q_list.push_back(q_goal);
    q_list.push_back(q_goal);

    std::vector<std::array<double, JOINTNUM>> home_trajectory = splineJointTrajectory(q_list,15,0.1, 1);
    std::vector<std::array<double, JOINTNUM>> home_vel_trajectory = splineJointTrajectory(q_list,15,0.1, 0);
    
    for(int j= 0;j<home_trajectory.size();j++){
        J tempq = home_trajectory.at(j);
        J tempqdot = home_trajectory.at(j);
        
        _tcpclient.set(tempq,tempqdot);
    }

}
void GvlOmplPlannerHelper::tcpIter(){
    int count = 0;
    while(true){
        J temp_q =  _tcpclient.get();
       joint_states(0) = temp_q.at(0);
       joint_states(1) = temp_q.at(1);
       joint_states(2) = temp_q.at(2);
       joint_states(3) = temp_q.at(3);
       joint_states(4) = temp_q.at(4);
       joint_states(5) = temp_q.at(5);
       if(count ==0){
            go_home();
            count = 1;
        }
        else if(count == 1){
            std::array<double, JOINTNUM>  temp = { -0.49305541,-0.33353235 ,-0.72431149,0.41626094 ,-1.22940967,-0.45012032};

            go_joint(temp);
            count = 2;
        }
        
       usleep(10000);
    }
   
   
}

void GvlOmplPlannerHelper::rosIter(){
    int argc;
    char **argv;
    ros::init(argc,argv,"gpu_voxel");

    const Vector3f camera_offsets(0.0f,
                                 0.0f, 
                                 0.0f); 


        
    TBaseToCamera = GvlOmplPlannerHelper::loadBaseToCam(TBaseToCamera);
    tf = Matrix4f(TBaseToCamera(0,0),TBaseToCamera(0,1),TBaseToCamera(0,2),TBaseToCamera(0,3)
        ,TBaseToCamera(1,0),TBaseToCamera(1,1),TBaseToCamera(1,2),TBaseToCamera(1,3)
        ,TBaseToCamera(2,0),TBaseToCamera(2,1),TBaseToCamera(2,2),TBaseToCamera(2,3)
        ,TBaseToCamera(3,0),TBaseToCamera(3,1),TBaseToCamera(3,2),TBaseToCamera(3,3));
    std::cout<<"==========TBaseToCmaera==========\n"<<std::endl;
    tf.print();
  std::cout << "Press Enter Key if ready!" << std::endl;
  std::cin.ignore();
    ros::NodeHandle nh;
    ros::NodeHandle nh2;

    ros::Subscriber joint_sub = nh.subscribe("/joint_smc", 1, rosjointStateCallback); 
    ros::Subscriber desiredPose_sub = nh.subscribe("/desired_pose", 1, rosDesiredPoseCallback); 
    ros::Subscriber moving_flag = nh.subscribe("/ismoving", 1, rosMovingFlagCallback); 

    
    //ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/color/points", 1,roscallback);
    std::cout<<point_topic_name<<std::endl;
    const char* point_topic_name_ =point_topic_name.c_str();
    ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(point_topic_name_, 1,roscallback);

    //ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/cam_0_zf", 1,roscallback);
    //ros::Publisher pub_joint =  nh.advertise<sensor_msgs::JointState>("/joint_states_desired", 1000);
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);

    ros::Publisher cmdMove_pub = nh2.advertise<std_msgs::Bool>("/cmdMove", 100); 
    ros::Rate r(100);
    new_data_received = true; // call visualize on the first iteration
    new_pose_received=false;
    size_t num_colls = 0;
    countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));
    myRobotCollisionMapBitVoxel = dynamic_pointer_cast<BitVectorVoxelList>(gvl->getMap("myRobotCollisionMapBitVoxel"));

    while (ros::ok())
    {
       // LOGGING_INFO(Gpu_voxels, "ROSITER " << endl);

        //init
        if(new_data_received){
          //  LOGGING_INFO(Gpu_voxels, "Recived Pointcloud " << endl);

            countingVoxelList->clearMap();
            myEnvironmentMap->clearMap();



            countingVoxelList->insertPointCloud(my_point_cloud,eBVM_OCCUPIED);
            countingVoxelList->as<gpu_voxels::voxellist::CountingVoxelList>()->subtractFromCountingVoxelList(
            myRobotCollisionMapBitVoxel->as<gpu_voxels::voxellist::BitVectorVoxelList>(),
            Vector3f());
            myEnvironmentMap->merge(countingVoxelList);
            num_colls = gvl->getMap("countingVoxelList")->as<gpu_voxels::voxellist::CountingVoxelList>()->collideWith(gvl->getMap("mySolutionMap")->as<gpu_voxels::voxellist::BitVectorVoxelList>(), 1.0f);

            if(num_colls>coll_threshold*2){
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                cmdMove=false;
                new_pose_received = true;
            }else{
                cmdMove=true;
                }


        }
        if(new_pose_received){
           // LOGGING_INFO(Gpu_voxels, "Recived Target Position " << endl);
            ob::PathPtr path;
            std::vector<KDL::JntArray> temp=GvlOmplPlannerHelper::doTaskPlanning(task_goal_values,joint_states,path);
            std::cout<<"tempetmpetmpemt"<<std::endl;
            if(temp.size()>0){
                std::vector<std::array<double,JOINTNUM>> temp_ = changeTrajectoryType(temp);

                float end_time=15;
                float dt = 0.1;
                std::vector<std::array<double,JOINTNUM>> vel_trajectory_ = splineJointTrajectory(temp_,end_time,dt, 1);
                std::vector<std::array<double,JOINTNUM>> joint_trajectory_ = splineJointTrajectory(temp_,end_time,dt, 0);
                std::cout<<"vel trajectory::"<<vel_trajectory_.size()<<std::endl;
                vel_trajectory=changeTrajectoryType(vel_trajectory_);
                joint_trajectory = changeTrajectoryType(joint_trajectory_);
            }
        }
       
          //std::cout << "!!!!!!!!!!!!!!!isMoving!!!!!!!!! " << isMoving << std::endl;
          //  std::vector<KDL::JntArray> joint_trajectory_temp = joint_trajectory;
            if(joint_trajectory.size()){
                trajectory_msgs::JointTrajectory jointTrajectory;

                jointTrajectory = trajectory_msgs::JointTrajectory();
                for(int j =0;j<jointnum;j++)
                    jointTrajectory.joint_names.push_back(joint_names[j]);
            
                jointTrajectory.header.stamp = ros::Time::now();
                trajectory_msgs::JointTrajectoryPoint points;
                for(int j=0;j<joint_trajectory.size();j++){
                    points=trajectory_msgs::JointTrajectoryPoint();
                    KDL::JntArray temp_q = joint_trajectory.at(j);
                    
                    for(int k=0;k<jointnum;k++){
                        points.positions.push_back(temp_q(k));
                        points.velocities.push_back(0.0);
                    }
                    
                    points.time_from_start = ros::Duration(0.01);
                    jointTrajectory.points.push_back(points);

                }
  
                pub.publish(jointTrajectory);   
                std::cout << "!!!!!!!!!!!!!!!PUBLISH!!!!!!!!! " << isMoving << std::endl;
               // joint_trajectory.clear();
               // isMoving=false;
            }
            
        

        GvlOmplPlannerHelper::visualizeSolution(joint_trajectory);    

        GvlOmplPlannerHelper::doVis();

        new_data_received = false;
        new_pose_received=false;
            std_msgs::Bool cmdMove_msg;
            cmdMove_msg.data=cmdMove;
            cmdMove_pub.publish(cmdMove_msg);
        ros::spinOnce();

        r.sleep();
    }

    exit(EXIT_SUCCESS);
}



GvlOmplPlannerHelper::GvlOmplPlannerHelper(const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si)
{


    si_ = si;
    stateSpace_ = si_->getStateSpace().get();

    assert(stateSpace_ != nullptr);

    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(map_dimensions.x,map_dimensions.y, map_dimensions.z, voxel_side_length);

    // We add maps with objects, to collide them
    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotMapBitVoxel");


    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotCollisionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentAllMap");

    //gvl->insertPointCloudFromFile("myEnvironmentAllMap", "./binvox/environment_all.binvox", true,
    //                                  gpu_voxels::eBVM_OCCUPIED, true, gpu_voxels::Vector3f(0.0, 0.0, -0.01),1);
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotCollisionMapBitVoxel");
    myEnvironmentMap = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("myEnvironmentMap"));

    gvl->addMap(MT_BITVECTOR_VOXELLIST,"mySolutionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myQueryMap");
    gvl->addMap(MT_COUNTING_VOXELLIST,"countingVoxelList");
    gvl->addRobot("myUrdfCollisionRobot",colilsion_urdf_name , true);
    gvl->addRobot("myUrdfRobot",urdf_name , true);
    

     if (!kdl_parser::treeFromFile(urdf_name, my_tree)){
             LOGGING_INFO(Gpu_voxels,"Failed to construct kdl tree");
    }

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl);

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Chain load : "<<my_tree.getChain(base_link_name,tcp_link_name,my_chain) <<"\n"<< endl);


    PERF_MON_ENABLE("pose_check");
    PERF_MON_ENABLE("motion_check");
    PERF_MON_ENABLE("motion_check_lv");
}



GvlOmplPlannerHelper::~GvlOmplPlannerHelper()
{
    gvl.reset(); // Not even required, as we use smart pointers.
}

void GvlOmplPlannerHelper::moveObstacle()
{

}

void GvlOmplPlannerHelper::doVis()
{
     //LOGGING_INFO(Gpu_voxels, "Dovis " << endl);
     gvl->visualizeMap("myEnvironmentMap");
     gvl->visualizeMap("myEnvironmentAllMap");
     
 
    
    gvl->visualizeMap("myRobotMap");    
    gvl->visualizeMap("myRobotMapBitVoxel");    
        gvl->visualizeMap("mySolutionMap");

    gvl->visualizeMap("myRobotCollisionMapBitVoxel");
    //gvl->visualizeMap("myRobotCollisionMap");


    gvl->visualizeMap("countingVoxelList");
    gvl->insertBoxIntoMap(Vector3f(1.0, 0.8 ,0.0), Vector3f(2.7, 0.81 ,1.5), "myEnvironmentMap", eBVM_OCCUPIED, 2);
    gvl->insertBoxIntoMap(Vector3f(1.0, 2.2 ,0.0), Vector3f(2.7, 2.21 ,1.5), "myEnvironmentMap", eBVM_OCCUPIED, 2);
    gvl->insertBoxIntoMap(Vector3f(1.0, 0.8 ,1.5), Vector3f(2.7, 2.21 ,1.5), "myEnvironmentMap", eBVM_OCCUPIED, 2);
     //gvl->insertBoxIntoMap(Vector3f(1.0, 0.8 ,0.0), Vector3f(2.7, 2.21 ,-0.01), "myEnvironmentMap", eBVM_OCCUPIED, 2);
}


void GvlOmplPlannerHelper::visualizeSolution(ob::PathPtr path)
{
    gvl->clearMap("mySolutionMap");

    PERF_MON_SUMMARY_PREFIX_INFO("pose_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check_lv");

    //std::cout << "Robot consists of " << gvl->getRobot("myUrdfRobot")->getTransformedClouds()->getAccumulatedPointcloudSize() << " points" << std::endl;

    og::PathGeometric* solution = path->as<og::PathGeometric>();
    solution->interpolate();


    for(size_t step = 0; step < solution->getStateCount(); ++step)
    {

        const double *values = solution->getState(step)->as<ob::RealVectorStateSpace::StateType>()->values;

        robot::JointValueMap state_joint_values;
        for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = values[j];
        // update the robot joints:
        gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("myUrdfRobot", "mySolutionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
    }

    gvl->visualizeMap("mySolutionMap");

}
void GvlOmplPlannerHelper::visualizeSolution(std::vector<KDL::JntArray> solution)
{
    gvl->clearMap("mySolutionMap");

   for(int j = 0;j<solution.size();j++)
    {

        KDL::JntArray temp_q = solution.at(j);

        robot::JointValueMap state_joint_values;
        for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = temp_q(j);
        // update the robot joints:
        gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("myUrdfRobot", "mySolutionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (j % 249) ));
    }
    gvl->visualizeMap("mySolutionMap");

}

void GvlOmplPlannerHelper::insertStartAndGoal(const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal) const
{
    gvl->clearMap("myQueryMap");

    robot::JointValueMap state_joint_values;
    for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = start[j];

    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START));

        for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = goal[j];
    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+1));
}

bool GvlOmplPlannerHelper::isValid(const ompl::base::State *state) const
{
    PERF_MON_START("inserting");

    std::lock_guard<std::mutex> lock(g_i_mutex);

    gvl->clearMap("myRobotMap");

    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;

    robot::JointValueMap state_joint_values;
    for(int j =0;j<jointnum;j++)
        state_joint_values[joint_names[j]] = values[j];
    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    // insert the robot into the map:
    gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

    //gvl->setRobotConfiguration("myUrdfCollisionRobot",state_joint_values);
    //gvl->insertRobotIntoMap("myUrdfCollisionRobot","myRobotCollisionMap",eBVM_OCCUPIED);


    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Pose Insertion", "pose_check");

    PERF_MON_START("coll_test");
    size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);
    gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+30));

   // size_t num_colls_pc2 = gvl->getMap("myRobotCollisionMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);
    //gvl->insertRobotIntoMap("myUrdfCollisionRobot", "myRobotCollisionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+30));

    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "pose_check");

    std::cout << "Validity check on state ["  << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << ", " << values[4] << ", " << values[5] << "] resulting in " <<  num_colls_pc << " colls." << std::endl;
    //std::cout << "Validity check on state2 ["  << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << ", " << values[4] << ", " << values[5] << "] resulting in " <<  num_colls_pc2 << " colls." << std::endl;

    
    //return num_colls_pc==0;
    if(num_colls_pc<coll_threshold){
        return true;
    }
    else if(num_colls_pc>=coll_threshold){
        return false;
    }

}

bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                       std::pair< ompl::base::State*, double > & lastValid) const
{

    //    std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //    std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //    std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;


    std::lock_guard<std::mutex> lock(g_j_mutex);
    gvl->clearMap("myRobotMap");

    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    //std::cout << "Called interpolating motion_check_lv to evaluate " << nd << " segments" << std::endl;

    PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check_lv");
    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))

            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;

            }
        }

        si_->freeState(test);

    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;
}



bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::lock_guard<std::mutex> lock(g_i_mutex);
    gvl->clearMap("myRobotMap");


    //        std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //        std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //        std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;



    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    // not required with ProbabVoxels:
    //    if(nd > 249)
    //    {
    //        std::cout << "Too many intermediate states for BitVoxels" << std::endl;
    //        exit(1);
    //    }

    if (nd > 1)
    {
        PERF_MON_START("inserting");

        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);


            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;

            robot::JointValueMap state_joint_values;
            for(int j =0;j<jointnum;j++)
                state_joint_values[joint_names[j]] = values[j];

            // update the robot joints:
            gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
            // insert the robot into the map:
            gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

        }
        PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check");

        si_->freeState(test);

        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Motion Insertion", "motion_check");

        //gvl->visualizeMap("myRobotMap");
        PERF_MON_START("coll_test");
        size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);

        std::cout << "CheckMotion1 for " << nd << " segments. Resulting in " << num_colls_pc << " colls." << std::endl;
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "motion_check");

       // result = (num_colls_pc == 0);
         if(num_colls_pc<coll_threshold){
                result= true;
         }
            else if(num_colls_pc>=coll_threshold){
                result= false;
            }

    }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;


}
