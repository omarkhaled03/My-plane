#include <stdio.h>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Eigen>
#include <string>
#include <gazebo/common/Time.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ignition/math.hh>
#include <fixedwing/state.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo{
    
    class  EOM: public ModelPlugin
    {
        public:
            EOM();
           ~EOM();
            double e0,e1,e2,e3;
            double e0_dot,e1_dot,e2_dot,e3_dot;
            double u,v,w;
            double phi,theta,psi;
            double p,q,r;
            double fx,fy,fz;
            double l,m,n;
            //void Callback(const fixedwing::stateConstPtr &msg);
            
        private:    
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void onUpdate(const common::UpdateInfo &_info);
            void GetInfo();
            void EOM_publish();
            int quaterian(double phi,double theta,double psi);

        private:
            
            

            std::string joint_name_;
            std::string link_name_;
            std::string namespace_;
            std::string State_topic_;

            //Eigen::Matrix<double, 14, 1> state_;
            

            physics::WorldPtr world_;
            physics::ModelPtr model_;
            physics::LinkPtr link_;
            physics::JointPtr joint_;
            
            event::ConnectionPtr updateConnection_;

            ros::Publisher EOM_pub_;
            ros::Subscriber EOM_sub;
            ros::NodeHandle *nh_;  

            double Jx,Jy,Jz,Jxz;
            double G,G1,G2,G3,G4,G5,G6,G7,G8;
            double M = 1.5; 
    };
}