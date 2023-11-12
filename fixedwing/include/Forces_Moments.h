# include <ros/ros.h>
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <gazebo/common/Time.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ignition/math.hh>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <fixedwing/input.h>

namespace gazebo
{
class ForeceMoments: public ModelPlugin
{
            public:
            ForeceMoments();
            ~ForeceMoments();
            void InitializeParams();

            void InputCallback(const fixedwing::inputPtr &msg);
        protected:
            void UpdateForcesMoments();
            void Reset();
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void onUpdate(const common::UpdateInfo &);
            void SendForces();
            void Forces_Moment_pub();
            
        private:
            double t_;
            double e_;
            double a_;
            double r_;
            std::string command_topic_;
            std::string wind_speed_topic_;
            std::string joint_name_;
            std::string link_name_;
            std::string parent_frame_id_;
            std::string motor_speed_pub_topic_;
            std::string namespace_;
            //std::string msg;
            struct Command
            {
                double t;
                double e;
                double a;
                double r;
                double state;
            }  command_;
            //fixedwing::input msg;
            physics::WorldPtr world_;
            physics::ModelPtr model_;
            physics::LinkPtr link_;
            physics::JointPtr joint_;
            physics::EntityPtr parent_link_;
            event::ConnectionPtr updateConnection_;

            // physics parameters

            double mass_;
            double Jx_;
            double Jy_;
            double Jz_;
            double Jxz_;
            double rho_;

            // Aerodynamic Coeff
            struct WingCoeff
            {
                double S;
                double b;
                double c;
                double M;
                double epsilon;
                double alpha0;

            }wing_;

            // Propeller Coeff
            struct PropCoeff
            {
                double k_motor;
                double k_T_P ;
                double k_Omega;
                double e;
                double S;
                double C;
            }prop_;

            // Lift Coeff
            struct LiftCoeff
            {       
                double O;
                double alpha;
                double beta;
                double p; 
                double q;
                double r;
                double delta_a;
                double delta_e;
                double delta_r;

            };

            LiftCoeff CL_;
            LiftCoeff CD_;
            LiftCoeff Cm_; 
            LiftCoeff CY_;
            LiftCoeff Cell_;
            LiftCoeff Cn_;

            //Actuators
            struct Actuators
            {
                double e;
                double a;
                double r;
                double t;
            }delta_;

              // wind
            struct Wind
            {
                double N;
                double E;
                double D;
            } wind_;
            struct ForcesAndTorques
            {
                double Fx;
                double Fy;
                double Fz;
                double l;
                double m;
                double n;
            } forces_;

            double sampling_time_ = 0;
            double prev_sim_time_ = 0;

            double F_X;
            double F_Y;
            double F_Z;
            ignition::math::Pose3d initial_pose_;

            ros::NodeHandle *nh_;
            ros::Subscriber command_sub_;
            ros::Publisher FAM_pub_;
            ros::Subscriber FAM_sub_;
            boost::thread callback_queue_thread_;

            ignition::math::Vector3d wind_speed_W_;
    };
    
}

