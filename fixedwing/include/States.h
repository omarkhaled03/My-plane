

#ifndef fixedwing_UAV_STATE_H
#define fixedwing_UAV_STATE_H


#include <stdio.h>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Eigen>
#include <string>
#include <gazebo/common/Time.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ignition/math.hh>
#include <fixedwing/state.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo{

    static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";
    class UAVstate: public ModelPlugin
    {

        public:
            UAVstate();
            ~UAVstate();
        void InitializeParams();
        protected:
            void Publishstate();
            void Load(physics::ModelPtr _model , sdf::ElementPtr _sdf);
            void onUpdate(const common::UpdateInfo &);
        private:
            std::string state_topic_;
            std::string wind_speed_topic_;
            std::string joint_name_;
            std::string link_name_;
            std::string parent_frame_id_;
            std::string motor_speed_pub_topic_;
            std::string namespace_;

            physics::WorldPtr world_;
            physics::ModelPtr model_;
            physics::LinkPtr  link_;
            physics::JointPtr joint_;
            physics::EntityPtr parent_link_;
            event:: ConnectionPtr updateConnection_;
            struct Wind
            {
                double N;
                double E;
                double D;
            }   wind_;

            double sampling_time_;
            double prev_sim_time_;


            ros::NodeHandle *nh_;
            ros::Subscriber wind_speed_sub_;
            ros::Publisher  true_state_pub_;

            boost::thread callback_queue_thread_;

            void QueueThread();
            void WindSpeedCallback(const geometry_msgs::Vector3 &wing);

            
    };
}
#endif