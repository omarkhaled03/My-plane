
#include <States.h>

namespace gazebo {
    UAVstate::UAVstate():
        ModelPlugin(),
        nh_(nullptr),
        prev_sim_time_(0)
        {}

    UAVstate::~UAVstate()
    {
        updateConnection_.reset();
        if (nh_)
        {
            nh_->shutdown();
            delete nh_;
        }
    }
    void UAVstate::Load(physics::ModelPtr _model , sdf::ElementPtr _sdf)
    {
        if (!ros::isInitialized())
         {
            ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin");
            return;
         }
        ROS_INFO("Loaded the ROSflight SIL plugin");
        model_ = _model;
        world_ = model_->GetWorld();

        if (_sdf->HasElement("namespace"))
            namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
        else
            gzerr <<"Please specify the name space in xcore.\n";
        nh_ = new ros::NodeHandle(namespace_);

        if(_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr<<"Please specify a linkname of the State plugin.\n";

        link_ = model_->GetLink(link_name_);

        if(link_== NULL)
            gzthrow("Couldn't find specified link.\""<<link_name_ <<"\".");
        
        /*Load Wind*/
        wind_speed_topic_ = nh_->param<std::string>("windSpeedTopic", "gazebo/wind_speed");
        state_topic_     = nh_->param<std::string>("stateTopic","state");

        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&UAVstate::onUpdate,this,_1));


        true_state_pub_ = nh_->advertise<fixedwing::state>(state_topic_,100);
        wind_speed_sub_ = nh_->subscribe(wind_speed_topic_,1,&UAVstate::WindSpeedCallback,this);
        ROS_INFO("%s",link_name_.c_str());
        ROS_INFO("%s",namespace_.c_str());
        ROS_INFO("%s",link_);
    }
    void UAVstate::onUpdate(const common::UpdateInfo &_info)
    {

        Publishstate();
    }
  
    void UAVstate::WindSpeedCallback(const geometry_msgs::Vector3 &wind)   
    {
        wind_.N = wind.x;
        wind_.E = wind.y;
        wind_.D = wind.z;

    }
    
    void UAVstate::Publishstate()
    {

        fixedwing::state msg;

        msg.initial_lat = 0;
        msg.initial_lon = 0;
        msg.initial_alt = 0;

        //Get the Position of body in inertia Frame 
        ignition::math::Pose3d pose= link_->WorldCoGPose();
        msg.postion[0] =  pose.Pos().X();
        msg.postion[1] = -pose.Pos().Y();
        msg.postion[2] = -pose.Pos().Z();

        //Get the Rotation of body in inertia Frame 
        ignition::math::Vector3d eul= pose.Rot().Euler();
        msg.phi  =  eul.X();
        msg.theta= -eul.Y();
        msg.psi  = -eul.Z();
        //Get the Velocity of body in Body Frame 
        ignition::math::Vector3d V_liner = link_->RelativeLinearVel();
        msg.u =  V_liner.X();
        msg.v = -V_liner.Y();
        msg.w = -V_liner.Z();
        double u  = msg.u; 
        double v  = msg.v; 
        double w  = msg.w; 
        msg.Vg = sqrt(pow(u,2.0)+ pow(v,2.0)+ pow(w,2.0));
        //Get the Angular Velocity of body in Body Frame 
        ignition::math::Vector3d anguler_v = link_->RelativeAngularVel();
        msg.p  =  anguler_v.X();
        msg.q  = -anguler_v.Y();
        msg.r  = -anguler_v.Z();

        msg.wn = wind_.N;
        msg.we = wind_.E;


        msg.Va    = sqrt(pow(u,2.0)+pow(v,2.0)+pow(w,2.0));
        msg.chi   = atan2(msg.Va*sin(msg.psi),msg.Va*cos(msg.psi));
        msg.alpha = atan2(w,u);
        msg.beta  = asin(v/msg.Va);

        msg.quat_vaild = false;
        msg.quat[0] = V_liner.X();
        msg.quat[1] = V_liner.Y();
        msg.quat[2] = V_liner.Z();

        msg.header.stamp.fromSec(world_->SimTime().Double());
        msg.header.frame_id = 1;

        
        msg.psi_deg = fmod(eul.X(), 2.0*(22/7))*180.0 / (22/7); //-360 to 360
        msg.psi_deg += (msg.psi_deg < -180.0 ? 360.0 : 0.0);
        msg.psi_deg -= (msg.psi_deg > 180.0 ? 360.0 : 0.0);
        msg.chi_deg = fmod(msg.chi, 2.0*(22/7))*180.0 / (22/7); //-360 to 360*/
        msg.chi_deg += (msg.chi_deg < -180.0 ? 360.0 : 0.0);
        msg.chi_deg -= (msg.chi_deg > 180.0 ? 360.0 : 0.0);
        
        true_state_pub_.publish(msg);

    }
GZ_REGISTER_MODEL_PLUGIN(UAVstate);     
}