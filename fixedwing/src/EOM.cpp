#include <ros/ros.h>
#include <fixedwing/state.h>
#include <fixedwing/EOM.h>
#include <EOM.h>
#include <cmath>

namespace  gazebo
{
  EOM :: EOM(){

  }
  EOM ::~EOM()
  {
        updateConnection_.reset();
        if (nh_)
        {
            nh_->shutdown();
            delete nh_;
        }
  }

  void EOM::Load(physics::ModelPtr _model , sdf::ElementPtr _sdf){
    model_ = _model;
    world_ = model_->GetWorld();

    if (_sdf->HasElement("namespace"))
      namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
    else
      gzerr <<"Please specify a namespace.";
        
    nh_ = new ros::NodeHandle(namespace_);

    if(_sdf->HasElement("linkName"))
      link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr<<"Please specify a linkName.";

    link_ = model_->GetLink(link_name_);
        
    if(link_==NULL)
        gzthrow("Couldn't find speified link");
        
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&EOM::onUpdate,this,_1));
    EOM_pub_ = nh_->advertise<fixedwing::EOM>("EOM",1);
    //EOM_sub = nh_->subscribe("/fixedwing/state", 1, &EOM::Callback, this);
  }

  void EOM::onUpdate(const common::UpdateInfo &_info){
    GetInfo();
    EOM_publish();
    //gzthrow("Couldn't find specified link.\""<<G8 <<"\".");
    //link_->AddRelativeForce(ignition::math::Vector3d (15, 0.0, 0.0));
    //link_->AddRelativeTorque(ignition::math::Vector3d (0.0, 0.0, 0.0));
    
    
  }

  void EOM::GetInfo(){

    ignition::math::Pose3d pose= link_->WorldCoGPose();
    ignition::math::Vector3d eul= pose.Rot().Euler();
    phi  =  eul.X();
    theta= -eul.Y();
    psi  = -eul.Z();
    ignition::math::Vector3d V_liner = link_->RelativeLinearVel();
    u =  V_liner.X();
    v = -V_liner.Y();
    w = -V_liner.Z();
    ignition::math::Vector3d anguler_v = link_->RelativeAngularVel();
    p  =  anguler_v.X();
    q  = -anguler_v.Y();
    r  = -anguler_v.Z();

    double e0,e1,e2,e3 = quaterian (phi,theta,psi);
    Jx  = 0.213;
    Jy  = 0.171;
    Jz  = 0.350;
    Jxz = 0.04;
    G   = Jx*Jy* pow(Jxz,2);
    G1  = (Jxz*(Jx-Jy+Jz)/G);
    G2  = (Jz*(Jz-Jy)+pow(Jxz,2))/G;
    G3  = (Jz/G);
    G4  = (Jxz)/G;
    G5  = (Jz - Jx)/Jy;
    G6  = (Jxz)/Jy;
    G7  = ((Jx - Jy)*Jx +pow(Jxz,2)/G);
    G8  = Jx/G;
    
  }
  
  int EOM::quaterian (double phi,double theta,double psi){
      double e0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
      double e1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
      double e2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
      double e3 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);

    return e0,e1,e2,e3;
  }
  void EOM::EOM_publish(){

    fixedwing::EOM msg;
    msg.N_dot = (pow(e1,2) + pow(e0,2) - pow(e2,2) - pow(e3,2))*u + 2*(e1*e2-e3*e0)*v + 2*(e1*e3+e2*e0)*w;
    msg.E_dot = 2*(e1*e2+e3*e0)*u  + (pow(e2,2)+pow(e0,2)-pow(e1,2)-pow(e3,2))*v + 2*(e2*e3 - e1*e0)*w;
    msg.d_dot = 2*(e1*e3+e2*e0)*u  +  2*(e3*e2+e1*e0)*v  + (pow(e3,2)+pow(e0,2)-pow(e1,2)-pow(e2,2))*w;

    msg.u_dot =(r*v - q*w) + fz/M;
    msg.v_dot =(p*w - r*u) + fy/M;
    msg.w_dot =(q*u - p*v) + fz/M;

    e0_dot = 0.5*(-p*e1 + -q*e2 + -r*e3);
    e1_dot = 0.5*( p*e0 +  r*e2 + -q*e3);
    e2_dot = 0.5*( q*e0 + -r*e1 +  p*e3);
    e3_dot = 0.5*( r*e0 +  q*e1 + -p*e2);

    
    EOM_pub_.publish(msg);
  }

GZ_REGISTER_MODEL_PLUGIN(EOM); 
}