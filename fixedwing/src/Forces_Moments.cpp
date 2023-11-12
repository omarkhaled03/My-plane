#include <Forces_Moments.h>
#include <fixedwing/FAM.h>
namespace  gazebo
{
    ForeceMoments :: ForeceMoments(){}
    ForeceMoments ::~ForeceMoments()
    {
        updateConnection_.reset();
        if (nh_)
        {
            nh_->shutdown();
            delete nh_;
        }
    }

    void ForeceMoments::Load(physics::ModelPtr _model , sdf::ElementPtr _sdf){
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
    

        // physical parameters
        mass_ = nh_->param<double>("mass",3.92);
        Jx_   = nh_->param<double>("Jx",0.213);
        Jy_   = nh_->param<double>("Jy",0.171);
        Jz_   = nh_->param<double>("Jz",0.350);
        Jxz_  = nh_->param<double>("Jxz",0.04);
        rho_  = nh_->param<double>("rho",1.268);
        // Wing Geometry
        wing_.S = nh_->param<double>("wing_s",0.468);
        wing_.b = nh_->param<double>("wing_b",1.8);
        wing_.c = nh_->param<double>("wing_c",0.26);
        wing_.M = nh_->param<double>("wing_M",50);
        
        wing_.epsilon =  nh_->param<double>("wing_epsilon",0.159);
        wing_.alpha0  =  nh_->param<double>("wing_alpha0",0.304);

        // Propeller Coefficients
        prop_.k_motor = nh_->param<double>("k_motor",80.0);
        prop_.k_T_P = nh_->param<double>("k_T_P", 0.0);
        prop_.k_Omega = nh_->param<double>("k_Omega", 0.0);
        prop_.e = nh_->param<double>("prop_e", 0.8);
        prop_.S = nh_->param<double>("prop_S", 0.0314);
        prop_.C = nh_->param<double>("prop_C", 1.0);
    
        // Lift Params
        CL_.O = nh_->param<double>("C_L_O", 0.2869);
        CL_.alpha = nh_->param<double>("C_L_alpha", 5.1378);
        CL_.beta = nh_->param<double>("C_L_beta", 0.0);
        CL_.p = nh_->param<double>("C_L_p", 0.0);
        CL_.q = nh_->param<double>("C_L_q", 1.7102);
        CL_.r = nh_->param<double>("C_L_r", 0.0);
        CL_.delta_a = nh_->param<double>("C_L_delta_a", 0.0);
        CL_.delta_e = nh_->param<double>("C_L_delta_e", 0.5202);
        CL_.delta_r = nh_->param<double>("C_L_delta_r", 0.0);

        // Drag Params
        CD_.O = nh_->param<double>("C_D_O", 0.03087);
        CD_.alpha = nh_->param<double>("C_D_alpha", 0.0043021);
        CD_.beta = nh_->param<double>("C_D_beta", 0.0);
        CD_.p = nh_->param<double>("C_D_p", 0.02815);
        CD_.q = nh_->param<double>("C_D_q", 0.2514);
        CD_.r = nh_->param<double>("C_D_r", 0.0);
        CD_.delta_a = nh_->param<double>("C_D_delta_a", 0.0);
        CD_.delta_e = nh_->param<double>("C_D_delta_e", 0.01879);
        CD_.delta_r = nh_->param<double>("C_D_delta_r", 0.0);

        // ell Params (x axis moment)
        Cell_.O = nh_->param<double>("C_ell_O", 0.0);
        Cell_.alpha = nh_->param<double>("C_ell_alpha", 0.00);
        Cell_.beta = nh_->param<double>("C_ell_beta", 0.0193);
        Cell_.p = nh_->param<double>("C_ell_p", -0.5406);
        Cell_.q = nh_->param<double>("C_ell_q", 0.0);
        Cell_.r = nh_->param<double>("C_ell_r", 0.1929);
        Cell_.delta_a = nh_->param<double>("C_ell_delta_a", 0.2818);
        Cell_.delta_e = nh_->param<double>("C_ell_delta_e", 0.0);
        Cell_.delta_r = nh_->param<double>("C_ell_delta_r", 0.00096);

        // m Params (y axis moment)
        Cm_.O = nh_->param<double>("C_m_O", 0.0362);
        Cm_.alpha = nh_->param<double>("C_m_alpha", -0.2627);
        Cm_.beta = nh_->param<double>("C_m_beta", 0.0);
        Cm_.p = nh_->param<double>("C_m_p", 0.0);
        Cm_.q = nh_->param<double>("C_m_q", -9.7213);
        Cm_.r = nh_->param<double>("C_m_r", 0.0);
        Cm_.delta_a = nh_->param<double>("C_m_delta_a", 0.0);
        Cm_.delta_e = nh_->param<double>("C_m_delta_e", -1.2392);
        Cm_.delta_r = nh_->param<double>("C_m_delta_r", 0.0);                 
        
        // n Params (z axis moment)
        Cn_.O = nh_->param<double>("C_n_O", 0.0);
        Cn_.alpha = nh_->param<double>("C_n_alpha", 0.0);
        Cn_.beta = nh_->param<double>("C_n_beta", 0.08557);
        Cn_.p = nh_->param<double>("C_n_p", -0.0498);
        Cn_.q = nh_->param<double>("C_n_q", 0.0);
        Cn_.r = nh_->param<double>("C_n_r", -0.0572);
        Cn_.delta_a = nh_->param<double>("C_n_delta_a", 0.0095);
        Cn_.delta_e = nh_->param<double>("C_n_delta_e", 0.0);
        Cn_.delta_r = nh_->param<double>("C_n_delta_r", -0.06);

        // Y Params (Sideslip Forces)
        CY_.O       = nh_->param<double>("C_Y_O", 0.0);
        CY_.alpha   = nh_->param<double>("C_Y_alpha", 0.00);
        CY_.beta    = nh_->param<double>("C_Y_beta", -0.2471);
        CY_.p       = nh_->param<double>("C_Y_p", -0.07278);
        CY_.q       = nh_->param<double>("C_Y_q", 0.0);
        CY_.r       = nh_->param<double>("C_Y_r", 0.1849);
        CY_.delta_a = nh_->param<double>("C_Y_delta_a", -0.02344);
        CY_.delta_e = nh_->param<double>("C_Y_delta_e", 10.0);
        CY_.delta_r = nh_->param<double>("C_Y_delta_r", 0.1591);

        // Initialize Wind
        wind_.N = 0.0;
        wind_.E = 0.0;
        wind_.D = 0.0;

        //initialize deltas
        delta_.t = 0.0;
        delta_.e = 0.0;
        delta_.a = 0.0;
        delta_.r = 0.0;

        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ForeceMoments::onUpdate,this,_1));
        FAM_pub_ = nh_->advertise<fixedwing::FAM>("ForcesMoments",1);
        initial_pose_  =link_->WorldCoGPose();  
    }
    void ForeceMoments::onUpdate(const common::UpdateInfo &_info)
    {
    UpdateForcesMoments();
    Forces_Moment_pub();
 
    }

    void ForeceMoments::Reset()
    {
    forces_.Fx=0.0;
    forces_.Fy=0.0;
    forces_.Fz=0.0;
    forces_.l =0.0;
    forces_.m =0.0;
    forces_.n =0.0;

    link_->SetWorldPose(initial_pose_);
    link_->ResetPhysicsStates();

    }
    void ForeceMoments::UpdateForcesMoments()
    {
    ignition::math::Vector3d Liner_Vel = link_->RelativeLinearVel();
    double u = Liner_Vel.X();
    double v = Liner_Vel.Y();
    double w = Liner_Vel.Z();
    ignition::math::Vector3d Angular_Vel =link_->RelativeAngularVel();
    double p = Angular_Vel.X();
    double q = Angular_Vel.Y();
    double r = Angular_Vel.Z();

    double ur = u-wind_.N;
    double vr = v-wind_.E;
    double wr = w-wind_.D;

    double Va = sqrt(pow(ur, 2.0) + pow(vr, 2.0) + pow(wr, 2.0));
     if (Va > 0.000001 && std::isfinite(Va))
    {

        double alpha = atan2(wr , ur);
        double beta = asin(vr/Va);

        double sign = (alpha >= 0 ? 1 : -1); 
        double sigma_a = (1 + exp(-(wing_.M*(alpha - wing_.alpha0))) + exp((wing_.M*(alpha + wing_.alpha0))))/((1 + exp(-(wing_.M*(alpha - wing_.alpha0))))*(1 + exp((wing_.M*(alpha + wing_.alpha0)))));

        
        double CL_a = (1 - sigma_a)*(CL_.O + CL_.alpha*alpha) + sigma_a*(2.0*sign*pow(sin(alpha), 2.0)*cos(alpha));
        double AR = (pow(wing_.b, 2.0))/wing_.S;
        double CD_a = CD_.p + ((pow((CL_.O + CL_.alpha*(alpha)),2.0))/(3.14159*0.9*AR));        

        double CX_a = -CD_a*cos(alpha) + CL_a*sin(alpha);
        double CX_q_a = -CD_.q*cos(alpha) + CL_.q*sin(alpha);
        double CX_deltaE_a = -CD_.delta_e*cos(alpha) + CL_.delta_e*sin(alpha);


        double CZ_a = -CD_a*sin(alpha) - CL_a*cos(alpha);
        double CZ_q_a = -CD_.q*sin(alpha) - CL_.q*cos(alpha);
        double CZ_deltaE_a = -CD_.delta_e*sin(alpha) - CL_.delta_e*cos(alpha);


        forces_.Fx = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CX_a + (CX_q_a*wing_.c*q)/(2.0*Va) + CX_deltaE_a*delta_.e) + 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t), 2.0) - pow(Va,2.0));
        forces_.Fy = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CY_.O + CY_.beta*beta + ((CY_.p*wing_.b*p) /(2.0*Va)) + ((CY_.r*wing_.b*r)/(2.0*Va)) + CY_.delta_a*delta_.a + CY_.delta_r*delta_.r);
        forces_.Fz = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CZ_a + (CZ_q_a*wing_.c*q)/(2.0*Va) + CZ_deltaE_a*delta_.e);

        forces_.l = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.b*(Cell_.O + Cell_.beta*beta + (Cell_.p*wing_.b*p)/(2.0*Va) + (Cell_.r*wing_.b*r)/(2.0*Va) + Cell_.delta_a*delta_.a + Cell_.delta_r*delta_.r) - prop_.k_T_P*pow((prop_.k_Omega*delta_.t), 2.0);
        forces_.m = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.c*(Cm_.O + Cm_.alpha*alpha + (Cm_.q*wing_.c*q)/(2.0*Va) + Cm_.delta_e*delta_.e);
        forces_.n = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.b*(Cn_.O + Cn_.beta*beta + (Cn_.p*wing_.b*p) /(2.0*Va) + (Cn_.r*wing_.b*r)/(2.0*Va) + Cn_.delta_a*delta_.a + Cn_.delta_r*delta_.r);
    }
    else {
            if (!std::isfinite(Va)){
                gzerr << "u = " << u << "\n";
                gzerr << "v = " << v << "\n";
                gzerr << "w = " << w << "\n";
                gzerr << "p = " << p << "\n";
                gzerr << "q = " << q << "\n";
                gzerr << "r = " << r << "\n";
                gzerr << "ur = " << ur << "\n";
                gzerr << "vr = " << vr << "\n";
                gzerr << "wr = " << wr << "\n";
                gzthrow("we have a NaN or an infinity:\n");
            }
            else
            {
                forces_.Fx = 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t), 2.0));
                forces_.Fy = 0.0;
                forces_.Fz = 0.0;
                forces_.l = 0.0;
                forces_.m = 0.0;
                forces_.n = 0.0;  
            }
        }
     link_->AddRelativeForce(ignition::math::Vector3d (forces_ .Fx, forces_ .Fy, forces_ .Fz));
     link_->AddRelativeTorque(ignition::math::Vector3d (forces_.l, forces_.m, forces_.n));
    }

    void ForeceMoments::Forces_Moment_pub(){
        //if (forces_.Fx>0){
        
        fixedwing::FAM msg;
        ignition::math::Vector3d Forces = link_->RelativeForce();
        msg.fx = Forces.X();
        msg.fy = Forces.Y();
        msg.fz = Forces.Z();

        ignition::math::Vector3d Moments =  link_->RelativeTorque();
        msg.l = Moments.X();
        msg.m = Moments.Y();
        msg.n = Moments.Z();
        FAM_pub_.publish(msg);
        //}
    }
GZ_REGISTER_MODEL_PLUGIN(ForeceMoments);
} // namespace  gazebo link_->SetLinearVel(ignition::math::Vector3d(0.5,0,0));
