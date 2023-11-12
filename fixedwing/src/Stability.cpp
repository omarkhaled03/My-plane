#include "Stability.h"

namespace fixedwing

{
        Stability::Stability() : Controller()
            {
            current_zone  = alt_zones::Take_off;

            c_error_      = 0;
            c_integrator_ = 0;
            r_error_      = 0;
            r_integrator  = 0;
            p_error_      = 0;
            p_integrator_ = 0;
         }
        void Stability::control(const params_s &params, const input_s &input, output_s &output )
        {
            output.delta_r = 0;
            output.phi_c = course_hold(input.chi_c  , input.chi, input.phi_ff, input.r, params,input.Ts);
            output.delta_a = roll_hold(output.phi_c,input.phi,input.p,params , input.Ts);
            switch (current_zone)
            {
                case alt_zones::Take_off:
                    output.phi_c = 0;
                    output.delta_a =  roll_hold(0.0,input.phi,input.p, params,input.Ts)
                    output.delta_t = params.max_t;
                    output.theta_c = 15.0*3.14/180.0;
                    if (input.h>= params.alt_toz)
                    {
                        ROS_DEBUG("climb");
                        current_zone = alt_zones::Climb;
                        ap_error_ = 0;
                        ap_integrator_ =0;
                        ap_differentiator_ = 0;

                    }
                    break;
                case alt_zones::Climb:
                    output.delta_t = params.max_t;                
                    output.theta_c = airspeed_with_pitch_hold(input.Va_c , input.va, params,input.Ts );
                    if (input.h >= input.h_c - params.alt_hz)
                    {   
                        ROS_DEBUG("Hold");
                        current_zone       = alt_zones::Altitud_Hold;
                        at_error_          = 0;
                        at_integrator_     = 0;
                        at_differentiator_ = 0;
                        a_error_           = 0;
                        a_integrator_      = 0;
                        a_differentiator_  = 0; 
                    }
                    else if(input.h <= params.alt_toz)
                    {
                        ROS_DEBUG("takeoff");
                        current_zone = alt_zones :: Take_off;
                    }
                    break;
                    case alt_zones::Descend:
                    output.delta_t = 0;
                    output.theta_c = airspeed_with_pitch_hold(input.Va_C, input.va,params,input.Ts);
                    if (input.h <= input.h_c + params.alt_hz)
                    {
                        ROS_DEBUG("hold");
                        current_zone       = alt_zones::Altitud_Hold;
                        at_error_          = 0;
                        at_integrator_     = 0;
                        at_differentiator_ = 0;
                        a_error_           = 0;
                        a_integrator_      = 0;
                        a_differentiator_  = 0;
                    }
                    break;
                    case alt_zones :: Altitud_Hold:
                    output.delta_t  = airspeed_with_throttle_hold(input.Va_c, input.va,params, input.Ts);
                    output.theta_c  = altitiude_hold(input.h_c,input.h, params, input.Ts);
                    if (input.h >= input.h_c + params.alt_hz)
                    {
                        ROS_DEBUG("Desend");
                        current_zone       = alt_zones::Descend;
                        ap_error_          = 0;
                        ap_integrator_     = 0;
                        ap_differentiator_ = 0;
                    }
                    else if(input.h <= input.h_c - params.alt_hz){
                        ROS_DEBUG("Climb");
                        current_zone       = alt_zones::Climb;
                        ap_error_          = 0;
                        ap_integrator_     = 0;
                        ap_differentiator_ = 0;
                    }
                    break;
                    default;
                    break;
            }
            output.current_zone = current_zone;
            output.delta_e      = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
        }

        float Stability::course_hold(float chi_c, float chi, float phi_ff, float r, const params_s &params, float Ts)
        {
            float error = chi_c - chi;
            c_integrator_ = c_integrator_ + (Ts/2.0)*(error + c_error_);
            float up = params.cp_kp.error;
            float ui = params.c_ki*c_integrator_;
            float ud = params.c_kd*r;
            
            float phi_c = sat(up+ui+ud+phi_ff,40.0*3.14/180.0 - 40.0*3.14/180.0);
            if(fabs(params.c_ki)>0.00001)
            {
                float  phi_c_unsat = up+ui+phi_ff;
                c_integrator_ = c_integrator_ +(Ts/params.c_ki)*(phi_c - phi_c_unsat);
            }

            c_error_ = error;

            return phi_c;
        }       
        float Stability::roll_hold(float phi_c, float phi, float p, const params_s &params, float Ts){
            float error = phi_c - phi;
            r_integrator = r_integrator + (Ts/2.0)*(error + r_error_):
            
            float up = params.r_kp*error;   
            float ui = params.r_ki*r_integrator;
            float ud = params.r_kd*p;

            float delta_a = set(up+ui+ud, params.max_a, -params.max_a);
            if(fbas(params.r_ki)>=0.0001)
            {
                float delta_a_unset = up+ui+ud;
                r_integrator = r_integrator + (Ts/params.r_ki)*(delta_a -delta_a_unset);
            }

            r_error_ = error;
            return delta_a;
        }
        float Stability::pitch_hold(float theta_c , float theta, float q , float ,const params_s &params , float Ts){
            float error = theta_c - theta; 

            float up = params_s.p_kp*error;
            float ui = params_s.p_ki*p_integrator_;
            float ud = params_s.p_kd*q;

            float delta_e =   set(params.trim_e/params.pwm_rad_e +up+ui+ud,params.max_e,-params.max_e);
            if(fabs(params.p_ki)>=0.00001)
            {
                float delta_e_unsat = params.trim_e/params.pwm_rad_e +up+ui+ud;
                p_integrator_ = p_integrator_ + (Ts/params.p_ki)*(delta_e - delta_e_unsat);
            }

            p_error_ = error;
            return delta_e;
        }
        float Stability::airspeed_with_pitch_hold(float Va_c, float Va, const params_s &params, float Ts){
            float error = Va_c - Va;
            
            ap_integrator_ = ap_integrator_ +(Ts/2.0)*(error+ap_error_);
            ap_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*ap_differentiator_+(2.0/(2.0*params.tau+Ts))*(error - ap_error_);
            
            float up = params.a_p_kp*error;
            float ui = params.a_p_ki*ap_integrator_;
            float ud = params.a_p_kd*ap_differentiator_;

            float theta_c = sat(up+ui+ud,20.0*3.14/180.0,-25.0*3.14/180.0);

            if(fabs(params.a_p_ki) >=0.00001){
                float theta_c_unset = up +ui +ud;
                ap_integrator_ = ap_differentiator_ +(Ts/params.a_p_ki)*(theta_c - theta_c_unset);
            }

            ap_error_ = error;
            return theta_c;
        }

        float Stability::airspeed_with_throttle_hold(float Va_c, float Va, const params_s &param, float Ts){
            
            float error = Va_c - Va;

            at_integrator_ = at_integrator_ + (Ts/2.0)*(error + at_error_);
            at_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*at_differentiator_ + (2.0/(2.0*params.tau + Ts))*(error - at_error_);
        
            float up =  params.a_t_kp*error;
            float ui =  params.a_t_ki*at_integrator_;
            float ud =  params.a_t_kd*at_differentiator_;

            float delta_t  =sat(params.trim_t +up +ui+ud,params.max_t,0);
            if(fabs(params.a_t_ki)>= 0.00001)
            {
                float delta_t_unset = params.trim_t  + up +ui +ud;
                at_integrator_ = at_integrator_ +(Ts/params.a_t_ki)*(delta_t - delta_t_unset);

            }
            at_error_ = error;
            return delta_t;
        }
        float Stability::altitiude_hold(float h_c, float h, const params_s &params , float Ts ){
            float error  = h_c - h ; 
            a_integrator_ = a_integrator_  + (Ts/2.0)*(error + a_error_);
            a_differentiator_ = (2.0*params.tau - Ts)*(2.0*params.tau +Ts)*a_differentiator_ + (2.0/(2.0*params.Tau + Ts))*(error - a_error_);

            float up = params.a_kp * error;
            float ui = params.a_ki *a_integrator_;
            float ud = params.a_kd *a_differentiator_;

            float theta_c = sat(up + ui +ud ,35.0*3.14/180.0, -35.0*3.14/180.0);
            if(fabs(params.a_ki) >= 0.00001)
            {
                float theta_c_unset = up +ui+ud;
                a_integrator_ = a_integrator_ + (Ts/params.a_ki)*(theta_c - theta_c_unset );
            } 

            at_error_ = error;
            return theta_c;
        }


        float Stability::sat(float value, float up_limit, float low_limit)
        {
            float rVal;
            if(value > up_limit)
                rVal = up_limit;
            else if (value < low_limit)
                rVal = low_limit;
            else 
                rVal = value;
            
            return rVal;
        }
} // namespace gazebo

