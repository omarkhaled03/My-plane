#include "Controller.h"

namespace fixedwing
{
    class Stability : public Controller
    {
        public: 
         Stability();
        private:
            virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output);
            alt_zones current_zone;

            float course_hold(float chi_c,float chi, float phi_ff, float r, const struct params_s &params, float Ts);
            float c_error_;
            float c_integrator_;

            float roll_hold(float phi_c, float phi, float p,const struct params_s &params, float Ts);
            float r_error_;
            float r_integrator_;

            float pitch_hold(float theta_c, float theta, float q, const struct params_s &params,float Ts );
            float p_error_;
            float p_integrator_;

            float airspeed_with_pitch_hold(float Va_c, float Va, const struct params_s &params, float Ts);
            float ap_error_;
            float ap_integrator_;
            float ap_differentiator_;
            
            float airspeed_with_throttle_hold(float Va_c ,float Va, const struct params_s &params, float Ts );
            float at_error_;
            float at_integrator_;
            float at_differentiator_;

            float altitiude_hold(float h_c, float h, const struct params_s &params, float Ts);
            float a_error_;
            float a_integrator_;
            float a_differentiator_;

            float sat(float value, float up_limit, float low_limit);
    };
    
} // namespace space
