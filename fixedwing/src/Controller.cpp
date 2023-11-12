#include "Controller.h"
#include "Stability.h"
namespace fixedwing
{
    Controller::Controller()
        nh_(),
        nh_private_("~")
    {
        vehicle_state_sub_ = nh_.subscribe("State", 10, &Controller::vehicle_state_callback, this);
        controller_commands_sub_ = nh_.subscribe("controller_command", 10, &Controller::controller_commands_callback,
                                                 this);

        memset(&UAV_state_, 0, sizeof(UAV_state_));
        memset(&controller_command_, 0, sizeof(controller_command_));

        nh_private_.param<double>("TRIM_E", params_.trim_e, 0.0);
        nh_private_.param<double>("TRIM_A", params_.trim_a, 0.0);
        nh_private_.param<double>("TRIM_R", params_.trim_r, 0.0);
        nh_private_.param<double>("TRIM_T", params_.trim_t, 0.6);
        nh_private_.param<double>("PWM_RAD_E", params_.pwm_rad_e, 1.0);
        nh_private_.param<double>("PWM_RAD_A", params_.pwm_rad_a, 1.0);
        nh_private_.param<double>("PWM_RAD_R", params_.pwm_rad_r, 1.0);
        nh_private_.param<double>("ALT_TOZ", params_.alt_toz, 20.0);
        nh_private_.param<double>("ALT_HZ", params_.alt_hz, 10.0);
        nh_private_.param<double>("TAU", params_.tau, 5.0);
        nh_private_.param<double>("COURSE_KP", params_.c_kp, 0.7329);
        nh_private_.param<double>("COURSE_KD", params_.c_kd, 0.0);
        nh_private_.param<double>("COURSE_KI", params_.c_ki, 0.0);
        nh_private_.param<double>("ROLL_KP", params_.r_kp, 1.2855);
        nh_private_.param<double>("ROLL_KD", params_.r_kd, -0.325);
        nh_private_.param<double>("ROLL_KI", params_.r_ki, 0.0); // 0.10f);
        nh_private_.param<double>("PITCH_KP", params_.p_kp, 1.0);
        nh_private_.param<double>("PITCH_KD", params_.p_kd, -0.17);
        nh_private_.param<double>("PITCH_KI", params_.p_ki, 0.0);
        nh_private_.param<double>("PITCH_FF", params_.p_ff, 0.0);
        nh_private_.param<double>("AS_PITCH_KP", params_.a_p_kp, -0.0713);
        nh_private_.param<double>("AS_PITCH_KD", params_.a_p_kd, -0.0635);
        nh_private_.param<double>("AS_PITCH_KI", params_.a_p_ki, 0.0);
        nh_private_.param<double>("AS_THR_KP", params_.a_t_kp, 3.2);
        nh_private_.param<double>("AS_THR_KD", params_.a_t_kd, 0.0);
        nh_private_.param<double>("AS_THR_KI", params_.a_t_ki, 0.0);
        nh_private_.param<double>("ALT_KP", params_.a_kp, 0.045);
        nh_private_.param<double>("ALT_KD", params_.a_kd, 0.0);
        nh_private_.param<double>("ALT_KI", params_.a_ki, 0.01);
        nh_private_.param<double>("BETA_KP", params_.b_kp, -0.1164);
        nh_private_.param<double>("BETA_KD", params_.b_kd, 0.0);
        nh_private_.param<double>("BETA_KI", params_.b_ki, -0.0037111);
        nh_private_.param<double>("MAX_E", params_.max_e, 0.610);
        nh_private_.param<double>("MAX_A", params_.max_a, 0.523);
        nh_private_.param<double>("MAX_R", params_.max_r, 0.523);
        nh_private_.param<double>("MAX_T", params_.max_t, 1.0);

        func_ = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
        server_.setCallback(func_);

        actuators_pub_ = nh_.advertise<fixedwing::command>("command", 10);
        internal_pub_ = nh_.advertise<fixedwing::controller_internals>("controller_inners", 10);
        act_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / 100.0), &Controller::actuator_controls_publish, this);

        command_recieved_ = false;
    }
    void Controller::vehicle_state_callback(const fixedwing::stateConstPtr &msg)
    {
        UAV_state_ = *msg;
    }

    void Controller::controller_commands_callback(const fixedwing::controller_commandConstPtr &msg)
    {
        controller_command_ = *msg;
    }
    void Controller::reconfigure_callback(fixedwing::ControllerConfig &config, uint32_t level)
    {
        params_.trim_e = config.TRIM_E;
        params_.trim_a = config.TRIM_A;
        params_.trim_r = config.TRIM_R;
        params_.trim_t = config.TRIM_T;

        params_.c_kp = config.COURSE_KP;
        params_.c_kd = config.COURSE_KD;
        params_.c_ki = config.COURSE_KI;

        params_.r_kp = config.ROLL_KP;
        params_.r_kd = config.ROLL_KD;
        params_.r_ki = config.ROLL_KI;
        params_.p_kp = config.PITCH_KP;
        params_.p_kd = config.PITCH_KD;
        params_.p_ki = config.PITCH_KI;
        params_.p_ff = config.PITCH_FF;

        params_.a_p_kp = config.AS_PITCH_KP;
        params_.a_p_kd = config.AS_PITCH_KD;
        params_.a_p_ki = config.AS_PITCH_KI;

        params_.a_t_kp = config.AS_THR_KP;
        params_.a_t_kd = config.AS_THR_KD;
        params_.a_t_ki = config.AS_THR_KI;

        params_.a_kp = config.ALT_KP;
        params_.a_kd = config.ALT_KD;
        params_.a_ki = config.ALT_KI;

        params_.b_kp = config.BETA_KP;
        params_.b_kd = config.BETA_KD;
        params_.b_ki = config.BETA_KI;
    }

    void Controller::convert_to_pwm(Controller::output_s &output)
    {
        output.delta_e = output.delta_e * params_.pwm_rad_e;
        output.delta_a = output.delta_a * params_.pwm_rad_a;
        output.delta_r = output.delta_r * params_.pwm_rad_r;
    }
    void Controller::actuator_controls_publish(const ros::TimerEvent &)
    {
        struct input_s input;
        input.h = UAV_state_.postion[2];
        input.va = UAV_state_.Va;
        input.phi = UAV_state_.phi;
        input.theta = UAV_state_.theta;
        input.p = UAV_state_.p;
        input.q = UAV_state_.q;
        input.r = UAV_state_.r;
        input.Va_c = controller_command_.Va_c;
        input.h_c = controller_command_.h_c;
        input.chi_c = controller_command_.chi_c;
        input.phi_ff = controller_command_.phi_ff;
        input.Ts = 0.01f;

        struct output_s output;
        if (command_recieved_ == true)
        {
            control(params_, input, output);
            convert_to_pwm(output);

            fixedwing::command actuators;

            actuators.ignore = 0;
            actuators.mode = fixedwing::command::MODE_PASS_THROUGH;
            actuators.x = output.delta_a; //(isfinite(output.delta_a)) ? output.delta_a : 0.0f;
            actuators.y = output.delta_e; //(isfinite(output.delta_e)) ? output.delta_e : 0.0f;
            actuators.z = output.delta_r; //(isfinite(output.delta_r)) ? output.delta_r : 0.0f;
            actuators.F = output.delta_t; //(isfinite(output.delta_t)) ? output.delta_t : 0.0f;

            actuators_pub_.publish(actuators);

            if (internals_pub_.getNumSubscribers() > 0)
            {
                fixedwing::controller_internals inners;
                inners.phi_c = output.phi_c;
                inners.theta_c = output.theta_c;
                switch (output.current_zone)
                {
                case alt_zones::Take_off:
                    inners.alt_zone = inners.ZONE_TAKE_OFF;
                    break;
                case alt_zones::Climb:
                    inners.alt_zone = inners.ZONE_CLIMB;
                    break;
                case alt_zones::Descend:
                    inners.alt_zone = inners.ZONE_DESEND;
                    break;
                case alt_zones::Altitud_Hold:
                    inners.alt_zone = inners.ZONE_ALTITUDE_HOLD;
                    break;
                default:
                    break;
                }
                inners.aux_valid = false;
                internals_pub_.publish(inners);
            }
        }
    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "controller");
        fixedwing::Controller *cont = new fixedwing::Stability();

        ros::spin();

        return 0;
    }
}