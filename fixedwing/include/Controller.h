#include <ros/ros.h>
#include <fixedwing/command.h>
#include <fixedwing/state.h>
#include <fixedwing/controller_command.h>
#include <fixedwing/controller_internals.h>

#include <dynamic_reconfigure/server.h>
#include <fixedwing/ControllerConfig.h>

namespace fixedwing {
    enum class alt_zones{
        Take_off,
        Climb,
        Descend,
        Altitud_Hold
    };

    class Controller
    {
        public:
            Controller();
            float spin();
        protected:
          struct input_s
            {
                float Ts;               /** time step */
                float h;                /** altitude */
                float va;               /** airspeed */
                float phi;              /** roll angle */
                float theta;            /** pitch angle */
                float chi;              /** course angle */
                float p;                /** body frame roll rate */
                float q;                /** body frame pitch rate */
                float r;                /** body frame yaw rate */
                float Va_c;             /** commanded airspeed (m/s) */
                float h_c;              /** commanded altitude (m) */
                float chi_c;            /** commanded course (rad) */
                float phi_ff;           /** feed forward term for orbits (rad) */
            };
          struct output_s
            {
            float theta_c;
            float delta_e;
            float phi_c;
            float delta_a;
            float delta_r;
            float delta_t;
            alt_zones current_zone;
            };

          struct params_s
            {   
                double alt_hz;           /**< altitude hold zone */
                double alt_toz;          /**< altitude takeoff zone */
                double tau;
                double c_kp;
                double c_kd;
                double c_ki;
                double r_kp;
                double r_kd;
                double r_ki;
                double p_kp;
                double p_kd;
                double p_ki;
                double p_ff;
                double a_p_kp;
                double a_p_kd;
                double a_p_ki;
                double a_t_kp;
                double a_t_kd;
                double a_t_ki;
                double a_kp;
                double a_kd;
                double a_ki;
                double b_kp;
                double b_kd;
                double b_ki;
                double trim_e;
                double trim_a;
                double trim_r;
                double trim_t;
                double max_e;
                double max_a;
                double max_r;
                double max_t;
                double pwm_rad_e;
                double pwm_rad_a;
                double pwm_rad_r;
            };

        virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;
        private:
         ros::NodeHandle nh_;
         ros::NodeHandle nh_private_;
         ros::Subscriber vehicle_state_sub_;
         ros::Subscriber controller_commands_sub_;
         ros::Publisher  actuators_pub_;
         ros::Publisher  internal_pub_;
         ros::Timer      act_pub_timer_;

         struct params_s params_;

         fixedwing::controller_command controller_command_;
         fixedwing::state UAV_state_;

         void vehicle_state_callback(const fixedwing::stateConstPtr &msg);
         void controller_commands_callback(const fixedwing::controller_commandConstPtr &msg);
         bool command_recieved_;
        dynamic_reconfigure::Server<fixedwing::ControllerConfig> server_;
        dynamic_reconfigure::Server<fixedwing::ControllerConfig>::CallbackType func_;

        void reconfigure_callback(fixedwing::ControllerConfig &config, uint32_t level);

        void convert_to_pwm(struct output_s &output);

        void actuator_controls_publish(const ros::TimerEvent &);
    };

}