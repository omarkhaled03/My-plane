#include <fixedwing/input.h>
#include <ros/ros.h>
#include <fixedwing/state.h>


double r_integral = 0;
double c_integral = 0;
double r_error = 0;
double c_error = 0;
double a_error = 0;
float phi_c = 50;
float a_integral = 0;
float kp = 0.5;
float ki = 0.001;
float kd = 0.01;
float integrator = 0;
double phi;
/*double update(double yref, double y, double dt, bool wrap_flag)
  {
    double error{yref - y};
    if(wrap_flag)
    {
      const double PI{3.14159265};
      while(error > PI)
        error -= (2 * PI);
      while(error <= -PI)
        error += (2 * PI);
    }
    integrateError(error, dt);
    differentiate(y, dt);

    double u_unsat{kp * error + ki * integrator - kd * ydot};
    double u_sat{saturate(u_unsat)};

    if(ki_ != 0)
      antiWindUp(u_unsat, u_sat, dt);

    return u_sat;
  }

double updateWithRate(double yref, double y, double ydot, double dt)
  {
    double error{yref - y};
    integrateError(error, dt);

    double u_unsat{kp_ * error + ki_ * integrator_ - kd_ * ydot};
    double u_sat{saturate(u_unsat)};

    if(ki_ != 0)
      antiWindUp(u_unsat, u_sat, dt);

    return u_sat;
  }

void integrateError(double error, double dt)
  {
    integrator_ += dt/2.0 * (error + eprev_);
    eprev_ = error;
  }

void differentiate(double y, double dt)
  {
    double a1 = (2.0 * sigma_ - dt) / (2.0 * sigma_ + dt);
    double a2 = 2.0 / (2.0 * sigma_ + dt);

    ydot_ = a1 * ydot_ + a2 * (y - yprev_);
    yprev_ = y;
  }

void antiWindUp(double u_unsat, double u, double dt)
  {
    integrator_ += dt/ki_ * (u - u_unsat);
  }

double saturate(double u)
  {
    double usat;
    if(u >= limit_h_)
      usat = limit_h_;
    else if(u <= limit_l_)
      usat = limit_l_;
    else
      usat = u;
    return usat;
  }*/


/*float altitiude_hold(float h_c, float h,float Ts){
  float error = h_c - h;
  float error = + phi_ff
}*/
float roll_hold(float kp,float kd, float phi_c, float phi,float p, float Ts){
  float error = phi_c - phi;
  r_integral  = r_integral + (Ts/2.0)*(error+ r_error);
  float ep    = kp*error;
  float ed     = p*kd;

  float delta_a = ep+ed;
  return  delta_a;
}
/*float cruse_control(float Kp,float chi_c, float chi, float phi_ff,  float Ts)
{
  float error = chi_c - chi;
  c_integral  = c_integral + (Ts/2.0)*(error + c_error);
  float ep    = error*Kp;
  float ei    = error*c_integral; 
  float phi_c    = ep + ei ; 
  c_error = error;
  return phi_c;
}*/

void StateCallback(const fixedwing::statePtr &state){
  phi = state->phi;
}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<fixedwing::input>("fixedwing/output_command", 100);
  ros::Subscriber subscriber = nh.subscribe("/fixedwing/state",100 ,StateCallback);
  ros::Publisher publishere = nh.advertise<fixedwing::input>("fixedwing/output_command", 100);

  fixedwing::input msg;
  fixedwing::state state;

  //float phi_c = 0;
  

  while (ros::ok()) {
    msg.t = 1.5;
    msg.e = 4;
    //phi_c =  cruse_control(1.25,20, state.chi , 5,  0.05);
    msg.a =  roll_hold(0.4743,0.1584, phi_c, phi,state.p, 0.05);
    msg.state = true;
    ROS_INFO("The value of minus:%f",phi_c-phi);
    ROS_INFO("The value of Phi:%f",phi);
    ROS_INFO("The value of Phi_c:%f",phi_c);
    publisher.publish(msg);
    ros::spinOnce();
  }

  return 0;
}