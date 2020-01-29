#ifdef SIMULATION_POSTIONCONTROLLER_H
#define SIMULATION_POSTIONCONTROLLER_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
/*
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
*/
#include <geometry_msgs/WrenchStamped.h>
/*
std_msgs/Header header
geometry_msgs/Wrench wrench
*/
#include <geometry_msgs/Point.h>
/*
float64 x
float64 y
float64 z
*/
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

// constant
#define TOTAL_DOF 7
#define SIM_DT 0.01 //unit: sec  10ms
#define PI 3.14159265359
//Macro
#define deg2rad(deg) ((deg) * PI / 180.0)
#define rad2deg(rad) ((rad) / PI * 180.0)

const std::string JOINT_NAME[TOTALDOF]={"Franka_joint1","Franka_joint2","Franka_joint3","Franka_joint4"
                                        "Franka_joint5","Franka_joint6","Franka_joint7"};
class posController
{
public:
  //contructor
  posController(ros::NodeHandle nh,double hz_);
  //destructor
  ~posController();

  //-------------- Callback Functions for subscribing topics --------------

  //joint Callback function
  void jointCallback(sensor_msgs::JointStateConstPtr &msg);
  //simulation state callback 0: stopped 1:running 2: paused
  void simStateCallback(std_msgs::Int32ConstPtr &msg);
  //simulation time callback
  void simTimeCallback(std_msgs::Float32ConstPtr &msg);
  //simulation step done callback
  void simStepDoneCallback(std_msgs::BoolConstPtr &msg);

  //-------------- vrep interface functions --------------

  void vrepStart();
  void vrepStop();
  void vrepStepTrigger();
  void vrepEnableSyncMode();
  void readVrep();
  void writeVrep();

  //-------------- position controller functions --------------

  void setDesiredPos(std::vector<float> des_q); //how about Eigen::Vectorxd
  void setExecTime(float t);
  void compute();
  void wait();


private:
  //use underscore at the end to represent member variables in class

  //-------------- Publisher Variables --------------

  ros::Publisher joint_set_pub_;

  ros::Publisher sim_start_pub_;
  //ros::Publisher simPausePub_; //if u need it, use it
  ros::Publisher sim_stop_pub_;

  ros::Publisher sim_enable_sync_mode_pub_;
  ros::Publisher trigger_next_step_pub;

  //-------------- Publisher Variables --------------

  ros::Subscriber joint_state_sub_;
  ros::Subscriber sim_step_done_sub_;
  ros::Subscriber sim_state_sub_;
  ros::Subscriber sim_time_sub_;

  //-------------- Time or Simulation Variables --------------
  bool sim_step_done_;
  float sim_time_;
  int tick_;
  ros::Rate rate_;

  //-------------- Joint Variables --------------

  Eigen::VectorXd q_; //current joint position
  Eigen::VectorXd q_dot_; //current joint velocity

  sensor_msgs::JointState joint_cmd_;

  Eigen::VectorXd desired_q_; // controller input from cubic spline
  Eigen::VectorXd target_q_;  // target position

  Eigen::VectorXd init_q_;
  Eigen::VectorXd goal_q_err_;

  bool is_first_run_;
  double start_time_;
  double current_time_;
  double final_time_;
  int vrep_sim_status_;
  float exec_time_;
};

static double cubicSpline(double time,
                          double time_0,
                          double time_f,
                          double x_0,
                          dobule x_f,
                          double x_dot_0
                          double x_dot_f)
//a0 + a1*t + a2*t^2 + a3*t^2 = u(t) 4 unknown 4 eqn(4 boundary condition)
{
  double x_t;
  if ( time < time_0 )
    x_t = x_0;
  else if (time > time_f )
    x_t = x_f;
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double a0 = x_0;
    double a1 = x_dot_0;
    double a2 = 3.0/total_time/total_time*(x_f - x_0) - 2.0/total_time*x_dot_0 - 1.0/total_time*x_dot_f;
    double a3 = -2.0/total_time/total_time/total_time*(x_f - x_0) + 1.0/total_time/total_time*(x_dot_f + x_dot_0);

    x_t = a0 + a1*elapsed_time + a2*pow(elapsed_time,2) + a3*pow(elapsed_time,3);
  }
  return x_t;
}

#endif










}
