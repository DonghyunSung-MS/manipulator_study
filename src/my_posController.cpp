#include "manipulator_study/my_posController.hpp"

//contructor
posController::posController(ros::NodeHandle nh_,double hz_):rate_(hz_)
{
  is_first_run_ = true;
  tick_ = 0;
  sim_step_done_=false;
  sim_time_ = 0.0f;
  //initialize state Eigen::VectorXd
  q_.resize(TOTAL_DOF);
  q_.setZero();

  q_dot_.resize(TOTAL_DOF);
  q_dot_.setZero();

  desired_q_.resize(TOTAL_DOF);
  desired_q_.setZero();

  target_q_.resize(TOTAL_DOF);
  target_q_.setZero();

  goal_q_err_.resize(TOTAL_DOF);
  goal_q_err_.setZero();

  //set Joint initial pose
  init_q_.resize(TOTAL_DOF);
  init_q_.setZero();
  init_q_(5) = deg2rad(30); //you can customize your initial position
  init_q_(6) = deg2rad(45);

  //sensor_msgs::JointState
  joint_cmd_.name.resize(TOTAL_DOF);
  joint_cmd_.position.resize(TOTAL_DOF);

  for(size_t i=0; i<TOTAL_DOF; i++)
  {
    joint_cmd_.name[i] = JOINT_NAME[i];
  }
  //-------------- Publisher and Subscriber Variables --------------

  //Publisher --> NodeHandle_object.advertise<datatype>("node name",queue size)
  joint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/Franka/joint_set",1);
  sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation",5);
  sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation",5);
  sim_enable_sync_mode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode",5);
  trigger_next_step_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep",100);

  //Subscriber --> NodeHandle_object.subscribe("node name",queue size,callback_function)
  joint_state_sub_ = nh_.subscribe("/Franka/joint_states",100, &posController::jointStateCallback,this);
  sim_step_done_sub_ = nh_.subscribe("/simulationStepDone",100, &posController::simStepDoneCallback,this);
  sim_state_sub_ = nh_.subscribe("/simulationState",100, &posController::simStateCallback,this);
  sim_time_sub_ = nh_.subscribe("/simulationTime",100, &posController::simTimeCallback,this);
}

//destructor
posController::~posController()
{

}

//-------------- Callback Functions for subscribing topics --------------

//joint Callback function

void posController::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if(msg->name.size() == TOTAL_DOF)
  {
    for(size_t i=0;i<TOTAL_DOF;i++)
    {
      q_[i] = msg->position[i];
      q_dot_[i] = msg->velocity[i];
    }
  }
  else
  {
    ROS_ERROR("Controller's total Dof and JointStates from VREP is not same size!!");
  }
}

//simulation state callback 0: stopped 1:running 2: paused
void posController::simStateCallback(const std_msgs::Int32ConstPtr& msg)
{
  vrep_sim_status_ = msg->data;
}
//simulation time callback
void posController::simTimeCallback(const std_msgs::Float32ConstPtr& msg)
{
  sim_time_ = msg->data;
  tick_ = (sim_time_)/SIM_DT; // sec per period
}
//simulation step done callback
void posController::simStepDoneCallback(const std_msgs::BoolConstPtr& msg)
{
  sim_step_done_ = msg->data;
}

//-------------- vrep interface functions --------------

void posController::vrepStart()
{
  ROS_INFO("Starting V-REP Simulation");
  std_msgs::Bool msg;
  msg.data = true;
  sim_start_pub_.publish(msg);
}
void posController::vrepStop()
{
  ROS_INFO("Stopping V-REP Simulation");
  std_msgs::Bool msg;
  msg.data = true;
  sim_stop_pub_.publish(msg);
}
void posController::vrepStepTrigger()
{
  std_msgs::Bool msg;
  msg.data = true;
  trigger_next_step_pub_.publish(msg);
}
void posController::vrepEnableSyncMode()
{
  ROS_INFO("Sync Mode On");
  std_msgs::Bool msg;
  msg.data = true;
  trigger_next_step_pub_.publish(msg);
}
void posController::readVrep()
{
  ros::spinOnce();
}
void posController::writeVrep()
{
  for(size_t i=0;i<TOTAL_DOF;i++)
  {
    joint_cmd_.position[i] = target_q_(i);
    goal_q_err_(i) = abs(desired_q_(i) - q_(i));
  }
  joint_set_pub_.publish(joint_cmd_);
  vrepStepTrigger();
}

//-------------- position controller functions --------------

void posController::setDesiredPos(std::vector<float> des_q)
{
  if (des_q.size() == TOTAL_DOF)
  {
    for(size_t i=0;i<TOTAL_DOF;i++)
    {
      desired_q_(i) = deg2rad(des_q.at(i));
    }
  }
  else
  {
    for(size_t i=0;i<TOTAL_DOF;i++)
    {
      desired_q_(i) = deg2rad(-45);
    }
    desired_q_(5) = deg2rad(45);
    desired_q_(6) = deg2rad(45);
    desired_q_(7) = deg2rad(45);
  }
}
void posController::setExecTime(float t)
{
  exec_time_ = t;
}
void posController::compute()
{
  if(is_first_run_)
  {
    start_time_ = tick_*SIM_DT;
    final_time_ = start_time_ + exec_time_;
    is_first_run_ = false;
  }
  current_time_ = tick_*SIM_DT;
  for(size_t i=0;i<TOTAL_DOF;i++)
  {
    target_q_(i)=cubicSpline(current_time_, start_time_, final_time_, init_q_(i), desired_q_(i),0.0,0.0);
  }
}
void posController::wait()
{
  while(ros::ok() && !sim_step_done_)
  {
    ros::spinOnce();
  }
  sim_step_done_ = false;
  rate_.sleep();
}

void posController::getTarget(ros::NodeHandle nh_)
{

  if (final_time_ < current_time_)
  {
    float joint_value[TOTAL_DOF];
    for(size_t i=0;i<TOTAL_DOF;i++)
    {
      nh_.getParam("j"+ToString(i+1),joint_value[i]);
      //ROS_INFO("%0.6f",joint_value[i]);
    }
    nh_.getParam("exec_time",exec_time_);

    for(size_t i=0;i<TOTAL_DOF;i++)
    {
      init_q_(i)= desired_q_(i);
      desired_q_(i) = deg2rad(joint_value[i]);
    }
    is_first_run_ = true;

  }
}
