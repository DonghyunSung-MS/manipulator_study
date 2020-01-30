#include "manipulator_study/my_posController.hpp"


int main(int argc,char** argv)
{
  ros::init(argc,argv,"ROS_vrep_control");
  ros::NodeHandle nh("~"); //private node;

  double control_hz = 200;
  posController pos_controller(nh,control_hz);

  float joint_value[TOTAL_DOF];
  for(size_t i=0;i<TOTAL_DOF;i++)
  {
    nh.getParam("j"+ToString(i+1),joint_value[i]);
    //ROS_INFO("%0.6f",joint_value[i]);
  }

  float exec_time;
  nh.getParam("exec_time",exec_time);

  std::vector<float> desired_q;
  for(size_t i=0;i<TOTAL_DOF;i++)
  {
    desired_q.push_back(joint_value[i]);
  }
  pos_controller.setDesiredPos(desired_q);
  pos_controller.setExecTime(exec_time);

  sleep(1);
  pos_controller.vrepStart();
  sleep(1);
  pos_controller.vrepEnableSyncMode();
  sleep(1);
  while (ros::ok())
  {
    pos_controller.readVrep();
    pos_controller.compute();
    pos_controller.writeVrep();
    pos_controller.wait();
  }
  return 0;
}
