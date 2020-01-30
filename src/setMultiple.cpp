#include "manipulator_study/my_posController.hpp"


int main(int argc,char** argv)
{
  ros::init(argc,argv,"ROS_vrep_control");
  ros::NodeHandle nh("~"); //private node;

  double control_hz = 200;
  posController pos_controller(nh,control_hz);

  sleep(1);
  pos_controller.vrepStart();
  sleep(1);
  pos_controller.vrepEnableSyncMode();
  sleep(1);
  while (ros::ok())
  {
    pos_controller.getTarget(nh);
    pos_controller.readVrep();
    pos_controller.compute();
    pos_controller.writeVrep();
    pos_controller.wait();
  }
  return 0;
}
