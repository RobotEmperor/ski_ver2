#include "robotis_math/robotis_math.h"


namespace base_module_state
{

class BaseModuleState
{
public:
  BaseModuleState();
  ~BaseModuleState();

  bool is_moving_state;
  double mov_time_state;
  int MAX_JOINT_ID_STATE;

  Eigen::MatrixXd joint_ini_pose_state;
  Eigen::MatrixXd joint_ini_pose_goal;

};

}

