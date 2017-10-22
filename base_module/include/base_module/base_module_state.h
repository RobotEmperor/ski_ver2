#include "robotis_math/robotis_math.h"


namespace base_module_state
{

class BaseModuleState
{
public:
  BaseModuleState();
  ~BaseModuleState();

  bool is_moving_state;

  int MAX_JOINT_ID_STATE;

  int cnt_state; // counter number

  double mov_time_state; // movement time
  double smp_time_state; // sampling time

  int all_time_steps_state; // all time steps of movement time

  Eigen::MatrixXd joint_ini_pose_state;

};

}

