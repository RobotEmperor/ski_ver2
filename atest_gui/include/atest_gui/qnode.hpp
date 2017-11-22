/**
 * @file /include/atest_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef atest_gui_QNODE_HPP_
#define atest_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Bool.h>
#include <atest_gui/dynamixel_info.h>
#include "diana_msgs/BalanceParam.h"
#include "diana_msgs/CenterChange.h"
#include <atest_gui/command.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <ros/package.h>

#include <fstream>
#include <iostream>
#endif

#define all_DXL 23
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atest_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
        bool init();
        void run();



        ros::Publisher pattern_pub; // motion_pattern 입력
        ros::Publisher module_on_off; // 모듈 on off

        // off set module topic //
        ros::Publisher joint_select_pub;  // 오프셋 적용 하고자 하는 조인트
        ros::Publisher change_joint_value_pub; // 선택된 조인트 값 변경
        ros::ServiceClient read_joint_value_client;

        ros::Publisher save_onoff_pub; // motion on off

        ros::Publisher init_pose_pub; //

        ros::Publisher offset_joint_value_pub; // offset_joint_value

        // pose module topic //
        ros::Publisher desired_pose_pub;

        // balance on off topic //
        ros::Publisher diana_balance_parameter_pub; // 모듈 on off

        // center change topic //
        ros::Publisher center_change_pub;



Q_SIGNALS:
        void rosShutdown();

private:
	int init_argc;
	char** init_argv;


};

}  // namespace atest_gui

#endif /* atest_gui_QNODE_HPP_ */
