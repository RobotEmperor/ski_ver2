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
#include "diana_msgs/ForceTorque.h"
#include "diana_msgs/CenterChange.h"
#include <atest_gui/command.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
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
        void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void forceTorqueDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg);


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

        // sensor value
        ros::Subscriber imu_data_sub;
        ros::Subscriber diana_force_torque_data_sub;
        double currentGyroX_gui, currentGyroY_gui, currentGyroZ_gui;
        double currentForceX_l_gui, currentForceY_l_gui, currentForceZ_l_gui;
        double currentForceX_r_gui, currentForceY_r_gui, currentForceZ_r_gui;
        double currentTorqueX_l_gui, currentTorqueY_l_gui, currentTorqueZ_l_gui;
        double currentTorqueX_r_gui, currentTorqueY_r_gui, currentTorqueZ_r_gui;



Q_SIGNALS:
        void rosShutdown();

private:
	int init_argc;
	char** init_argv;


};

}  // namespace atest_gui

#endif /* atest_gui_QNODE_HPP_ */
