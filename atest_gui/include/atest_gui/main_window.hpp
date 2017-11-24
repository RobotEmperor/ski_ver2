/**
 * @file /include/atest_gui/main_window.hpp
 *
 * @brief Qt based gui for atest_gui.
 *
 * @date November 2010
 **/
#ifndef atest_gui_MAIN_WINDOW_H
#define atest_gui_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <vector>
#include <math.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <QString>
#include "diana_msgs/BalanceParam.h"
#endif

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace atest_gui {

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();




	public Q_SLOTS:
	void graph_draw(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count);
	void graph_draw_updata(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ);
	void graph_draw_clean(QCustomPlot *ui_graph);
  void realtimeDataSlot();

  //slot
  void on_graph_stop_button_1_clicked();
  void on_graph_start_button_1_clicked();

  // control
	void center_change(int k);

	// off set module ////
	void on_read_data_clicked();


	void on_plus_button_clicked();
	void on_minus_button_clicked();
	void on_read_data_one_clicked();

	void on_save_onoff_clicked();
	void on_initial_button_clicked();
	void on_send_offset_value_clicked();


	void on_change_kinematics_1_1_clicked();
	void on_change_kinematics_2_clicked();
	void on_change_kinematics_3_clicked();
	void on_change_kinematics_4_clicked();
	void on_change_kinematics_4_1_clicked();
	void on_change_kinematics_5_1_clicked();
	void on_change_kinematics_6_1_clicked();
	void on_change_kinematics_7_1_clicked();
	void on_change_kinematics_8_1_clicked();
	void on_change_kinematics_9_1_clicked();
	void on_change_kinematics_10_1_clicked();
	void on_change_kinematics_11_1_clicked();
	void on_change_kinematics_12_1_clicked();
	void on_change_kinematics_13_1_clicked();
	void on_change_kinematics_100_1_clicked();
	void on_change_kinematics_1000_1_clicked();



	// module on off //////////////////////
	void on_pose_module_clicked();
	void on_initialize_module_2_clicked();

	void on_off_module_clicked();


	void on_base_module_1_clicked();
	void on_init_module_call_clicked();

	void on_offset_module_clicked();

	void on_motion_module_clicked();
	void on_initialize_module_clicked();
	void on_balance_on_clicked();
	void on_balance_off_clicked();

	void on_off_module_2_clicked();
	void on_off_module_3_clicked();
	void on_off_module_4_clicked();

	void parse_gain_data();



	private:
	Ui::MainWindowDesign ui;
	QNode qnode;

	std_msgs::Int32 pattern_msg; // pattern command msg
	std_msgs::String module_msg; // module on off command msg
	//<------------------------------------------------------------------- offset module -->
	// offset module msg //////////

	std_msgs::Bool save_onoff_msg_;
	std_msgs::Int8 joint_select_msg_;
	std_msgs::Int16MultiArray change_joint_value_msg_;
	std_msgs::Float64MultiArray offset_joint_value_msg_;
	atest_gui::command read_joint_value_msg_; // motor joint value read msg


	// offset_joint variables /////////

	// 독립 제어 용 변수
	QString offset_joint_str_; // 사용자에서 joint 번호를 string 으로 받아오는 변수
	int16_t offset_joint_int16_;   // string -> int16 으로 변환
	QString offset_change_value_str_;
	int16_t offset_change_value_int16_;

	int16_t new_offset_final_value_; // 최종적으로 다이나믹셀에 들어가는 오프셋 명령


	QString offset_DataToRad_str_;// 프레임워크 원점 조정을 위한 0-4095 에서 rad 으로 변환
	int16_t offset_DataToRad_int16_; // 프레임워크 원점 조정을 위한 0-4095 에서 rad 으로 변환 정수형으로 변수

	double offset_joint_value_[30];

	//<------------------------------------------------------------------------------------->

	//<------------------------------------------------------------------- base module -->
	std_msgs::String init_pose_msg; // init_pose command msg

	//<------------------------------------------------------------------- pose module -->
	std_msgs::Float64MultiArray desired_pose_msg; // desired_pose command msg

	//<------------------------------------------------------------------- motion module -->
	// balance on off msg
	diana_msgs::BalanceParam diana_balance_parameter_msg;

	// center_change msg
	diana_msgs::CenterChange center_change_msg;

	// graph variables
	QTimer *dataTimer;
	QFont legendFont;
	double key;


	double updating_duration;

	double cob_x_offset_m;
	double cob_y_offset_m;
	double foot_roll_gyro_p_gain;
	double foot_roll_gyro_d_gain;
	double foot_pitch_gyro_p_gain;
	double foot_pitch_gyro_d_gain;
};

}  // namespace atest_gui

#endif // atest_gui_MAIN_WINDOW_H
