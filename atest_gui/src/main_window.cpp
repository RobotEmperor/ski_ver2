/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <stdio.h>
#include "../include/atest_gui/main_window.hpp"


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace atest_gui {
using namespace std;
using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent)
, qnode(argc,argv)
{
	qnode.init();
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	legendFont = font();  // start out with MainWindow's font..
	graph_draw(ui.sensor_value_plot_1, "Angular velocity", "Rad/s", -4, 4 , 10);
	graph_draw(ui.sensor_value_plot_fl, "Force Sensor Left leg", "N", -3, 3, 10);
	graph_draw(ui.sensor_value_plot_fr, "Force Sensor Right leg", "Nm", -3, 3, 10);
	graph_draw(ui.sensor_value_plot_tl, "Torque Sensor Left leg", "N", -3, 3, 10);
	graph_draw(ui.sensor_value_plot_tr, "Torque Sensor Right leg", "Nm", -3, 3, 10);

	dataTimer = new QTimer(this);
	connect(dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
	dataTimer->start(0); // Interval 0 means to refresh as fast as possible

	connect(ui.slider_center_change, SIGNAL(valueChanged(int)), this, SLOT(center_change(int)));
	connect(ui.slider_edge_change, SIGNAL(valueChanged(int)), this, SLOT(edge_change(int)));

	for(int i=0;i<30;i++)
	{
		offset_joint_value_[i] = 0;
	}
	// gain value //
	updating_duration = 0.0;

	cob_x_offset_m = 0.0;
	cob_y_offset_m = 0.0;
	foot_roll_gyro_p_gain = 0.0;
	foot_roll_gyro_d_gain = 0.0;
	foot_pitch_gyro_p_gain = 0.0;
	foot_pitch_gyro_d_gain = 0.0;
}

MainWindow::~MainWindow()
{

}
/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/
void MainWindow::realtimeDataSlot()
{
	static QTime time(QTime::currentTime());
	// calculate two new data points:
	key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
	static double lastPointKey = 0;

	if (key-lastPointKey > 0.002) // at most add point every 2 ms
	{
		graph_draw_updata(ui.sensor_value_plot_1, qnode.currentGyroX_gui, qnode.currentGyroY_gui, qnode.currentGyroZ_gui);
		graph_draw_updata(ui.sensor_value_plot_fl, qnode.currentForceX_l_gui, qnode.currentForceY_l_gui, qnode.currentForceZ_l_gui);
		graph_draw_updata(ui.sensor_value_plot_fr, qnode.currentForceX_r_gui, qnode.currentForceY_r_gui, qnode.currentForceZ_r_gui);
		graph_draw_updata(ui.sensor_value_plot_tl, qnode.currentTorqueX_l_gui, qnode.currentTorqueY_l_gui, qnode.currentTorqueZ_l_gui);
		graph_draw_updata(ui.sensor_value_plot_tr, qnode.currentTorqueX_r_gui, qnode.currentTorqueY_r_gui, qnode.currentTorqueZ_r_gui);
		lastPointKey = key;
	}
	// make key axis range scroll with the data (at a constant range size of 8):
	graph_draw_clean(ui.sensor_value_plot_1);
	graph_draw_clean(ui.sensor_value_plot_fl);
	graph_draw_clean(ui.sensor_value_plot_fr);
	graph_draw_clean(ui.sensor_value_plot_tl);
	graph_draw_clean(ui.sensor_value_plot_tr);
}

void MainWindow::graph_draw(QCustomPlot *ui_graph, const QString title, const QString unit, int min_value, int max_value, int tick_count)
{
	ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
	ui_graph->legend->setVisible(true);
	legendFont.setPointSize(9); // and make a bit smaller for legend
	ui_graph->legend->setFont(legendFont);
	ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));
	// by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
	ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

	ui_graph->plotLayout()->insertRow(0);
	ui_graph->plotLayout()->addElement(0, 0, new QCPTextElement(ui_graph, title, QFont("sans", 12, QFont::Bold)));


	ui_graph->addGraph();
	ui_graph->graph(0)->setPen(QPen(QColor(40, 110, 255)));
	ui_graph->graph(0)->setName("X");
	ui_graph->addGraph();
	ui_graph->graph(1)->setPen(QPen(QColor(255, 110, 40)));
	ui_graph->graph(1)->setName("Y");
	ui_graph->addGraph();
	ui_graph->graph(2)->setPen(QPen(QColor(0, 0, 0)));
	ui_graph->graph(2)->setName("Z");
	ui_graph->xAxis->setLabel("Time(s)");
	ui_graph->yAxis->setLabel(unit);

	QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
	timeTicker->setTimeFormat("%s");
	timeTicker->setFieldWidth(timeTicker->tuSeconds,1);
	timeTicker->setTickCount(tick_count);
	ui_graph->xAxis->setTicker(timeTicker);

	ui_graph->axisRect()->setupFullAxesBox();
	ui_graph->yAxis->setRange(min_value, max_value);
}
void MainWindow::graph_draw_updata(QCustomPlot *ui_graph, double valueX, double valueY, double valueZ)
{
	// add data to lines:
	ui_graph->graph(0)->addData(key, valueX);
	ui_graph->graph(1)->addData(key, valueY);
	ui_graph->graph(2)->addData(key, valueZ);

	// ui->customPlot->graph(1)->addData(key, qCos(key)+qrand()/(double)RAND_MAX*0.5*qSin(key/0.4364));
	// rescale value (vertical) axis to fit the current data:
	ui_graph->graph(0)->rescaleValueAxis(true);
	ui_graph->graph(1)->rescaleValueAxis(true);
	ui_graph->graph(2)->rescaleValueAxis(true);
}
void MainWindow::graph_draw_clean(QCustomPlot *ui_graph)
{
	ui_graph->xAxis->setRange(key, 8, Qt::AlignRight);
	ui_graph->replot();
}
void MainWindow::on_graph_stop_button_1_clicked()
{
	dataTimer->stop();
}
void MainWindow::on_graph_start_button_1_clicked()
{
	dataTimer->start(0);
}

// control page gui /////////////////////////////////////////////////////////
void MainWindow::on_l_hip_pitch_button_clicked()  //11
{
	ui.joint_name_edit->setText("l_hip_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[11]));
}
void MainWindow::on_l_hip_roll_button_clicked()  //13
{
	ui.joint_name_edit->setText("l_hip_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[13]));
}
void MainWindow::on_l_hip_yaw_button_clicked()  //15
{
	ui.joint_name_edit->setText("l_hip_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[15]));
}
void MainWindow::on_l_knee_pitch_button_clicked()  //17
{
	ui.joint_name_edit->setText("l_knee_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[17]));
}
void MainWindow::on_l_ankle_pitch_button_clicked()  // 19
{
	ui.joint_name_edit->setText("l_ankle_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[19]));
}
void MainWindow::on_l_ankle_roll_button_clicked()  // 21
{
	ui.joint_name_edit->setText("l_ankle_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[21]));
}
void MainWindow::on_r_hip_pitch_button_clicked()  // 12
{
	ui.joint_name_edit->setText("r_hip_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[12]));
}
void MainWindow::on_r_hip_roll_button_clicked()  // 14
{
	ui.joint_name_edit->setText("r_hip_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[14]));
}
void MainWindow::on_r_hip_yaw_button_clicked()  // 16
{
	ui.joint_name_edit->setText("r_hip_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[16]));
}
void MainWindow::on_r_knee_pitch_button_clicked()  // 18
{
	ui.joint_name_edit->setText("r_knee_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[18]));
}
void MainWindow::on_r_ankle_pitch_button_clicked()  // 20
{
	ui.joint_name_edit->setText("r_ankle_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[20]));
}
void MainWindow::on_r_ankle_roll_button_clicked()  // 22
{
	ui.joint_name_edit->setText("r_ankle_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[22]));
}
void MainWindow::on_head_yaw_button_clicked()  // 1
{
	ui.joint_name_edit->setText("head_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[1]));
}
void MainWindow::on_head_pitch_button_clicked()  // 2
{
	ui.joint_name_edit->setText("head_pitch");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[2]));
}
void MainWindow::on_head_roll_button_clicked()  // 3
{
	ui.joint_name_edit->setText("head_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[3]));
}

void MainWindow::on_waist_roll_button_clicked()  // 10
{
	ui.joint_name_edit->setText("waist_roll");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[10]));
}
void MainWindow::on_waist_yaw_button_clicked()  // 9
{
	ui.joint_name_edit->setText("waist_yaw");
	ui.p_gain_edit->setText(QString::number(p_gain_value_[9]));
}



void MainWindow::on_initial_p_gain_load_button_clicked()
{
	qnode.read_p_gain_value_client.call(read_p_gain_value_msg_);// 저장된 위치값을 요청하고 받아온다.
	for(int id = 1; id < 30; id++)
	{
		p_gain_value_[id] = read_p_gain_value_msg_.response.dxl_state[id];
	}
}
void MainWindow::on_final_gain_save_button_clicked()
{

}
void MainWindow::on_gain_adjustment_button_clicked()
{

}




// control page gui /////////////////////////////////////////////////////////
void MainWindow::on_pflug_bogen_button_clicked()
{
	center_change_msg.turn_type = "pflug_bogen";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::on_parallel_button_clicked()
{
	center_change_msg.turn_type = "parallel";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::on_carving_button_clicked()
{
	center_change_msg.turn_type = "carving";
	qnode.center_change_pub.publish(center_change_msg);
}

void MainWindow::on_center_change_button_clicked()
{
	center_change_msg.change_type = "center_change";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::on_edge_change_button_clicked()
{
	center_change_msg.change_type = "edge_change";
	qnode.center_change_pub.publish(center_change_msg);
}
void MainWindow::center_change(int k)
{
	QString time_change_value_str;
	double  time_change_value;
	QString slider_value_str_temp;

	slider_value_str_temp = QString::number(k*0.01);

	time_change_value_str = ui.time_change->text();
	time_change_value = time_change_value_str.toDouble();

	ui.slider_center_change_value->setText(slider_value_str_temp);

	center_change_msg.center_change = k*0.01;
	center_change_msg.time_change = time_change_value;

	qnode.center_change_pub.publish(center_change_msg);
}

void MainWindow::edge_change(int k)
{
	QString time_change_value_str;
	double  time_change_value;
	QString slider_value_str_temp;

	slider_value_str_temp = QString::number(k*0.01);

	time_change_value_str = ui.time_change_edge->text();
	time_change_value = time_change_value_str.toDouble();

	ui.slider_edge_change_value->setText(slider_value_str_temp);

	center_change_msg.edge_change = k*0.01;
	center_change_msg.time_change = time_change_value;

	qnode.center_change_pub.publish(center_change_msg);
}
///////////////////////////////////////////////////////////////////////////////////////


void MainWindow::on_read_data_clicked() { // 모든 조인트 값을 읽어온다.
	//<------------------------------------------------->
	qnode.read_joint_value_client.call(read_joint_value_msg_);// 저장된 위치값을 요청하고 받아온다.

	//<-- 각 값들을 GUI에 표시 -->
	ui.lcd_read_joint_1->display(read_joint_value_msg_.response.dxl_state[1]);
	ui.lcd_read_joint_2->display(read_joint_value_msg_.response.dxl_state[2]);
	ui.lcd_read_joint_3->display(read_joint_value_msg_.response.dxl_state[3]);
	ui.lcd_read_joint_4->display(read_joint_value_msg_.response.dxl_state[4]);
	ui.lcd_read_joint_5->display(read_joint_value_msg_.response.dxl_state[5]);

	ui.lcd_read_joint_6->display(read_joint_value_msg_.response.dxl_state[6]);
	ui.lcd_read_joint_7->display(read_joint_value_msg_.response.dxl_state[7]);
	ui.lcd_read_joint_8->display(read_joint_value_msg_.response.dxl_state[8]);
	ui.lcd_read_joint_9->display(read_joint_value_msg_.response.dxl_state[9]);
	ui.lcd_read_joint_10->display(read_joint_value_msg_.response.dxl_state[10]);

	ui.lcd_read_joint_11->display(read_joint_value_msg_.response.dxl_state[11]);
	ui.lcd_read_joint_12->display(read_joint_value_msg_.response.dxl_state[12]);
	ui.lcd_read_joint_13->display(read_joint_value_msg_.response.dxl_state[13]);
	ui.lcd_read_joint_14->display(read_joint_value_msg_.response.dxl_state[14]);
	ui.lcd_read_joint_15->display(read_joint_value_msg_.response.dxl_state[15]);

	ui.lcd_read_joint_16->display(read_joint_value_msg_.response.dxl_state[16]);
	ui.lcd_read_joint_17->display(read_joint_value_msg_.response.dxl_state[17]);
	ui.lcd_read_joint_18->display(read_joint_value_msg_.response.dxl_state[18]);
	ui.lcd_read_joint_19->display(read_joint_value_msg_.response.dxl_state[19]);
	ui.lcd_read_joint_20->display(read_joint_value_msg_.response.dxl_state[20]);

	ui.lcd_read_joint_21->display(read_joint_value_msg_.response.dxl_state[21]);
	ui.lcd_read_joint_22->display(read_joint_value_msg_.response.dxl_state[22]);
}

void MainWindow::on_plus_button_clicked()
{
	int data[23] = {0,2040,0,0,0,0,0,0,0,0,2040,3068,1020,3068,1020,2040,2040,0,4090,3575,511,2040,2040};// 4095 값 미니멈

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	offset_change_value_str_ = ui.offset_value->text();
	offset_change_value_int16_ = offset_change_value_str_.toInt();

	joint_select_msg_.data = offset_joint_int16_;
	qnode.joint_select_pub.publish(joint_select_msg_);

	qnode.read_joint_value_client.call(read_joint_value_msg_);

	new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_] + offset_change_value_int16_; // 읽어온 현재 위치 값에서 변화된 값을 더해 새로운 값을 얻는다. 예) 1024(읽어온값) + 10 (변화시키고자한 값)

	if(new_offset_final_value_ > data[offset_joint_int16_]) // 4095을 넘어가는 것을 방지
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_]; // 지금 값 유지
	else
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_]+ offset_change_value_int16_; // 변화 값 저장

	ROS_INFO("%d",new_offset_final_value_ );

	change_joint_value_msg_.data.push_back(new_offset_final_value_); // 메세지에 저장
	qnode.change_joint_value_pub.publish(change_joint_value_msg_); // 메세지 전송

	change_joint_value_msg_.data.clear();


}

void MainWindow::on_minus_button_clicked()
{
	int data[23] = {0,-2040,0,0,0,0,0,0,0,0,-2040,-1020,-3060,-1020,-3060,-2040,-2040,-4090,0,-510,-3570,-2040,-2040};// 0 값 미니멈

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	offset_change_value_str_ = ui.offset_value->text();
	offset_change_value_int16_ = offset_change_value_str_.toInt();

	joint_select_msg_.data = offset_joint_int16_;
	qnode.joint_select_pub.publish(joint_select_msg_);

	qnode.read_joint_value_client.call(read_joint_value_msg_);

	new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_] - offset_change_value_int16_;

	if(new_offset_final_value_ < data[offset_joint_int16_])
	{
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_];
		ROS_INFO("END");
	}
	else
		new_offset_final_value_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_] - offset_change_value_int16_;

	change_joint_value_msg_.data.push_back(new_offset_final_value_); // 메세지에 저장
	qnode.change_joint_value_pub.publish(change_joint_value_msg_);

	change_joint_value_msg_.data.clear();
}


void MainWindow::on_read_data_one_clicked() {

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	joint_select_msg_.data = offset_joint_int16_;
	qnode.joint_select_pub.publish(joint_select_msg_);

	qnode.read_joint_value_client.call(read_joint_value_msg_);

	ui.lcd_read_data->display(read_joint_value_msg_.response.dxl_state[offset_joint_int16_]);

}
void MainWindow::on_initial_button_clicked() {

	int data[23] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};// 이론상 영점의 위치
	double degTorad = 0.0;
	QString DataToRad_str_temp;

	offset_joint_str_ = ui.offset_id->text();
	offset_joint_int16_ = offset_joint_str_.toInt();

	qnode.read_joint_value_client.call(read_joint_value_msg_);
	offset_DataToRad_int16_ = read_joint_value_msg_.response.dxl_state[offset_joint_int16_];

	ui.lcd_read_data->display(read_joint_value_msg_.response.dxl_state[offset_joint_int16_]);
	ui.lcd_read_data_2->display(data[offset_joint_int16_]);


	degTorad  = static_cast<double> (((offset_DataToRad_int16_ - data[offset_joint_int16_])*0.088*M_PI)/180);
	offset_joint_value_[offset_joint_int16_] = degTorad;

	DataToRad_str_temp = QString::number(degTorad);

	ui.lineEdit_offset->setText(DataToRad_str_temp);
}

void MainWindow::on_save_onoff_clicked() {

	save_onoff_msg_.data = true;
	qnode.save_onoff_pub.publish(save_onoff_msg_);

}

void MainWindow::on_send_offset_value_clicked() {

	for(int i=0;i<30;i++)
	{
		offset_joint_value_msg_.data.push_back(offset_joint_value_[i]);
	}

	qnode.offset_joint_value_pub.publish(offset_joint_value_msg_);

	offset_joint_value_msg_.data.clear();
}

void MainWindow::on_base_module_1_clicked() {
	module_msg.data = "base_module";
	qnode.module_on_off.publish(module_msg);
}

void MainWindow::on_init_module_call_clicked() {
	init_pose_msg.data = "init_pose";
	qnode.init_pose_pub.publish(init_pose_msg);
}


void MainWindow::on_offset_module_clicked() {
	module_msg.data = "offset_module";
	qnode.module_on_off.publish(module_msg);
}

void MainWindow::on_pose_module_clicked() {
	module_msg.data = "pose_module";
	qnode.module_on_off.publish(module_msg);
}
//////////////////////////////////////////////////////////
void MainWindow::on_motion_module_clicked() {
	module_msg.data = "motion_module";
	qnode.module_on_off.publish(module_msg);
}

void MainWindow::on_balance_on_clicked() {
	parse_gain_data();
	diana_balance_parameter_msg.updating_duration =  updating_duration;

	diana_balance_parameter_msg.cob_x_offset_m        = cob_x_offset_m;
	diana_balance_parameter_msg.cob_y_offset_m        = cob_y_offset_m;

	diana_balance_parameter_msg.foot_roll_gyro_p_gain = foot_roll_gyro_p_gain;
	diana_balance_parameter_msg.foot_roll_gyro_d_gain = foot_roll_gyro_d_gain;

	diana_balance_parameter_msg.foot_pitch_gyro_p_gain = foot_pitch_gyro_p_gain;
	diana_balance_parameter_msg.foot_pitch_gyro_d_gain = foot_pitch_gyro_d_gain;

	qnode.diana_balance_parameter_pub.publish(diana_balance_parameter_msg);
}

void MainWindow::on_balance_off_clicked() {
	parse_gain_data();
	diana_balance_parameter_msg.updating_duration =  updating_duration;

	diana_balance_parameter_msg.cob_x_offset_m        = 0;
	diana_balance_parameter_msg.cob_y_offset_m        = 0;

	diana_balance_parameter_msg.foot_roll_gyro_p_gain = 0;
	diana_balance_parameter_msg.foot_roll_gyro_d_gain = 0;

	diana_balance_parameter_msg.foot_pitch_gyro_p_gain = 0;
	diana_balance_parameter_msg.foot_pitch_gyro_d_gain = 0;

	qnode.diana_balance_parameter_pub.publish(diana_balance_parameter_msg);
}
//////////////////////////////////////////////////////////



void MainWindow::on_off_module_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}

void MainWindow::on_off_module_2_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_off_module_3_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}
void MainWindow::on_off_module_4_clicked() {
	module_msg.data = "none";
	qnode.module_on_off.publish(module_msg);
}


void MainWindow::on_change_kinematics_1_1_clicked() {
	//  1 pose   11 자 ////////////////////////////////////////////////////
	QString data_str_temp1;
	QString data_str_temp2;
	QString data_str_temp3;

	QString data_str_temp4;
	QString data_str_temp5;
	QString data_str_temp6;

	QString data_str_temp7;
	QString data_str_temp8;
	QString data_str_temp9;

	QString data_str_temp10;
	QString data_str_temp11;
	QString data_str_temp12;

	QString data_str_temp13;

	double data1 = 0;
	double data2 = 0.105;
	double data3 = -0.55;
	double data4 = 0;
	data4 = (data4*M_PI)/180;

	double data5 = 0;
	data5 = (data5*M_PI)/180;

	double data6 = 5;
	data6 = (data6*M_PI)/180;

	double data7 = 0;
	double data8 = -0.105;
	double data9 = -0.55;
	double data10 = 0;
	data10 = (data10*M_PI)/180;

	double data11 = 0;
	data11 = (data11*M_PI)/180;
	double data12 = -5;
	data12 = (data12*M_PI)/180;
	double data13 = 0;
	data13 = (data13*M_PI)/180;

	data_str_temp1 = QString::number(data1);
	data_str_temp2 = QString::number(data2);
	data_str_temp3 = QString::number(data3);

	data_str_temp4 = QString::number((data4*180)/M_PI);
	data_str_temp5 = QString::number((data5*180)/M_PI);
	data_str_temp6 = QString::number((data6)*180/M_PI);

	data_str_temp7 = QString::number(data7);
	data_str_temp8 = QString::number(data8);
	data_str_temp9 = QString::number(data9);

	data_str_temp10 = QString::number((data10)*180/M_PI);
	data_str_temp11 = QString::number((data11)*180/M_PI);
	data_str_temp12 = QString::number((data12)*180/M_PI);

	data_str_temp13 = QString::number((data13)*180/M_PI);

	ui.Edit_X->setText(data_str_temp1);
	ui.Edit_Y->setText(data_str_temp2);
	ui.Edit_Z->setText(data_str_temp3);

	ui.Edit_Z_angle->setText(data_str_temp4);
	ui.Edit_Y_angle->setText(data_str_temp5);
	ui.Edit_X_angle->setText(data_str_temp6);

	ui.Edit_X_2->setText(data_str_temp7);
	ui.Edit_Y_2->setText(data_str_temp8);
	ui.Edit_Z_2->setText(data_str_temp9);

	ui.Edit_Z_angle_2->setText(data_str_temp10);
	ui.Edit_Y_angle_2->setText(data_str_temp11);
	ui.Edit_X_angle_2->setText(data_str_temp12);

	ui.Edit_joint_10->setText(data_str_temp13);

	desired_pose_msg.data.push_back(data1);
	desired_pose_msg.data.push_back(data2);
	desired_pose_msg.data.push_back(data3);
	desired_pose_msg.data.push_back(data4);
	desired_pose_msg.data.push_back(data5);
	desired_pose_msg.data.push_back(data6);

	desired_pose_msg.data.push_back(data7);
	desired_pose_msg.data.push_back(data8);
	desired_pose_msg.data.push_back(data9);
	desired_pose_msg.data.push_back(data10);
	desired_pose_msg.data.push_back(data11);
	desired_pose_msg.data.push_back(data12);

	desired_pose_msg.data.push_back(data13);


	qnode.desired_pose_pub.publish(desired_pose_msg);

	desired_pose_msg.data.clear();

}

void MainWindow::on_initialize_module_clicked() {
	int data1 = 0; //
	pattern_msg.data = data1 ;

	qnode.pattern_pub.publish(pattern_msg);

}

void MainWindow::on_initialize_module_2_clicked() {
	// 2 pose ///////////////////////////////////
	QString data_str_temp1;
	QString data_str_temp2;
	QString data_str_temp3;

	QString data_str_temp4;
	QString data_str_temp5;
	QString data_str_temp6;

	QString data_str_temp7;
	QString data_str_temp8;
	QString data_str_temp9;

	QString data_str_temp10;
	QString data_str_temp11;
	QString data_str_temp12;

	QString data_str_temp13;

	double data1 = 0.1;
	double data2 = 0.255;
	double data3 = -0.51;
	double data4 = -15;
	data4 = (data4*M_PI)/180;

	double data5 = -10;
	data5 = (data5*M_PI)/180;

	double data6 = 15;
	data6 = (data6*M_PI)/180;

	double data7 = 0.1;
	double data8 = -0.255;
	double data9 = -0.51;
	double data10 = 15;
	data10 = (data10*M_PI)/180;

	double data11 = -10;
	data11 = (data11*M_PI)/180;

	double data12 = -15;
	data12 = (data12*M_PI)/180;

	double data13 = 0;
	data13 = (data13*M_PI)/180;

	data_str_temp1 = QString::number(data1);
	data_str_temp2 = QString::number(data2);
	data_str_temp3 = QString::number(data3);

	data_str_temp4 = QString::number((data4*180)/M_PI);
	data_str_temp5 = QString::number((data5*180)/M_PI);
	data_str_temp6 = QString::number((data6)*180/M_PI);

	data_str_temp7 = QString::number(data7);
	data_str_temp8 = QString::number(data8);
	data_str_temp9 = QString::number(data9);

	data_str_temp10 = QString::number((data10)*180/M_PI);
	data_str_temp11 = QString::number((data11)*180/M_PI);
	data_str_temp12 = QString::number((data12)*180/M_PI);

	data_str_temp13 = QString::number((data13)*180/M_PI);

	ui.Edit_X->setText(data_str_temp1);
	ui.Edit_Y->setText(data_str_temp2);
	ui.Edit_Z->setText(data_str_temp3);

	ui.Edit_Z_angle->setText(data_str_temp4);
	ui.Edit_Y_angle->setText(data_str_temp5);
	ui.Edit_X_angle->setText(data_str_temp6);

	ui.Edit_X_2->setText(data_str_temp7);
	ui.Edit_Y_2->setText(data_str_temp8);
	ui.Edit_Z_2->setText(data_str_temp9);

	ui.Edit_Z_angle_2->setText(data_str_temp10);
	ui.Edit_Y_angle_2->setText(data_str_temp11);
	ui.Edit_X_angle_2->setText(data_str_temp12);

	ui.Edit_joint_10->setText(data_str_temp13);

	desired_pose_msg.data.push_back(data1);
	desired_pose_msg.data.push_back(data2);
	desired_pose_msg.data.push_back(data3);
	desired_pose_msg.data.push_back(data4);
	desired_pose_msg.data.push_back(data5);
	desired_pose_msg.data.push_back(data6);

	desired_pose_msg.data.push_back(data7);
	desired_pose_msg.data.push_back(data8);
	desired_pose_msg.data.push_back(data9);
	desired_pose_msg.data.push_back(data10);
	desired_pose_msg.data.push_back(data11);
	desired_pose_msg.data.push_back(data12);

	desired_pose_msg.data.push_back(data13);
	qnode.desired_pose_pub.publish(desired_pose_msg);

	desired_pose_msg.data.clear();
}
void MainWindow::on_change_kinematics_2_clicked() {
	QString str1 = ui.Edit_X->text();
	double data1 = str1.toDouble();

	QString str2 = ui.Edit_Y->text();
	double data2 = str2.toDouble();

	QString str3 = ui.Edit_Z->text();
	double data3 = str3.toDouble();

	QString str4 = ui.Edit_Z_angle->text();
	double data4 = str4.toDouble();

	data4 = (data4*M_PI)/180;

	QString str5 = ui.Edit_Y_angle->text();
	double data5 = str5.toDouble();

	data5 = (data5*M_PI)/180;

	QString str6 = ui.Edit_X_angle->text();
	double data6 = str6.toDouble();

	data6 = (data6*M_PI)/180;




	QString str7 = ui.Edit_X_2->text();
	double data7 = str7.toDouble();

	QString str8 = ui.Edit_Y_2->text();
	double data8 = str8.toDouble();

	QString str9 = ui.Edit_Z_2->text();
	double data9 = str9.toDouble();

	QString str10 = ui.Edit_Z_angle_2->text();
	double data10 = str10.toDouble();

	data10 = (data10*M_PI)/180;

	QString str11 = ui.Edit_Y_angle_2->text();
	double data11 = str11.toDouble();

	data11 = (data11*M_PI)/180;

	QString str12 = ui.Edit_X_angle_2->text();
	double data12 = str12.toDouble();

	data12 = (data12*M_PI)/180;

	desired_pose_msg.data.push_back(data1);
	desired_pose_msg.data.push_back(data2);
	desired_pose_msg.data.push_back(data3);
	desired_pose_msg.data.push_back(data4);
	desired_pose_msg.data.push_back(data5);
	desired_pose_msg.data.push_back(data6);
	desired_pose_msg.data.push_back(data7);
	desired_pose_msg.data.push_back(data8);
	desired_pose_msg.data.push_back(data9);
	desired_pose_msg.data.push_back(data10);
	desired_pose_msg.data.push_back(data11);
	desired_pose_msg.data.push_back(data12);


	qnode.desired_pose_pub.publish(desired_pose_msg);

	desired_pose_msg.data.clear();

}
void MainWindow::parse_gain_data()
{
	updating_duration = 0.0;
	cob_x_offset_m = 0.0;
	cob_y_offset_m = 0.0;
	foot_roll_gyro_p_gain = 0.0;
	foot_roll_gyro_d_gain = 0.0;
	foot_pitch_gyro_p_gain = 0.0;
	foot_pitch_gyro_d_gain = 0.0;

	YAML::Node doc; // YAML file class 선언!
	std::string path_ = ros::package::getPath("atest_gui") + "/config/gain.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	// time load //
	updating_duration = doc["updating_duration"].as<double>();

	// offset load //
	cob_x_offset_m = doc["cob_x_offset_m"].as<double>();
	cob_y_offset_m = doc["cob_y_offset_m"].as<double>();

	//gain load //
	foot_roll_gyro_p_gain = doc["foot_roll_gyro_p_gain"].as<double>();
	foot_roll_gyro_d_gain = doc["foot_roll_gyro_d_gain"].as<double>();

	foot_pitch_gyro_p_gain = doc["foot_roll_gyro_p_gain"].as<double>();
	foot_pitch_gyro_d_gain = doc["foot_roll_gyro_d_gain"].as<double>();
}

}  // namespace atest_gui

