#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "diana_msgs/ForceTorque.h"
#include "ethercat.h"

#include "get_time.h"

#define EC_TIMEOUTMON 500

using namespace std;
using namespace GetTime;
using namespace boost::filesystem;
using namespace boost::lambda;

const int storage_size = 500;
double data_storage[2][6][storage_size];
double data_offset[2][6];

char IOmap[4096];
boolean needlf;
std_msgs::Bool ft_init_done;
uint8 currentgroup = 0;

void WriteLog(ofstream &log_file, double data[2][6]);
void SendCommand(uint8_t *data, uint16_t *buf, int buf_length);
void PrintValues(uint8_t *data);
void GetSensorValue(double new_data[6], uint8_t *data);
void SetRosMsg(diana_msgs::ForceTorque *ft_msg, double data[][6]);
double Kalman(double *input, double *data, int length);
OSAL_THREAD_FUNC ec_check(void *ptr);
void SetupFT(int addr);
void InitFT(int addr);
int CountFiles(path the_path);

void InitCallback(const std_msgs::Bool msg)
{
  if(msg.data == TRUE)
  {
    ft_init_done.data = FALSE;
    InitFT(1);
    InitFT(2);
    ft_init_done.data = TRUE;
  }
}

int main(int argc, char *argv[])
{
  char ifname[] = "eth0";

  ofstream log_file;
  char file_location[80] = "/home/nvidia/logs/ski_project/force_torque/";
  char date[40];

  sprintf(date, "log_%04d.txt", CountFiles(path(file_location))+1);
  strcat(file_location, date);

  log_file.open(file_location);

  for(int i=0 ; i<6 ; i++)
    for(int j=0 ; j<storage_size ; j++) {
      data_storage[0][i][j] = 0;
      data_storage[1][i][j] = 0;
    }

  needlf = FALSE;

  ros::init(argc, argv, "ec_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_init = nh.subscribe("/diana/ft_init", 1, InitCallback);
  ros::Publisher ec_data = nh.advertise<diana_msgs::ForceTorque>("/diana/force_torque_data", 1);
  ros::Publisher init_done = nh.advertise<std_msgs::Bool>("/diana/ft_init_done", 1);

  diana_msgs::ForceTorque ft_msg;

  if(ec_init(ifname))
  {
    if(ec_config_init(FALSE) > 0)
    {
      ec_config_map(&IOmap);
      ec_configdc();

      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_writestate(0);

      int chk = 40;
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      }
      while(chk-- && ec_slave[0].state != EC_STATE_OPERATIONAL); 

      log_file << "      time |" 
        << "     Fx_l ,      Fy_l ,      Fz_l  |     Tx_l ,      Ty_l ,      Tz_l  |" 
        << "     Fx_r ,      Fy_r ,      Fz_r  |     Tx_r ,      Ty_r ,      Tz_r  |" 
        << endl;

      while(ec_slave[0].state == EC_STATE_OPERATIONAL && ros::ok())
      {
        if(!needlf)
        {
          SetupFT(1);
          SetupFT(2);
          needlf = TRUE;
        }

        // update ethercat slave....
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        double data[2][6];
        for(int n=0 ; n<ec_slavecount ; n++)
        {
          //PrintValues(ec_slave[n+1].inputs);
          GetSensorValue(data[n], ec_slave[n+1].inputs);

          for(int i=0 ; i<6 ; i++)
          {
            Kalman(data[n]+i, data_storage[n][i], storage_size);
            data[n][i] -= data_offset[n][i];
          }

          /*
             printf("\n");
             printf("Force  : Fx= %10.2f, Fy= %10.2f, Fz= %10.2f \n",
             data[n][0], data[n][1], data[n][2]);
             printf("Torque : Tx= %10.2f, Ty= %10.2f, Tz= %10.2f \n",
             data[n][3], data[n][4], data[n][5]);
           */
        }

        WriteLog(log_file, data);

        SetRosMsg(&ft_msg, data);

        ec_data.publish(ft_msg);
        init_done.publish(ft_init_done);
        ros::spinOnce();

        //osal_usleep(10);
      }


      printf("Not all slaves reached operational state.\n");
      ec_readstate();
      for(int i = 1; i<=ec_slavecount ; i++)
      {
        if(ec_slave[i].state != EC_STATE_OPERATIONAL)
        {
          printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
              i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
      }
      printf("\nRequest init state for all slaves\n");
      ec_slave[0].state = EC_STATE_INIT;
      /* request INIT state for all slaves */
      ec_writestate(0);
    }
    else
    {
      printf("No slaves found!\n");
    }
    ec_close();
    log_file.close();
  }
  else
  {
    printf("No socket connection on %s\nExcecute as root\n",ifname);
  }
}

void WriteLog(ofstream &log_file, double data[2][6])
{
  log_file << fixed << setprecision(3) << setw(10) << GetMillis() << " |" 
    << setw(10) << data[0][0] << ", " << setw(10) << data[0][1] << ", " << setw(10) << data[0][2] << " |"
    << setw(10) << data[0][3] << ", " << setw(10) << data[0][4] << ", " << setw(10) << data[0][5] << " |"
    << setw(10) << data[1][0] << ", " << setw(10) << data[1][1] << ", " << setw(10) << data[1][2] << " |"
    << setw(10) << data[1][3] << ", " << setw(10) << data[1][4] << ", " << setw(10) << data[1][5] << " |"
    << endl;
}


void SendCommand(uint8_t *data, uint16_t *buf, int buf_length)
{
  for(int i=0 ; i<buf_length ; i++)
  {
    *data++ = (buf[buf_length-i-1] >> 0) & 0xFF;
    *data++ = (buf[buf_length-i-1] >> 8) & 0xFF;
  }
  ec_send_processdata();
}

void GetSensorValue(double new_data[6], uint8_t *data)
{
  int16_t force_torque_data[6];

  // Raw_Fx, Raw_Fy, Raw_Fy (2 Bytes * 3)
  force_torque_data[0] = (int16_t)(data[24] | data[25] << 8);
  force_torque_data[1] = (int16_t)(data[26] | data[27] << 8);
  force_torque_data[2] = (int16_t)(data[28] | data[29] << 8);

  // Raw_Tx, Raw_Ty, Raw_Tz (2 Bytes * 3)
  force_torque_data[3] = (int16_t)(data[30] | data[31] << 8);
  force_torque_data[4] = (int16_t)(data[32] | data[33] << 8);
  force_torque_data[5] = (int16_t)(data[34] | data[35] << 8);

  double force_divider = 50;
  double torque_divider = 2000;

  for(int i=0 ; i<3 ; i++)
  {
    new_data[i] = (double)force_torque_data[i]/force_divider;
    new_data[i+3] = (double)force_torque_data[i+3]/torque_divider;
  }

  /*
     printf("\n");
     printf("Raw_Force  : Fx= %10.2f, Fy= %10.2f, Fz= %10.2f \n",
     force[0], force[1], force[2]);
     printf("Raw_Torque : Tx= %10.2f, Ty= %10.2f, Tz= %10.2f \n",
     torque[0], torque[1], torque[2]);
   */
}

void SetRosMsg(diana_msgs::ForceTorque *ft_msg, double data[][6])
{
  ft_msg->force_x_raw_l = data[0][0];
  ft_msg->force_y_raw_l = data[0][1];
  ft_msg->force_z_raw_l = data[0][2];
  ft_msg->torque_x_raw_l = data[0][3];
  ft_msg->torque_y_raw_l = data[0][4];
  ft_msg->torque_z_raw_l = data[0][5];
  ft_msg->force_x_raw_r = data[1][0];
  ft_msg->force_y_raw_r = data[1][1];
  ft_msg->force_z_raw_r = data[1][2];
  ft_msg->torque_x_raw_r = data[1][3];
  ft_msg->torque_y_raw_r = data[1][4];
  ft_msg->torque_z_raw_r = data[1][5];
}

double Kalman(double *input, double *data, int length)
{
  double result = (*input / length);

  for(int i=1 ; i<length ; i++)
  {
    result += (data[i] / length);
    data[i-1] = data[i];
  }
  data[length-1] = *input;

  *input = result;
  return result;
}

void PrintValues(uint8_t *data)
{
  // CanRx1_id (2 Bytes)
  uint16_t can_rx1_id = *data | (*(data+1) >> 8);
  data += 2;

  // CanRx1_len (2 Bytes)
  uint16_t can_rx1_len = *data | (*(data+1) >> 8);
  data += 2;

  // CanRx1_data_d1 ~ d8 (1 Byte * 8)
  uint8_t can_rx1_data[8];
  for(int i=0 ; i<8 ; i++)
    can_rx1_data[i] = *data++;

  // CanRx2_id (2 Bytes)
  uint16_t can_rx2_id = *data | (*(data+1) >> 8);
  data += 2;

  // CanRx2_len (2 Bytes)
  uint16_t can_rx2_len = *data | (*(data+1) >> 8);
  data += 2;

  // CanRx2_data_d1 ~ d8 (1 Byte * 8)
  uint8_t can_rx2_data[8];
  for(int i=0 ; i<8 ; i++)
    can_rx2_data[i] = *data++;

  int16_t raw_force[3];
  // Raw_Fx (2 Bytes)
  raw_force[0] = (int16_t)(*data | (*(data+1) >> 8));
  data += 2;
  // Raw_Fy (2 Bytes)
  raw_force[1] = (int16_t)(*data | (*(data+1) >> 8));
  data += 2;
  // Raw_Fz (2 Bytes)
  raw_force[2] = (int16_t)(*data | (*(data+1) >> 8));
  data += 2;

  int16_t raw_torque[3];
  // Raw_Tx (2 Bytes)
  raw_torque[0] = (int16_t)(*data | (*(data+1) >> 8));
  data += 2;
  // Raw_Ty (2 Bytes)
  raw_torque[1] = (int16_t)(*data | (*(data+1) >> 8));
  data += 2;
  // Raw_Tz (2 Bytes)
  raw_torque[2] = (int16_t)(*data | (*(data+1) >> 8));
  data += 2;

  // OverloadStatus (1 Byte)
  uint8_t overload_status = *data++;

  // ErrorFlag (1 Byte)
  uint8_t error_flag = *data++;

  printf("CanRx1_id  : %d \n", can_rx1_id);
  printf("CanRx1_len : %d \n", can_rx1_len);
  printf("CanRx1_data_d1~d8 : ");
  for(int i=0 ; i<8 ; i++)
    printf("%3d ", can_rx1_data[i]);
  printf("\n");
  printf("CanRx2_id  : %d \n", can_rx2_id);
  printf("CanRx2_len : %d \n", can_rx2_len);
  printf("CanRx2_data_d1~d8 : ");
  for(int i=0 ; i<8 ; i++)
    printf("%3d ", can_rx2_data[i]);
  printf("\n");
  printf("Raw_Force  : Fx= %4d, Fy= %4d, Fz= %4d \n", raw_force[0], raw_force[1], raw_force[2]);
  printf("Raw_Torque : Tx= %4d, Ty= %4d, Tz= %4d \n", raw_torque[0], raw_torque[1], raw_torque[2]);
  printf("OverloadStatus : %2d \n", overload_status);
  printf("ErrorFlag  : %2d \n\n\n", error_flag);
}

void SetupFT(int addr)
{
  int buf_size = 4;
  uint16_t buf[buf_size];

  //stop command (for setup the f/t sensor)
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x0C;
  SendCommand(ec_slave[addr].outputs, buf, buf_size);

  //set cutoff frequency to 150Hz
  buf[0] = 0x00;
  buf[1] = 0x04;
  buf[2] = 0x01;
  buf[3] = 0x08;
  SendCommand(ec_slave[addr].outputs, buf, buf_size);
  //ec_receive_processdata(EC_TIMEOUTRET);

  //start command
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x0B;
  SendCommand(ec_slave[addr].outputs, buf, buf_size);

  InitFT(addr);
}

void InitFT(int addr)
{
  for(int j=0 ; j<6 ; j++)
  {
    data_offset[addr-1][j] = 0;
  }

  double data[6];
  for(int i=0 ; i<storage_size ; i++)
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    GetSensorValue(data, ec_slave[addr].inputs);
    for(int j=0 ; j<6 ; j++)
    {
      data_storage[addr-1][j][i] = data[j];
      data_offset[addr-1][j] += data[j];
    }
  }

  for(int j=0 ; j<6 ; j++)
  {
    data_offset[addr-1][j] /= storage_size;
  }
}

int CountFiles(path the_path)
{
  int cnt = count_if(
      directory_iterator(the_path),
      directory_iterator(),
      static_cast<bool(*)(const path&)>(is_regular_file) );
  return cnt;
}


