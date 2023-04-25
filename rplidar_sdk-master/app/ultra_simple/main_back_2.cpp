/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

#define LIMIT_DIST 600
#define BRAKE_DIST 300
#define MAX_ANG 2000
const float MIN_TARGET_ANGLE = 50, MAX_TARGET_ANGLE = 90;
const float FRONT_ANGLE = 10;
double error = 0.0;     // 偏差值
double error_sum = 0.0; // 偏差值的累积和
double error_diff = 0.0;// 偏差值的变化率
double prev_output = 0.0;
double prev_error = 0.0;

double PID_control(double target_dist, double v = 100){
    // PID控制器参数
    double Kp = 10; //0.3;
    double Ki = 0;
    double Kd = 20; //3;

    // 车辆状态变量
    
    // 更新车辆状态变量

    // 计算偏差值
    error = LIMIT_DIST - target_dist;
    // 计算偏差值的累积和
    error_sum += error;
    // 计算偏差值的变化率
    error_diff = error - prev_error;
    prev_error = error;
    // 计算控制输出值
    double output = Kp * error + Ki * error_sum + Kd * error_diff;
    if (output > MAX_ANG) output = MAX_ANG;
    else if(output < -MAX_ANG) output = -MAX_ANG;
    return output/MAX_ANG;
}


void print_usage(int argc, const char * argv[])
{
    printf("hello world!");
    printf("Simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s \n"
           "Usage:\n"
           " For serial channel %s --channel --serial <com port> [baudrate]\n"
           "The baudrate is 115200(for A2) or 256000(for A3).\n"
		   " For udp channel %s --channel --udp <ipaddr> [port NO.]\n"
           "The LPX default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , "SL_LIDAR_SDK_VERSION", argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
    std::ofstream logfile("/home/hu/Desktop/memory_file_sys/lidar_log.txt");
	const char * opt_is_channel = NULL; 
	const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
	sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

	bool useArgcBaudrate = false;

    IChannel* _channel;

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", "SL_LIDAR_SDK_VERSION");

	 
	if (argc>1)
	{ 
		opt_is_channel = argv[1];
	}
	else
	{
		print_usage(argc, argv);
		return -1;
	}

	if(strcmp(opt_is_channel, "--channel")==0){
		opt_channel = argv[2];
		if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
		{
			// read serial port from the command line...
			opt_channel_param_first = argv[3];// or set to a fixed value: e.g. "com3"
			// read baud rate from the command line if specified...
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);	
			useArgcBaudrate = true;
		}
		else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
		{
			// read ip addr from the command line...
			opt_channel_param_first = argv[3];//or set to a fixed value: e.g. "192.168.11.2"
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);//e.g. "8089"
			opt_channel_type = CHANNEL_TYPE_UDP;
		}
		else
		{
			print_usage(argc, argv);
			return -1;
		}
	}
	else
	{
		print_usage(argc, argv);
        return -1;
	}

	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
	{
		if (!opt_channel_param_first) {
#ifdef _WIN32
		// use default com port
		opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
		opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
		opt_channel_param_first = "/dev/ttyUSB0";
#endif
		}
	}

    
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
        if(useArgcBaudrate){
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result)) 
                {
	                connectSuccess = true;
                }
                else{
                    delete drv;
					drv = NULL;
                }
            }
        }
        else{
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
			for(size_t i = 0; i < baudRateArraySize; ++i)
			{
				_channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result)) 
                    {
	                    connectSuccess = true;
                        break;
                    }
                    else{
                        delete drv;
					    drv = NULL;
                    }
                }
			}
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP){
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) 
            {
	            connectSuccess = true;
            }
            else{
                delete drv;
				drv = NULL;
            }
        }
    }


    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));
		
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);
    
    // fetech result and print it out...
    
    while (1) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        
        float output = 0;
        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            int frame_cnt = 0;
            int target_cnt = 0, front_cnt = 0;
            float target_dist = 0, front_dist = 0;
            for (int pos = 0; pos < (int)count ; ++pos) {

                bool start_flag = (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT);    //flag of start frames
                float theta = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;    // theta in degrees
                float dist = nodes[pos].dist_mm_q2/4.0f;            // distance in milimeters
                int qua = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT; //frame quality, 0 for unavailable points
                
                
                
                if (qua != 0 && theta <= MAX_TARGET_ANGLE && theta >= MIN_TARGET_ANGLE){
                    target_dist += dist;
                    target_cnt++;
                }
                else if (qua != 0 && (theta <= FRONT_ANGLE || theta >= 360-FRONT_ANGLE)){
                    front_dist += dist;
                    front_cnt++;
                }
                
                if(start_flag){
                    frame_cnt++;
                    int tmp = count;
                    printf("pos: %d, frame_cnt: %d, count: %d\n", pos, frame_cnt, tmp);
                } 
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    start_flag ?" Start Frame ":"  ", 
                    theta,
                    dist,
                    qua);
                
                logfile << start_flag << " " << theta << " " << dist << " " << qua << std::endl;

            }
            if (target_cnt > 20)  output = PID_control(target_dist /= target_cnt)/2;
            if (front_cnt > 10)  front_dist /= front_cnt;
            bool brake_flag = (front_cnt >= 10 && front_dist <= BRAKE_DIST);
            if (abs(output - prev_output) > 1.5) output = prev_output;
            else prev_output = output;

            // logfile << (double)target_dist << ' ' << (double)front_dist << ' ' << (double)output << std::endl;
            std::ofstream outfile("/home/hu/Desktop/memory_file_sys/lidar_data.txt");
            outfile << (double)target_dist << ' ' << (double)output << ' ' << brake_flag << std::endl;
            // outfile.seekp(0, std::ios::beg );
            // outfile << '\r';
            outfile.flush();
            outfile.close();
            printf("front_cnt: %d", front_cnt);
            printf("----target dist=%0.2f, target_cnt=%d, output=%0.2f, front_dist=%0.2f, brake=%d----\n", target_dist, target_cnt, output, front_dist, brake_flag);

        }

        if (ctrl_c_pressed){ 
            logfile.close();
            // outfile.close();
            break;
        }
    }
    // outfile.close();
    drv->stop();
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    // done!
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}

