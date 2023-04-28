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
#include <sstream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <numeric>
#include <cmath>

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
using namespace std;
#define LIMIT_DIST 700
#define BRAKE_DIST 500

#define MAX_ANG 2000
#define WINDOW_SIZE 30
#define MAX_DATA_POINTS 1000

#define AMBIGUITY_TH 1.8

double error = 0.0;     // 偏差值
double error_sum = 0.0; // 偏差值的累积和
double error_diff = 0.0;// 偏差值的变化率
double prev_output = 0.0;
double prev_error = 0.0;
double prev_dist = 0.0;
double cur_time = 0.0, prev_time = 0.0;
double DEC_DIST = 1500.0;

bool check_target_angle(double ang){
    const double MIN_TARGET_ANGLE = 230.0, MAX_TARGET_ANGLE = 270.0;
    return ang <= MAX_TARGET_ANGLE && ang >= MIN_TARGET_ANGLE;
}
bool check_front_angle(double ang){
    const double MIN_FRONT_ANGLE = 170.0, MAX_FRONT_ANGLE = 190.0;
    return ang <= MAX_FRONT_ANGLE && ang >= MIN_FRONT_ANGLE;
}


double get_vel_relative(double front_dist, double ratio=0.1){
    double ret = (front_dist - prev_dist) / (cur_time - prev_time);
    prev_time = cur_time;
    prev_dist = front_dist;
    return ret / 1000.0;
}

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

// 定义二维点结构体
struct Point {
    double x, y;
};

// 计算两个点之间的距离
double distance(Point p1, Point p2) {
    return std::sqrt(pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 判断一个点是否在圆锥形空间内
bool inCone(Point p) {
    double angle = std::atan2(p.y, p.x) * 180 / M_PI;  // 计算点p的极角，单位为度
    return check_front_angle(angle);
}
Point theta2point(double theta, double dist){
    Point ret;
    ret.x = dist * std::cos(theta / 360 * 3.146 * 2);
    ret.y = dist * std::sin(theta / 360 * 3.146 * 2);
    return ret;
}
// 聚类算法，返回聚类中心坐标
Point cluster(std::vector<Point> points) {
    double sum_x = 0, sum_y = 0;
    for (auto p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    return {sum_x / points.size(), sum_y / points.size()};
}

double check_front_distance(const std::vector<Point>& points) {
    // 过滤出在圆锥形空间内的点

    // 聚类算法，返回聚类中心坐标
    Point center = cluster(points);

    // 找到距离原点最近的聚类中心
    double min_distance = distance({0, 0}, center);
    for (auto p : points) {
        double d = distance({0, 0}, p);
        if (d < min_distance) {
            center = p;
            min_distance = d;
        }
    }

    // std::cout << "Front distance: " << min_distance << std::endl;
    return min_distance;
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
    std::vector<double> history_dist;
    auto current_time = std::chrono::system_clock::now();
    std::time_t current_time_t = std::chrono::system_clock::to_time_t(current_time);
    std::tm* current_time_tm = std::localtime(&current_time_t);
    std::ostringstream filename_stream;
    filename_stream << "/home/hu/Desktop/memory_file_sys/lidar_log/" << std::put_time(current_time_tm, "%Y%m%d_%H%M%S") << ".txt";
    std::string filename = filename_stream.str();
    std::ofstream logfile(filename);
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
		opt_channel_param_first = "/dev/a-lidar";
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

        
        double output = 0;
        std::vector<Point> front_points;
        // front_points.clear();
        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            int frame_cnt = 0;
            int target_cnt = 0, front_cnt = 0;
            double target_dist = 0, front_dist = 0;
            
            
            for (int pos = 0; pos < (int)count ; ++pos) {

                bool start_flag = (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT);    //flag of start frames
                double theta = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;    // theta in degrees
                double dist = nodes[pos].dist_mm_q2/4.0f;            // distance in milimeters
                int qua = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT; //frame quality, 0 for unavailable points
                
                
                
                if (qua != 0 && check_target_angle(theta)){
                    target_dist += dist * std::abs(std::sin(theta / 360 * 3.1415926 * 2));
                    target_cnt++;
                }
                else if (qua != 0 && check_front_angle(theta)){
                    front_dist += dist;
                    front_cnt++;
                    auto p = theta2point(theta, dist);
                    front_points.emplace_back(p);
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
            if (target_cnt > 10){
                target_dist /= target_cnt;
                
                
                logfile << "history_size: " << history_dist.size() <<  " window_size:" <<  WINDOW_SIZE << std::endl;
                if (history_dist.size() >= 1){
                    double his_avg = std::accumulate(history_dist.begin(), history_dist.end(), 0.0) / history_dist.size();
                    logfile << "Before avg: " << target_dist <<  " his_avg" << his_avg<< std::endl;
                    if (abs(target_dist - LIMIT_DIST) >= 200){
                        if (target_dist > AMBIGUITY_TH * his_avg) logfile << "foo!" << std::endl;
                        target_dist = (target_dist > AMBIGUITY_TH * his_avg)?
                            his_avg : target_dist; // 默认window size为10
                    }
                }
                history_dist.emplace_back(target_dist);
                output = PID_control(target_dist) / 2;
                
                

                if (history_dist.size() > WINDOW_SIZE + 10){
                    history_dist.erase(history_dist.begin(), history_dist.begin() + 10);
                    logfile << "ERASE!!" << std::endl;
                }
                // 确保历史距离记录值不超过一定数目

                //record_data(target_dist, output); // 记录数据
                // record_data中每记录200个点，调用gnuplot画图
            }
            auto current_time2 = std::chrono::high_resolution_clock::now();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(current_time2.time_since_epoch()).count();
            cur_time = static_cast<double>(millis) / 1000.0f;
            if (front_cnt > 0)  front_dist = check_front_distance(front_points);
            double vel_relative = get_vel_relative(front_dist);
            bool brake_flag = (front_cnt >= 10 && front_dist <= BRAKE_DIST);



           
            std::ofstream outfile("/home/hu/Desktop/memory_file_sys/lidar_data.txt");
            outfile << (double)target_dist << ' ' << (double)output << ' ' << vel_relative << ' ' << brake_flag << ' ' << front_dist << std::endl;
            logfile << "Frame Over:" << (double)target_dist << ' ' << (double)output << ' ' << vel_relative << ' ' << brake_flag << std::endl;
            outfile.flush();
            outfile.close();
            printf("front_cnt: %d", front_cnt);
            printf("----target dist=%0.2f, target_cnt=%d, output=%0.2f, front_dist=%0.2f, vel_relative=%0.2f, brake=%d----\n", 
                target_dist, target_cnt, output, front_dist, vel_relative, brake_flag);

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

