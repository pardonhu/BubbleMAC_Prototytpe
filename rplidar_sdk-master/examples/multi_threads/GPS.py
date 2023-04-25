# coding: utf-8
# last modified:20220824
import time
import serial
import re
import datetime

import numpy as np
# import matplotlib.pyplot as plt
import subprocess


utctime = ''
lat = ''
ulat = ''
lon = ''
ulon = ''
numSv = ''
msl = ''
cogt = ''
cogm = ''
sog = ''
kph = ''
gps_t = 0





def Convert_to_degrees(in_data1, in_data2):
    len_data1 = len(in_data1)
    str_data2 = "%05d" % int(in_data2)
    temp_data = int(in_data1)
    symbol = 1
    if temp_data < 0:
        symbol = -1
    degree = int(temp_data / 100.0)
    str_decimal = str(in_data1[len_data1-2]) + str(in_data1[len_data1-1]) + str(str_data2)
    f_degree = int(str_decimal)/60.0/100000.0
    # print("f_degree:", f_degree)
    if symbol > 0:
        result = degree + f_degree
    else:
        result = degree - f_degree
    return result


gps_time_list = []
pc_time_list = []


def format_time(raw_time):
    dt = datetime.datetime.strptime(raw_time.decode(), ",%H%M%S.%f")
    # return dt.strftime("2023-%m-%d %H:%M:%S")
    return dt.strftime("2023-04-21 %H:%M:%S")

def GPS_read(ser):
    global utctime
    if ser.inWaiting():
        if ser.read(1) == b'G':
            if ser.inWaiting():
                if ser.read(1) == b'N':
                    if ser.inWaiting():
                        choice = ser.read(1)
                        if choice == b'G':
                            if ser.inWaiting():
                                if ser.read(1) == b'G':
                                    if ser.inWaiting():
                                        if ser.read(1) == b'A':
                                            utctime = ser.read(11) # 读取串口数据
                                            gps_time_list.append(utctime) #记录gps报告时间
                                            pc_time_list.append(time.time()) #记录电脑时间
                                            print('utctime:', format_time(utctime))

                                            str1 = str(format_time(utctime))     #格式化GPS报告的时间
                                            # str2 = "2022-04-17 12:00:00"

                                            print('set clock:', str1)
                                            #print('2:', str2)
                                            output = subprocess.check_output(["date","-s", str1])  #设置时间
                                            # print(output.decode())
                                            return True
    return False

def main(Ser_name = "/dev/ttyUSB0", period = 300):
    
    print("begin to set the clock!")
    Baudrate = 115200

    ser = serial.Serial(Ser_name, Baudrate)

    if ser.isOpen():
        print("GPS Serial Opened! Baudrate="+str(Baudrate))
    else:
        print("GPS Serial Open Failed!")
       
    while True: 
        try:
            if(GPS_read(ser)):
                time.sleep(period)
        except Exception as e:
            print('GPS Exceptions!', e)
            time.sleep(10)
        except KeyboardInterrupt:
            ser.close()
            print("GPS serial Close!")
        
if __name__ == '__main__':
    main()
    





# print(gps_time_list)
# print(pc_time_list)
# print(len(gps_time_list))


# # 求误差
# errors = []
# for i in range(0, len(gps_time_list)):
#     s = gps_time_list[i].decode("utf-8")
#     gps_t = datetime.datetime.strptime(s, ",%H%M%S.%f") + datetime.timedelta(days=45012,hours = 8)
#     print(gps_t)
#     gps_timestamp = gps_t.timestamp()

#     pc_timestamp = pc_time_list[i]
#     errors.append(gps_timestamp - pc_timestamp)
#     # gps_t = float(str(gps_time_list[i])[3:-1])
#     # print(gps_t)



# print(errors)



# # 存储文件
# import pickle


# with open(Ser_name +'_data1.pickle', 'wb') as f:
#     pickle.dump(errors, f)



# with open('COM5_data.pickle', 'rb') as f:
#     errors_data1 = pickle.load(f)

# with open('COM7_data.pickle', 'rb') as f:
#     errors_data2 = pickle.load(f)

# data = [errors_data1, errors_data2]
# labels = ['module 1', 'module 2']

# # 绘制箱线图
# fig, ax = plt.subplots()
# ax.boxplot(data, labels=labels)
# ax.set_title('Error Boxplot')
# ax.set_xlabel('Error')
# ax.set_ylabel('Value')
# plt.show()

