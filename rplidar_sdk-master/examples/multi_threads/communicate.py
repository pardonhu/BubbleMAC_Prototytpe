import serial #导入模块
import threading
import time
import math
import struct
import numpy as np
from . import GPS
STRGLO="" #读取的数据
OP="" #acc or dec
BOOL=True  #读取标志位
MAX_SPEED = 6.0
ACC = 0.4
DEC = 0.2
DELTA_TIME = 0.1
TAO = 1
front_dist = 3
rel_dist = 2
vel_leader = 0.0
vel_cur = 0.0
slot = 0
queue_flag = False
prev_seq_num, cur_seq_num = -1, 0
received_num = 0
vel_log = open('/home/hu/Desktop/memory_file_sys/vel_log.txt', 'w')

def get_vel(vel_l, dist):
    # dist /= 1000
    dist = max(dist - 0.5, 0.0)
    vel_f = vel_cur
    vel_l = max(vel_l, 0)
    vel_new = min(MAX_SPEED, vel_f + ACC * DELTA_TIME, \
        -DEC*TAO + math.sqrt((DEC*TAO)**2 + vel_l**2 + 2*DEC*dist))
    # if vel_new < 0.1:
    #     vel_new = 0
    vel_new = min(max(vel_new, 0), 0.9)
    print('3 vels: ', MAX_SPEED, vel_f + ACC * DELTA_TIME, \
        -DEC*TAO + math.sqrt((DEC*TAO)**2 + vel_l**2 + 2*DEC*dist))
    vel_info = f'vel_info: {vel_f + ACC * DELTA_TIME} {-DEC*TAO + math.sqrt((DEC*TAO)**2 + vel_l**2 + 2*DEC*dist)} {dist} {vel_new} {vel_f} {vel_l} {vel_cur}\n'
                
    vel_log.write(vel_info)
    vel_log.flush()
    print(f'dist:{dist}, vel_new:{vel_new}, vel_f:{vel_f}, vel_l:{vel_l}, vel_cur:{vel_cur}')
    return vel_new

def get_comm_radius(dist):
    return dist + 500

def pwm_to_vel(pwm):
    ret = (pwm - 100) / 200 * 0.9
    return max(ret, 0)
    

def vel_to_pwm(pwm):
    return pwm / 0.9 * 200 + 100

#读数代码本体实现
def ReadData(ser):
    global STRGLO, BOOL, vel_cur
    
    # 循环接收数据，此为死循环，可用线程实现
    cur_time = time.time()
    prev_time = cur_time
    while BOOL:
        if ser.in_waiting:
            STRGLO = ser.read(ser.in_waiting)
            try:
                vel_info = STRGLO[:7].decode('gbk')

                # print(STRGLO.decode('gbk')[:-1], end='')
                # print('over')
                loc = vel_info.find(':')
                if loc != -1:
                    # loc2 = vel_info.find('\t')
                    cur_time = time.time()
                    tmp = vel_info[loc+1:loc + 4]
                    if tmp[-1] == '\t':
                        tmp = tmp[:-1]
                    elif tmp[0] == '\t':
                        tmp = 100
                    # print(tmp)
                    vel_cur = pwm_to_vel(int(tmp))
                    vel_info.write(f'vel_cur: {vel_cur}\n')
                    vel_info.flush()
                    if(cur_time - prev_time > 1):
                        print(vel_info, f'read from vel_cur:{tmp} to {vel_cur}')
                        prev_time = cur_time
                        
                
            except Exception as e:
                pass
                # print('ERROR STM ', STRGLO)
                # print(e)
                # vel_cur = 0
            

                


#打开串口
# 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
# 波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
# 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
def DOpenPort(portx,bps,timeout):
    ret=False
    try:
        # 打开串口，并得到串口对象
        ser = serial.Serial(portx, bps, timeout=timeout)
        #判断是否打开成功
        if(ser.is_open):
           ret=True
           threading.Thread(target=ReadData, args=(ser,)).start()
    except Exception as e:
        print("---异常---:", e)
    return ser,ret

def GPSOpenPort(portx,bps,timeout):
    ret=False
    try:
        # 打开串口，并得到串口对象
        ser = serial.Serial(portx, bps, timeout=timeout)
        #判断是否打开成功
        if(ser.is_open):
           ret=True
           threading.Thread(target=GPS.GPS_run, args=(ser,)).start()
    except Exception as e:
        print("---异常---:", e)
    return ser,ret


#关闭串口
def DColsePort(ser):
    global BOOL
    BOOL=False
    ser.close()



#写数据
def DWritePort(ser,text):
    if type(text) is str:
        result = ser.write(text.encode("gbk"))  # 写数据
    else:
        
        result = ser.write(text)  # 写数据
    return result


#读数据
def DReadPort():
    global STRGLO
    str=STRGLO
    STRGLO=""#清空当次读取
    return str

import os


def read_latest_packet(ser_stm, role, wifi_path='/home/hu/Desktop/memory_file_sys/wifi.pcap', \
                       lidar_path = '/home/hu/Desktop/memory_file_sys/lidar_data.txt', packet_len=50):
    global front_dist, vel_cur, prev_seq_num, cur_seq_num, rel_dist, vel_leader, received_num, queue_flag
    missed_packet_num = 0
    while(1):
        packet_flag = False
        # print('---begin reading lidar---')
        
        with open(lidar_path, 'r') as in_lidar:
            try:
                l = in_lidar.readline()
                info = l.split()
                target_dist = float(info[0])
                front_dist = float(info[-1]) / 1000
                ang = (float(info[1])) 
                vel_ralative = float(info[2])
                brake_flag = int(info[3])
                B = struct.pack('f', ang)
                # print('tmp: ', type(ang), ang, 'B: ', B, B.hex())
                count=DWritePort(ser_stm,'c'.encode('gbk')+B)
                
                # print(f"向STM32 写入 转向系数 {ang}, count={count} ")
                
                
            except Exception as e:
                print(f'---lidar error! {e}--')
        # print('---begin reading pcap---')
        
        try:
            filesize = os.path.getsize(wifi_path)
            if (filesize == 24):
                print(' PCAP EMPTY!')
            else:
                with open(wifi_path, 'rb') as in_wifi:
                
                    in_wifi.seek(-24, os.SEEK_END)
                    # while in_wifi.read(1) != b"\n":
                    #     in_wifi.seek(-2, os.SEEK_CUR)
                    l = in_wifi.readline()
                    
                    # print(l[-23:])
                    packet_data = l[-23:].decode().strip()
                    # print('read line : ', packet_data)
                    in_wifi.close()
                    
                    packet_data = packet_data.split(' ')
                    # print(packet_data, type(packet_data))
                    source_vel = packet_data[0]
                    cur_seq_num = int(packet_data[1])
                    rel_dist = float(packet_data[2])
                    vel_leader = float(packet_data[3])
                    print(f'read from pcap {source_vel} {cur_seq_num}, {rel_dist}, {vel_leader}')
                    # print(f'cur_seq_num = {cur_seq_num} while prev_seq_num = {prev_seq_num}')
                    if (np.random.rand() < 0.8 and cur_seq_num != prev_seq_num):
                        packet_flag = True
                        queue_flag = True
                        received_num += 1
                        print(f'received packet num: {received_num}')
                        prev_seq_num = cur_seq_num
                    else:
                        missed_packet_num += 1
                        if (missed_packet_num > 5):
                            queue_flag = False
                    
        except Exception as e:
            print(f'---wifi error!--- {e}')
        # print('---end reading pcap---')
        
        if(packet_flag or True):
            # print('---making decision---')
            if role == 'rear':
                
                # print(comm_radius)
                # if (packet_flag == True):
                vel_leader = vel_ralative + vel_cur
                # front_dist = rel_dist
                comm_radius = get_comm_radius(front_dist)
                vel_new = get_vel(vel_leader, front_dist) # get vel_l by communication
                # vel_new = get_vel(vel_ralative + vel_cur, front_dist) # get vel_l by lidar
                # print(vel_new)
                pwm_vel = vel_to_pwm(vel_new)
                # print(pwm_vel)
                if brake_flag == 1:
                # if True:
                    count=DWritePort(ser_stm,'s')
                    print(f"dist:{front_dist} 向STM32 写入 brake , count={count} ")
                elif (front_dist < 3.0 and front_dist > 0.0):
                    # pwm_vel = 300
                    V = struct.pack('f', pwm_vel)
                    print('V: ', V, V.hex())
                    count=DWritePort(ser_stm,'v'.encode('gbk')+V)
                    print(f"dist:{front_dist} 向STM32 写入 pwm_vel {pwm_vel}, count={count} ")
                # print('---end reading lidar---')
                else:
                    count=DWritePort(ser_stm,'a')
                    print(f"dist:{front_dist} 向STM32 写入 a ")
            else:
                comm_radius = get_comm_radius(rel_dist)
                # print(comm_radius)
                # vel_new = get_vel(vel_leader, front_dist) # get vel_l by communication
                # vel_new = get_vel(vel_ralative + vel_cur, front_dist) # get vel_l by lidar
                # print(vel_new)
                # pwm_vel = vel_to_pwm(vel_new)
                # print(pwm_vel)
                if brake_flag == 1:
                # if True:
                    count=DWritePort(ser_stm,'s')
                    print(f"向STM32 写入 brake , count={count} ")
                else:
                    if role == 'lane':
                        op = 'a'
                    else:
                        op = 'f'
                    countc=DWritePort(ser_stm, op)
                    print(f"向STM32 写入 {op}")
            
        time.sleep(0.1)

def ReadPcap(ser_stm, role = 'rear'):
    try:
        threading.Thread(target=read_latest_packet, args=(ser_stm, role, )).start()
    except Exception as e:
        print("---异常---: ", e)




def main(gps_serial_path = "COM2", stm_serial_path = "COM5"):
    ser_stm,ret_stm=DOpenPort(stm_serial_path,115200,None)
    ser_gps,ret_gps=GPSOpenPort(gps_serial_path,115200,None)
    while(ret_stm==True):#判断串口是否成功打开
        count=DWritePort(ser_stm,"a")
        print("向STM32 写入 加速指令 a ")
        print("GPS.kph:", GPS.kph)
        #DReadPort() #读串口数据
        #DColsePort(ser)  #关闭串口
        time.sleep(10)
        count=DWritePort(ser_stm,"d")
        print("向STM32 写入 减速指令 d ")
        print("GPS.kph:", GPS.kph)
        time.sleep(10) 

if __name__=="__main__":
    
    main()