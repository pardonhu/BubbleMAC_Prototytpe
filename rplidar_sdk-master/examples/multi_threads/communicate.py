import serial #导入模块
import threading
import time
import math
import struct
import numpy as np
from . import GPS
cur_time = time.strftime("%d-%H%M%S")
STRGLO="" #读取的数据
OP="" #acc or dec
BOOL=True  #读取标志位
MAX_SPEED = 6.0
ACC = 1
DEC = 0.2
vel_max = [200, 300, 400, 500]
DELTA_TIME = 0.1
TAO = 1
front_dist = 3
rel_dist = 1
tx_dist = 1.5
vel_leader = 0.0
vel_cur = 0.0
slot = 0
queue_flag = False
prev_seq_num, cur_seq_num = -1, 0
received_num = 0
import os
home_dir = os.environ['HOME']
vel_log = open(f'{home_dir}/Desktop/memory_file_sys/vel_log/{cur_time}.txt', 'w')
stm_log = open(f'{home_dir}/Desktop/memory_file_sys/stm_log/{cur_time}.txt', 'w')

def get_vel(vel_l, dist, args):
    # dist /= 1000
    dist = max(dist - 0.5, 0.0)
    vel_f = vel_cur
    vel_l = min(max(vel_l, 0), 3)
    v3 = -DEC*args.RO*(TAO+DELTA_TIME) + math.sqrt((DEC*args.RO*(TAO+DELTA_TIME))**2\
            + args.RO*vel_l**2 + 2*DEC*args.RO*dist)
    vel_new = min(MAX_SPEED, vel_f + ACC * DELTA_TIME, v3)
    # if vel_new < 0.1:
    #     vel_new = 0
    
    vel_new = max(vel_new, 0)
    # print('3 vels: ', MAX_SPEED, vel_f + ACC * DELTA_TIME, v3)
    vel_info = f'vel_info: {vel_f + ACC * DELTA_TIME} {v3} {dist} {vel_new} {vel_f} {vel_l} {vel_cur}\n'
                
    vel_log.write(vel_info)
    vel_log.flush()
    print('dist:%.3f, vel_new:%.3f, vel_f:%.3f, vel_l:%.3f, vel_cur:%.3f'%(dist, vel_new, vel_f, vel_l, vel_cur))
    return vel_new

def get_comm_radius(dist):
    return dist + 500

def pwm_to_vel(pwm):
    ret = (pwm - 100) / 200.0 * 0.9
    return max(ret, 0)
    

def vel_to_pwm(pwm, args):
    return min(pwm / 0.9 * 200 + 100, vel_max[args.gear])

#读数代码本体实现
def ReadData(ser):
    global STRGLO, BOOL, vel_cur
    
    # 循环接收数据，此为死循环，可用线程实现
    cur_time = time.time()
    prev_time = cur_time
    while BOOL:
        if ser.in_waiting:
            STRGLO = ser.read(ser.in_waiting)
            for bi in STRGLO:
                if (bi <= ord('z') and bi >= ord('a')) or \
                    (bi <= ord('Z') and bi >= ord('A')) or \
                        (bi <= ord('9') and bi >= ord('0')) or\
                            bi in [ord(':'), ord('\t'), ord('\n'), ord('!'), ord('\r')]:
                    stm_log.write(chr(bi))
                else:
                    stm_log.write(' ' + str(hex(bi)) + ' ')
                
            # stm_log.write(''.join(str(STRGLO.hex())))
            stm_log.write('\n')
            stm_log.flush()
            length = len(STRGLO)
            for i, byt in enumerate(STRGLO):
                if chr(byt) == 'v':
                    try:
                        vel_info = STRGLO[i:i+7].decode('gbk')

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
                                pass
                                # tmp = 100
                            # print(tmp)
                            vel_cur = pwm_to_vel(int(tmp))
                            vel_log.write(f'vel_cur: {vel_cur}\n')
                            vel_log.flush()
                            if(cur_time - prev_time > 1):

                                print(vel_info, f'read from vel_cur:{tmp} to {vel_cur}')
                                prev_time = cur_time
                            
                        
                    except Exception as e:
                        pass
                        print('ERROR STM ', STRGLO)
                        print(e)
                        # vel_cur = 0
            # elif chr(STRGLO[0]) != 'c':
            #     print('foooooooooooooooooo', STRGLO.hex(), STRGLO)

                


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
home_dir = os.environ['HOME']

def read_latest_packet(ser_stm, args, wifi_path=f'{home_dir}/Desktop/memory_file_sys/wifi_log/{cur_time}.pcap', \
                       lidar_path = f'{home_dir}/Desktop/memory_file_sys/lidar_data.txt', packet_len=50):
    global front_dist, vel_cur, prev_seq_num, cur_seq_num, rel_dist, vel_leader, received_num, queue_flag, tx_dist
    missed_packet_num = 0
    continuous_receive_num = 0
    count=DWritePort(ser_stm, f'a{args.gear}'.encode('gbk'))
    print(f"dist:{front_dist}  向STM32 写入 {args.gear} ")
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
                xor_check = B[0] ^ B[1] ^ B[2] ^ B[3] 
                # print('tmp: ', type(ang), ang, 'B: ', B, B.hex())
                input = 'c'.encode('gbk')+B+xor_check.to_bytes(1, 'big')
                count=DWritePort(ser_stm, input)
                # stm_log.write(f" 向STM32 写入 转 向系数 {ang}, {input.hex()}\n")
                # print(f" 向STM32 写入 转 向系数 {ang}, count={count} ")
                
                
            except Exception as e:
                print(f'---lidar error! {e}--')
        # print('---begin reading pcap---')
        time.sleep(0.05)
        try:
            
            cur_seq_num += 1
            # print(f'read from pcap {source_vel} {cur_seq_num}, {rel_dist}, {vel_leader}')
            # print(f'cur_seq_num = {cur_seq_num} while prev_seq_num = {prev_seq_num}')
            if (np.random.rand() < 0.8):
                packet_flag = True
                
                received_num += 1
                missed_packet_num == 0
                continuous_receive_num += 1
                if continuous_receive_num > 1:
                    queue_flag = True
                print(f'received packet num: {received_num}')
            else:
                continuous_receive_num = 0
                missed_packet_num += 1
                if (missed_packet_num > 5):
                    queue_flag = False
            
        except Exception as e:
            print(f'---wifi error!--- {e}')
        # print('---end reading pcap---')
        
        if(packet_flag or True):
            # print('---making decision---')
            if args.role == 'rear':
                
                # print(comm_radius)
                # if (packet_flag == True):
                vel_leader = vel_ralative + vel_cur
                # front_dist = rel_dist
                tx_dist = front_dist
                comm_radius = get_comm_radius(front_dist)
                vel_new = get_vel(vel_leader, front_dist, args) # get vel_l by communication
                # vel_new = get_vel(vel_ralative + vel_cur, front_dist) # get vel_l by lidar
                # print(vel_new)
                pwm_vel = vel_to_pwm(vel_new, args)
                # print(pwm_vel)
                if brake_flag == 1:
                # if True:
                    
                    input = 's'.encode('gbk')
                    count=DWritePort(ser_stm, input)
                    print(f"dist:{front_dist}  向STM32 写入 brake , input={input.hex()} ")
                    stm_log.write(f"dist:{front_dist}  向STM32 写入 brake, {input.hex()}\n")
                elif (front_dist < 2.5 and front_dist > 0.01):
                    # pwm_vel = 300
                    pwm_vel = int(pwm_vel)
                    V = struct.pack('f', pwm_vel)
                    xor_check = V[0] ^ V[1] ^ V[2] ^ V[3] 
                    print('V: ', V, V.hex(), xor_check)
                    input = 'b'.encode('gbk')+V+xor_check.to_bytes(1, 'big')
                    count=DWritePort(ser_stm, input)
                    # count=DWritePort(ser_stm,'C')
                    # count=DWritePort(ser_stm, f'a{args.gear}'.encode('gbk'))
                    stm_log.write(f"dist:{front_dist}  向STM32 写入 pwm_vel{pwm_vel}, {input.hex()}\n")
                    print(f"dist:{front_dist}  向STM32 写入 pwm_vel {pwm_vel}, count={count} ")
                # print('---end reading lidar---')
                else:
                    input = f'a{args.gear}'.encode('gbk')
                    count=DWritePort(ser_stm, input)
                    stm_log.write(f"dist:{front_dist}  向STM32 写入 a{args.gear}, {input.hex()}\n")
                    print(f"dist:{front_dist}  向STM32 写入 a{args.gear} ")
            else:
                comm_radius = get_comm_radius(rel_dist)
                tx_dist = rel_dist
                # print(comm_radius)
                vel_new = get_vel(vel_leader, front_dist, args) # get vel_l by communication
                # vel_new = get_vel(vel_ralative + vel_cur, front_dist) # get vel_l by lidar
                # print(vel_new)
                # pwm_vel = vel_to_pwm(vel_new)
                # print(pwm_vel)
                if brake_flag == 1:
                # if True:
                    
                    input = 's'.encode('gbk')
                    count=DWritePort(ser_stm, input)
                    print(f"dist:{front_dist}  向STM32 写入 brake , input={input.hex()} ")
                    stm_log.write(f"dist:{front_dist}  向STM32 写入 brake, {input.hex()}\n")
                else:
                    if args.auto_stop == 0:
                        op = 'a' + str(args.gear)
                    else:
                        op = 'f'
                    countc=DWritePort(ser_stm, op)
                    print(f" 向STM32 写入 {op}")
                    stm_log.write(f" 向STM32 写入 {op} {op.encode('gbk').hex()}\n")
        stm_log.flush()
        time.sleep(0.05)

def ReadPcap(ser_stm, args, wifi_path):
    try:
        threading.Thread(target=read_latest_packet, args=(ser_stm, args, wifi_path, )).start()
    except Exception as e:
        print("---异常---: ", e)




def main(gps_serial_path = "COM2", stm_serial_path = "COM5"):
    ser_stm,ret_stm=DOpenPort(stm_serial_path,115200,None)
    ser_gps,ret_gps=GPSOpenPort(gps_serial_path,115200,None)
    while(ret_stm==True):#判断串口是否成功打开
        count=DWritePort(ser_stm,"a")
        print(" 向STM32 写入 加速指令 a ")
        print("GPS.kph:", GPS.kph)
        #DReadPort() #读串口数据
        #DColsePort(ser)  #关闭串口
        time.sleep(10)
        count=DWritePort(ser_stm,"d")
        print(" 向STM32 写入 减速指令 d ")
        print("GPS.kph:", GPS.kph)
        time.sleep(10) 

if __name__=="__main__":
    
    main()
    