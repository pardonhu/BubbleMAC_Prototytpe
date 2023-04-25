import serial #导入模块
import threading
import time
import math
import struct
from . import GPS
STRGLO="" #读取的数据
OP="" #acc or dec
BOOL=True  #读取标志位
MAX_SPEED = 6.0
ACC = 0.4
DEC = 0.2
DELTA_TIME = 0.1
TAO = 1
vel_cur = 0.0


def get_vel(vel_relative, dist):
    dist /= 1000
    vel_f = vel_cur
    vel_l = vel_f + vel_relative
    vel_new = min(MAX_SPEED, vel_f + ACC * DELTA_TIME, \
        -DEC*TAO + math.sqrt((DEC*TAO)**2 + vel_l**2 + 2*DEC*dist))
    if vel_new < 0.1:
        vel_new = 0
    print('3 vels: ', MAX_SPEED, vel_f + ACC * DELTA_TIME, \
        -DEC*TAO + math.sqrt((DEC*TAO)**2 + vel_l**2 + 2*DEC*dist))
    print(f'vel_new:{vel_new}, vel_f:{vel_f}, vel_l:{vel_l}, vel_cur:{vel_cur}')
    return min(max(vel_new, 0), 0.9)

def get_comm_radius(dist):
    return dist + 500

def pwm_to_vel(pwm):
    return (pwm - 100) / 200 * 0.9
    

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
                    # print(tmp)
                    vel_cur = pwm_to_vel(int(tmp))
                    
                    if(cur_time - prev_time > 1):
                        print(vel_info, f'read from vel_cur:{tmp} to {vel_cur}')
                        prev_time = cur_time
                        
                
            except Exception as e:
                print('ERROR STM ', STRGLO)
                print(e)
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



def read_latest_packet(ser_stm, wifi_path='/home/hu/Desktop/memory_file_sys/wifi.pcap', \
                       lidar_path = '/home/hu/Desktop/memory_file_sys/lidar_data.txt',packet_len=5):
    while(1):
        # print('---begin reading lidar---')
        with open(lidar_path, 'r') as in_lidar:
            try:
                l = in_lidar.readline()
                print('read line : ', l)
                in_lidar.close()
                
                info = l.split()
                target_dist = float(info[0])
                front_dist = float(info[-1])
                ang = (float(info[1])) 
                vel_ralative = float(info[2])
                brake_flag = int(info[3])
                B = struct.pack('f', ang)
                # print('tmp: ', type(ang), ang, 'B: ', B, B.hex())
                count=DWritePort(ser_stm,'c'.encode('gbk')+B)
                # print('s' + B)
                # count=DWsritePort(ser_stm, B)
                print(f"向STM32 写入 转向系数 {ang}, count={count} ")
                
                comm_radius = get_comm_radius(front_dist)
                print(comm_radius)
                vel_new = get_vel(vel_ralative, front_dist)
                print(vel_new)
                pwm_vel = vel_to_pwm(vel_new)
                print(pwm_vel)
                if brake_flag == 2:
                # if True:
                    count=DWritePort(ser_stm,'s')
                    print(f"向STM32 写入 brake , count={count} ")
                elif front_dist < 2500:
                    V = struct.pack('f', pwm_vel)
                    print('V: ', V, V.hex())
                    count=DWritePort(ser_stm,'v'.encode('gbk')+V)
                    # count=DWritePort(ser_stm,'a')
                    # print('s' + B)
                    # count=DWsritePort(ser_stm, B)
                    print(f"向STM32 写入 pwm_vel {pwm_vel}, count={count} ")
                # print('---end reading lidar---')
                else:
                    count=DWritePort(ser_stm,'a')
                    print(f"向STM32 写入 a ")
            except Exception as e:
                print(f'---lidar error! {e}--')
        # print('---begin reading pcap---')
        # with open(wifi_path, 'rb') as in_wifi:
        #     try:
        #         in_wifi.seek(-packet_len, 2)
        #         tmp = in_wifi.read(packet_len)
        #         print(tmp)
        #         OP=chr(tmp[-5])
        #         count=DWritePort(ser_stm,OP)
        #         print(f"向STM32 写入 加速指令 {OP} ")

        #         in_wifi.close()
        #     except:
        #         print('---wifi error!---')
        # print('---end reading pcap---')
        time.sleep(0.05)

def ReadPcap(ser_stm):
    try:
        threading.Thread(target=read_latest_packet, args=(ser_stm,)).start()
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