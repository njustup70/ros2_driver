"""
    @Author: WenXiaomo(SummerWen-Lab)
    @Date: 2025-07-02
    @Description: 读取下位机数据，记录日志
"""

import struct
import json
import serial
import time
import os
from collections import OrderedDict
from datetime import datetime

class LaserDataLogger:
    def __init__(self, port, baudrate=115200, timeout=1):
        """
        初始化激光数据记录器
        
        参数:
            port (str): 串口设备路径 (如 'COM3' 或 '/dev/ttyUSB0')
            baudrate (int): 波特率 (默认115200)
            timeout (float): 串口超时时间(秒)
        """
        # 记录程序启动时间作为时间零点
        self.start_time = time.time()
        
        # 初始化串口连接
        self.ser = self.init_serial(port, baudrate, timeout)
        
        # 创建带时间戳的唯一日志文件名
        self.output_file = self.generate_unique_filename()
        print(f"日志文件: {self.output_file}")
    
    def init_serial(self, port, baudrate, timeout):
        """初始化串口连接"""
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            print(f"串口 {port} 已打开 (波特率: {baudrate})")
            return ser
        except serial.SerialException as e:
            print(f"打开串口 {port} 失败: {e}")
            raise
    
    def generate_unique_filename(self):
        """生成带时间戳的唯一文件名，并保存到指定文件夹"""
        # 指定目标文件夹
        target_folder = "shootdata(log)"
    
        # 创建目标文件夹（如果不存在）
        if not os.path.exists(target_folder):
            os.makedirs(target_folder)  # 递归创建目录[1,3](@ref)
    
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = f"shoot_data_{timestamp}"
        extension = ".json"
    
        # 检查文件是否重名并生成完整路径
        counter = 1
        filename = f"{base_name}{extension}"
        file_path = os.path.join(target_folder, filename)  # 关键修改：拼接路径[2,4](@ref)
    
        while os.path.exists(file_path):
            filename = f"{base_name}_{counter}{extension}"
            file_path = os.path.join(target_folder, filename)
        counter += 1
    
        return file_path  # 返回完整路径
    
    def parse_laser_frame(self, frame_data):
        """
        解析激光雷达数据帧
        帧结构: [帧头0x34(1字节)] + [SetRpm(2字节)] + [DeltaRpm_Up(2字节)] + 
                [DeltaRpm_Down(2字节)] + [Velo(4字节)]
        
        参数:
            frame_data (bytes): 原始字节数据(长度应为11字节)
        
        返回:
            dict: 解析后的数据字典
        """
        # 验证数据长度
        if len(frame_data) != 11:
            raise ValueError(f"无效数据长度: 需要11字节, 实际收到{len(frame_data)}字节")
        
        # 验证帧头
        if frame_data[0] != 0x34:
            raise ValueError(f"帧头错误: 应为0x34, 实际为0x{frame_data[0]:02X}")
        
        # 解析三个uint16_t (小端序)
        set_rpm = struct.unpack('<H', frame_data[1:3])[0]         # 字节1-2: SetRpm
        delta_rpm_up = struct.unpack('<H', frame_data[3:5])[0]    # 字节3-4: DeltaRpm_Up
        delta_rpm_down = struct.unpack('<H', frame_data[5:7])[0]  # 字节5-6: DeltaRpm_Down
        
        # 解析float32 (小端序)
        velo = struct.unpack('<f', frame_data[7:11])[0]           # 字节7-10: Velo
        
        # 计算相对时间（从程序启动开始）
        relative_time = time.time() - self.start_time
        
        return OrderedDict([
            ('TimeOffset', relative_time),  # 相对时间（秒）
            ('SetRpm', set_rpm),
            ('DeltaRpm_Up', delta_rpm_up),
            ('DeltaRpm_Down', delta_rpm_down),
            ('Velo', velo)
        ])
    
    def save_to_json(self, data_dict):
        """
        将字典数据保存到JSON文件
        
        参数:
            data_dict (dict): 要保存的数据字典
        """
        with open(self.output_file, 'a') as f:
            json.dump(data_dict, f)
            f.write('\n')  # 每帧数据单独一行
    
    def process_serial_stream(self):
        """
        处理串口数据流，实时解析并保存数据帧
        """
        print("开始接收数据 (按Ctrl+C停止)...")
        buffer = bytearray()
        
        try:
            while True:
                # 读取串口数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                
                # 在缓冲区中查找帧头
                while len(buffer) >= 11:
                    # 查找帧头0x34的位置
                    header_pos = -1
                    for i in range(len(buffer) - 10):
                        if buffer[i] == 0x34:
                            header_pos = i
                            break
                    
                    # 找到帧头且后面有足够数据
                    if header_pos != -1 and len(buffer) - header_pos >= 11:
                        # 提取完整帧数据
                        frame_data = bytes(buffer[header_pos:header_pos+11])
                        
                        try:
                            # 解析数据帧
                            parsed = self.parse_laser_frame(frame_data)
                            print(f"接收到数据帧: Time={parsed['TimeOffset']:.3f}s, "
                                  f"SetRpm={parsed['SetRpm']}, Velo={parsed['Velo']:.2f}")
                            
                            # 保存到文件
                            self.save_to_json(parsed)
                        except ValueError as e:
                            print(f"帧解析错误: {str(e)}")
                        
                        # 从缓冲区移除已处理数据
                        del buffer[:header_pos+11]
                    else:
                        break  # 没有完整帧，等待更多数据
                
                # 短暂休眠减少CPU占用
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n用户中断，停止接收")
        finally:
            self.ser.close()
            print(f"串口已关闭，数据保存至 {self.output_file}")

            
"""
if __name__ == "__main__":
    # 配置串口参数 (根据实际情况修改)
    SERIAL_PORT = 'COM3'      # Windows: 'COMx', Linux: '/dev/ttyUSB0'
    BAUD_RATE = 115200        # 波特率
    
    # 创建并启动日志记录器
    logger = LaserDataLogger(port=SERIAL_PORT, baudrate=BAUD_RATE)
    logger.process_serial_stream()
"""