"""
    @Author: WenXiaomo(SummerWen-Lab)
    @Date: 2025-07-02
    @Description: 读取下位机数据，记录日志(LocalTest)
"""

# Input dataframe manually

import struct
import json
import time
import os
from datetime import datetime
from collections import OrderedDict

class LaserDataSimulator:
    def __init__(self):
        # 记录程序启动时间作为时间零点
        self.start_time = time.time()
        
        # 创建带时间戳的唯一日志文件名
        self.output_file = self.generate_unique_filename()
        print(f"日志文件: {self.output_file}")
    
    def generate_unique_filename(self):
        """生成带时间戳的唯一文件名，并保存到指定文件夹"""
        # 指定目标文件夹
        target_folder = "laserdata(log)"
    
        # 创建目标文件夹（如果不存在）
        if not os.path.exists(target_folder):
            os.makedirs(target_folder)  # 递归创建目录[1,3](@ref)
    
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = f"laser_data_{timestamp}"
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
            ('TimeOffset', round(relative_time, 3)),  # 相对时间（秒），保留3位小数
            ('SetRpm', set_rpm),
            ('DeltaRpm_Up', delta_rpm_up),
            ('DeltaRpm_Down', delta_rpm_down),
            ('Velo', round(velo, 2))  # 速度保留2位小数
        ])
    
    def save_to_json(self, data_dict):
        """将字典数据保存到JSON文件"""
        with open(self.output_file, 'a') as f:
            json.dump(data_dict, f)
            f.write('\n')  # 每帧数据单独一行
    
    def generate_test_frames(self):
        """生成三组测试数据帧"""
        frames = []
        
        # 第一组测试数据
        frame1 = bytes([
            0x34,               # 帧头
            0xE8, 0x03,         # SetRpm = 1000 (0x03E8)
            0xD0, 0x07,         # DeltaRpm_Up = 2000 (0x07D0)
            0xB8, 0x0B,         # DeltaRpm_Down = 3000 (0x0BB8)
            0xA4, 0x70, 0x45, 0x41  # Velo = 12.34 (0x414570A4)
        ])
        frames.append(("帧1", frame1))
        time.sleep(0.1)  # 模拟时间间隔
        
        # 第二组测试数据
        frame2 = bytes([
            0x34,               # 帧头
            0xDC, 0x05,         # SetRpm = 1500 (0x05DC)
            0xE4, 0x09,         # DeltaRpm_Up = 2500 (0x09E4)
            0xEC, 0x0D,         # DeltaRpm_Down = 3500 (0x0DEC)
            0x8F, 0xC2, 0x63, 0x42  # Velo = 56.78 (0x4263C28F)
        ])
        frames.append(("帧2", frame2))
        time.sleep(0.15)  # 模拟时间间隔
        
        # 第三组测试数据
        frame3 = bytes([
            0x34,               # 帧头
            0x40, 0x1F,         # SetRpm = 8000 (0x1F40)
            0x20, 0x4E,         # DeltaRpm_Up = 20000 (0x4E20)
            0x58, 0x1B,         # DeltaRpm_Down = 7000 (0x1B58)
            0x33, 0x33, 0xB3, 0x42  # Velo = 89.6 (0x42B33333)
        ])
        frames.append(("帧3", frame3))
        
        return frames
    
    def run_simulation(self):
        """运行数据帧模拟测试"""
        print("开始模拟激光雷达数据帧处理...")
        test_frames = self.generate_test_frames()
        
        for name, frame_data in test_frames:
            try:
                # 解析数据帧
                parsed = self.parse_laser_frame(frame_data)
                print(f"{name}解析结果: "
                      f"时间={parsed['TimeOffset']}s, "
                      f"SetRpm={parsed['SetRpm']}, "
                      f"DeltaRpm_Up={parsed['DeltaRpm_Up']}, "
                      f"DeltaRpm_Down={parsed['DeltaRpm_Down']}, "
                      f"Velo={parsed['Velo']}")
                
                # 保存到文件
                self.save_to_json(parsed)
            except ValueError as e:
                print(f"{name}解析错误: {str(e)}")
        
        print(f"模拟完成，数据已保存到 {self.output_file}")
        return test_frames

# 主函数
if __name__ == "__main__":
    # 创建并运行模拟器
    simulator = LaserDataSimulator()
    simulator.run_simulation()
    
    # 打印文件内容预览
    print("\n日志文件内容预览:")
    with open(simulator.output_file, 'r') as f:
        for line in f:
            print(line.strip())