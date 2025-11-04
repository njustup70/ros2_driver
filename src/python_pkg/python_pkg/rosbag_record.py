import os
import time
import rclpy
import rclpy.executors
from rclpy.node import Node
import rclpy.qos
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import shutil


class SmartBagRecorder(Node):
    def __init__(self):
        super().__init__('smart_bag_recorder')

        # 声明ROS参数
        self.declare_parameter('max_size_gb', 10.0)  # 最大单个bag大小(GB)
        self.declare_parameter('max_folder_num', 10)  # 最大保存文件夹数量
        self.declare_parameter('mcap', True)  # 是否使用mcap格式
        self.declare_parameter('topic_blacklist', [  # 类似.gitignore的黑名单语法
            # 示例规则
            "*/image_raw*",       # 匹配任何层级下的image_raw开头话题
            "*/compressed_image*",# 匹配任何层级下的compressed_image开头话题
            "/camera/*",          # 精确匹配/camera/下的所有子话题
            "depth_image*",       # 匹配任何depth_image开头的话题
        ])

        # 参数解析
        self.max_size_bytes = int(self.get_parameter('max_size_gb').value * 1024 ** 3)
        self.max_folder_num = self.get_parameter('max_folder_num').value
        self.topic_blacklist = self.get_parameter('topic_blacklist').value
        self.record_dir_root = os.path.abspath(os.path.join(
            os.path.expanduser('~'), "ros2_driver/rosbag_record"))
        self.bag_path = self.prepare_record_path()
        print(f'\033[95m📁 Recording to: {self.bag_path}\033[0m')

        # 初始化writer
        self.writer = SequentialWriter()
        if self.get_parameter('mcap').value:
            storage_options = StorageOptions(uri=self.bag_path, storage_id='mcap')
            converter_options = ConverterOptions('cdr', 'cdr')
            self.writer.open(storage_options, converter_options)
            print(f'\033[95m📦 Using MCAP format for recording\033[0m')
        else:
            storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
            converter_options = ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)
            print(f'\033[95m📦 Using SQLite3 format for recording\033[0m')

        # 初始化变量
        self.subscribers = []
        self.subscribed_topics = set()
        # 预处理黑名单规则（转换为匹配函数）
        self.blacklist_rules = [self._compile_rule(rule) for rule in self.topic_blacklist]

        # 创建定时器
        self.timer = self.create_timer(3.0, self.timer_callback)
        print(f'\033[92m📋 Topic blacklist (gitignore style): {self.topic_blacklist}\033[0m')

    def _compile_rule(self, rule):
        """将.gitignore风格的规则转换为匹配函数"""
        # 分割路径组件（处理类似/namespace/*的规则）
        rule_parts = rule.split('/')
        # 移除空字符串（处理开头或结尾的/）
        rule_parts = [p for p in rule_parts if p]
        
        def matcher(topic):
            topic_parts = topic.split('/')
            topic_parts = [p for p in topic_parts if p]  # 移除空组件
            
            # 规则以/开头（绝对路径匹配）
            if rule.startswith('/'):
                # 必须完全匹配前n个组件
                if len(topic_parts) < len(rule_parts):
                    return False
                for r_part, t_part in zip(rule_parts, topic_parts[:len(rule_parts)]):
                    if not self._wildcard_match(r_part, t_part):
                        return False
                return True
            # 非绝对路径匹配（任意层级）
            else:
                # 检查是否有任意连续的组件匹配规则
                for i in range(len(topic_parts) - len(rule_parts) + 1):
                    match = True
                    for j in range(len(rule_parts)):
                        if not self._wildcard_match(rule_parts[j], topic_parts[i + j]):
                            match = False
                            break
                    if match:
                        return True
                return False
        
        return matcher

    def _wildcard_match(self, pattern, text):
        """处理*通配符的匹配（类似shell的*）"""
        # 分割pattern为不含*的片段
        parts = pattern.split('*')
        # 边缘情况处理
        if not parts:
            return text == ''
        # 检查前缀
        if parts[0] and not text.startswith(parts[0]):
            return False
        # 检查后缀
        if parts[-1] and not text.endswith(parts[-1]):
            return False
        # 检查中间部分
        current = parts[0]
        remaining = text[len(current):] if current else text
        for part in parts[1:-1]:
            if not part:
                continue  # 连续*等同于单个*
            idx = remaining.find(part)
            if idx == -1:
                return False
            remaining = remaining[idx + len(part):]
        return True

    def prepare_record_path(self):
        os.makedirs(self.record_dir_root, exist_ok=True)
        record_dirs = sorted(
            [d for d in os.listdir(self.record_dir_root) if os.path.isdir(os.path.join(self.record_dir_root, d))],
            key=lambda d: os.path.getctime(os.path.join(self.record_dir_root, d))
        )
        if len(record_dirs) >= self.max_folder_num:
            old_path = os.path.join(self.record_dir_root, record_dirs[0])
            print(f'\033[91m📦 Removing oldest folder: {old_path}\033[0m')
            shutil.rmtree(old_path, ignore_errors=True)
        
        file_name = time.strftime("%m-%d-%H-%M", time.localtime())
        file_path = os.path.join(self.record_dir_root, file_name)
        
        if os.path.exists(file_path):
            print(f'\033[93m⚠️ Existing path detected, removing: {file_path}\033[0m')
            shutil.rmtree(file_path, ignore_errors=True)

        return file_path

    def create_callback(self, topic_name):
        def callback(msg):
            try:
                self.writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)
            except Exception as e:
                print(f'\033[91m⚠️ Error writing message from {topic_name}: {e}\033[0m')
        return callback

    def subscribe_topic(self, topic_name, msg_type_str, qos_profile: QoSProfile = None):
        if topic_name in self.subscribed_topics:
            return

        # 检查是否在黑名单中
        for matcher in self.blacklist_rules:
            if matcher(topic_name):
                print(f'\033[93m🔇 Skipping blacklisted topic: {topic_name}\033[0m')
                return

        try:
            msg_type = get_message(msg_type_str)
            if qos_profile is None:
                # 默认QoS配置
                qos_profile = QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT
                )
                topic_info = TopicMetadata(name=topic_name, type=msg_type_str, serialization_format='cdr')
            else:
                topic_info = TopicMetadata(name=topic_name, type=msg_type_str, serialization_format='cdr')

            sub = self.create_subscription(
                msg_type,
                topic_name,
                self.create_callback(topic_name),
                qos_profile
            )
            self.writer.create_topic(topic_info)
            self.subscribers.append(sub)
            self.subscribed_topics.add(topic_name)
            print(f'\033[95m✅ Recording topic: {topic_name} [{msg_type_str}]\033[0m')
        except Exception as e:
            print(f'\033[91m⛔ Failed to subscribe to {topic_name}: {e}\033[0m')

    def check_new_topics(self):
        """检查并订阅新出现的话题"""
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, types in topic_names_and_types:
            if not types:
                continue
            msg_type_str = types[0]
            
            # 特殊处理TF话题的QoS
            if topic_name == '/tf_static':
                qos_profile = QoSProfile(
                    depth=10,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE
                )
                self.subscribe_topic(topic_name, msg_type_str, qos_profile)
            else:
                self.subscribe_topic(topic_name, msg_type_str)

    def check_size_limit(self):
        """检查当前bag大小是否超过限制"""
        if not os.path.exists(self.bag_path):
            return

        total_size = 0
        for root, dirs, files in os.walk(self.bag_path):
            for f in files:
                total_size += os.path.getsize(os.path.join(root, f))
        
        if total_size > self.max_size_bytes:
            print(f'\033[91m🚫 Reached size limit ({self.max_size_bytes / 1024**3:.2f} GB). Stopping recording...\033[0m')
            rclpy.shutdown()

    def timer_callback(self):
        self.check_size_limit()
        self.check_new_topics()


def main(args=None):
    rclpy.init(args=args)
    exe = rclpy.executors.MultiThreadedExecutor()
    recorder = SmartBagRecorder()
    exe.add_node(recorder)
    
    try:
        exe.spin()
    except KeyboardInterrupt:
        print("\033[92m🛑 Recording stopped by user.\033[0m")
    finally:
        exe.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()