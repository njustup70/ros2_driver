"""_summary_
先过滤黑名单如果有白名单则过滤白名单
"""
import rclpy
from rclpy.node import Node
import yaml,os,glob
class bag_play_node(Node):
    def __init__(self):
        super().__init__('bag_play_node')
        self.declare_parameter('filter_debug',False) #是否开启定位调试
        self.declare_parameter('rosbag_root_path','/home/Elaina/ros2_driver/bag_play') #rosbag的根目录
        self.declare_parameter('rate',1) #播放的速率
        self.declare_parameter('loop',True) #是否循环播放
        # self.declare_parameter("tf_debug",False)
        # self.declare_parameter('tf_static_debug',False)
        self.whitelist=[] #播放的白名单
        self.typewhitelist=[] #话题类型的白名单
        self.blacklist=["/tf_static"] #播放的黑名单
        self.playlist=[] #播放的列表
        self.filteredList=[]#被过滤的话题
        self.rosbag_path=''
        self.yaml_path=''
        self.find_db_and_yaml()
        self.active_whitelist=True #是否激活白名单
        if self.get_parameter('filter_debug').value:
            self.whitelist=['/tf']
            self.typewhitelist=['std_msgs/msg/String','geometry_msgs/msg/Twist']
            self.blacklist+=['/vel_predict','/sick/vel']
        if len(self.whitelist)==0 or len(self.typewhitelist)==0:
            print(f'\033[95m 不启动白名单] \033[0m')
            self.active_whitelist=False
        else:
            print(f'\033[95m 启动白名单: {self.whitelist} \033[0m')
            print(f'\033[95m 启动类型白名单: {self.typewhitelist} \033[0m')
    def find_db_and_yaml(self):
        """查找rosbag的db3和yaml文件"""
        rosbag_root_path = self.get_parameter('rosbag_root_path').value
        folders = [d for d in os.listdir(rosbag_root_path) if os.path.isdir(os.path.join(rosbag_root_path, d))]
        if folders:
            first_folder = folders[0]  # 取第一个文件夹
            rosbag_path = os.path.join(rosbag_root_path, first_folder)

            # 查找 .db3 文件
            db3_files = glob.glob(os.path.join(rosbag_path, "*.db3"))
            yaml_files = glob.glob(os.path.join(rosbag_path, "*.yaml"))
            if db3_files:
                self.rosbag_path=db3_files[0]
            if yaml_files:
                self.yaml_path=yaml_files[0]
                
        else:
            print("没有找到文件夹")
        if self.rosbag_path=='' or self.yaml_path=='':
            FileNotFoundError("没有找到db3或yaml文件")
    def add_topic(self,topic_name:str,topic_type:str=None):
        """添加一个话题到播放列表"""
        if topic_name in self.blacklist:
            print(f'\033[91m 话题 {topic_name} 在黑名单中，无法添加 \033[0m')
            self.filteredList.append(topic_name)
            return
        if topic_name not in self.playlist : #如果不在播放列表中
            if (self.active_whitelist) : #如果激活了白名单
                if (topic_name not in self.whitelist) and (topic_type not in self.typewhitelist) : #如果不在白名单或类型白名单中,或者在黑名单中
                    self.filteredList.append(topic_name)
                else:
                    self.playlist.append(topic_name)
                    print(f'\033[95m 播放话题: {topic_name} \033[0m')
            elif not self.active_whitelist: #如果没有激活白名单
                self.playlist.append(topic_name)
                print(f'\033[95m 播放话题: {topic_name} \033[0m')
            # else:
                # self.filteredList.append(topic_name) #增加到被过滤列表
    def yaml_to_playlist(self):
        """从yaml文件中读取话题列表"""
        
        if not os.path.exists(self.yaml_path):
            print(f'\033[91m yaml文件不存在: {self.yaml_path} \033[0m')
            return

        with open(self.yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            # 注意这里先进入 rosbag2_bagfile_information
            info = data.get('rosbag2_bagfile_information', {})
            topics = info.get('topics_with_message_count', [])

            if topics:
                for topic in topics:
                    topic_name = topic['topic_metadata']['name']
                    topic_type = topic['topic_metadata']['type']
                    self.add_topic(topic_name, topic_type)
            else:
                print(f'\033[91m yaml文件中没有找到 topics_with_message_count 字段 \033[0m')
    def playbag(self):
        """_summary_
        调用ros2 bag play 播放并remap 被过滤的话题
        """
        if not self.playlist:
            print(f'\033[91m 没有可播放的话题 \033[0m')
            return
        cmd = ['ros2', 'bag', 'play', self.rosbag_path, '--rate', str(self.get_parameter('rate').value)]
        if self.get_parameter('loop').value:
            cmd.append('--loop')
        # for topic in self.playlist:
        #     cmd.append('--remap')
        #     cmd.append(f'{topic}:={topic}')
        cmd.append('--remap')
        for topic in self.filteredList:
            
            cmd.append(f'{topic}:={topic}/filtered')
        print(f'\033[95m 正在播放: {" ".join(cmd)} \033[0m')
        os.system(" ".join(cmd))
def main(args=None):
    rclpy.init(args=args)
    node = bag_play_node()
    node.yaml_to_playlist()
    if len(node.playlist) > 0:
        node.playbag()
    else:
        print(f'\033[91m 没有可播放的话题 \033[0m')
    rclpy.shutdown()
if __name__ == '__main__':
    main()