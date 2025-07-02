#!/bin/bash

# 获取脚本的绝对路径
SCRIPT_DIR=$(dirname "$(realpath "$0")")
sudo apt-get install v4l-utils
#煞笔盲文挤占ch340
sudo apt-get remove brltty 

# 进入脚本同目录下的 librealsense 目录
cd "$SCRIPT_DIR/packages/librealsense"
echo "当前目录: $(pwd)"
./scripts/setup_udev_rules.sh

# 安装奥比中光规则
cd "$SCRIPT_DIR/packages/orbbecSDK/misc/scripts"
echo "当前目录: $(pwd)"
sudo ./install_udev_rules.sh

# 安装 wheel_imu 规则
cd "$SCRIPT_DIR/packages/wheel_imu/fdilink_ahrs_ROS2"
echo "当前目录: $(pwd)"
# sudo ./wheeltec_udev.sh
#增加对轮趣imu (fdilink_arhs)的支持
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_FDI_IMU_GNSS"' >/etc/udev/rules.d/my_dev.rules
#安装ms_200规则
# 设置设备别名并设置权限
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="ms200"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="ms200"' >> /etc/udev/rules.d/my_dev.rules

# 下位机串口
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_x64"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_x64"' >> /etc/udev/rules.d/my_dev.rules

# echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_340"' >> /etc/udev/rules.d/my_dev.rules
# echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_340"' >> /etc/udev/rules.d/my_dev.rules

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_ch340"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="serial_ch340"' >> /etc/udev/rules.d/my_dev.rules
#添加ch040imu规则
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    ATTRS{serial}=="26454afeb1ebed1181ec429aa88ea882", \
    MODE:="0777", GROUP:="dialout", SYMLINK+="ch040_imu"' >> /etc/udev/rules.d/my_dev.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", "KERNELS=="3-2:1.0" SYMLINK+="serial_sick"' >> /etc/udev/rules.d/my_dev.rules
service udev reload
sleep 2
service udev restart


