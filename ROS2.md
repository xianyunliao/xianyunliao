# Ubuntu setup
## VMware register <-- broadcom signing
原因：Broadcom账号认证（verification）时间过长
直接下载安装包
## 镜像下载
22.04版本适配 ROS2 Humble
## ROS2 Humble 的配置
### ssh 初始化命令
`sudo apt install openssh-server -y`
`sudo systemctl enable --now ssh`
### 查看虚拟机 ip 地址
`ip a`
### Mac终端
`ssh xianyun@这里替换成刚才查到的IP`
### 虚拟机上，检查 ssh 是否正常启动
`sudo systemctl status ssh`
### 安装依赖
`sudo apt update && sudo apt install curl gnupg lsb-release -y`
`curl -s https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg`
### 写入 ROS2 软件源
`echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`
`sudo apt update`
### 安装
`sudo apt install ros-humble-desktop -y`
### 配置环境变量
`echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
`source ~/.bashrc`
### 要注意检查安装的方式，与homebrew不同
`source ~/.bashrc`
`dpkg -l | grep ros-humble`
`ros2 --help`

## ssh登录检查
`sudo systemctl status ssh`
`ip a`
`ssh xianyun@192.168.240.131`


