# Ubuntu setup
## VMware register <-- broadcom signing
原因：Broadcom账号认证（verification）时间过长
直接下载安装包
## 镜像下载
22.04版本适配 ROS2 Humble
multipass安装也可以
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

## ⚠️ssh登录检查
`sudo systemctl status ssh`
`ip a`
`ssh xianyun@192.168.240.131`

## 辅助：Mac + XQuartz SSH 远程运行 ROS2 Humble turtlesim 完整步骤
### 安装 XQuartz（Mac 上的 X11 图形转发服务）
`brew install --cask xquartz`
在terminal运行
`ssh -Y xianyun@192.168.240.131`
加载ROS2环境
`source /opt/ros/humble/setup.bash`
`export QT_X11_NO_MITSHM=1`
#### 图形显示界面
`ros2 run turtlesim turtlesim_node`
#### 图形操作界面
`ros2 run turtlesim turtle_teleop_key`
### Ubuntu 虚拟机端操作
修改 ssh 配置文件
`sudo nano /etc/ssh/sshd_config`

`X11Forwarding yes`
`X11UseLocalhost no`
重启 ssh 服务
`sudo systemctl restart sshd`
安装图形依赖包（防止缺少库）
`sudo apt update`
`sudo apt install x11-apps libxcb-cursor0 ros-humble-turtlesim`

## rqt使用
参考海龟的配置环境
`ssh -Y xianyun@192.168.240.131`
`source /opt/ros/humble/setup.bash`
`rqt`
### 使用插件
“Plugins” --> “Services” --> “Service caller”
