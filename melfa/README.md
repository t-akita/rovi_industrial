# MELFA対応

TORKの[melfa_robotリポジトリ](https://github.com/tork-a/melfa_robot)を参考に  

## インストール  
Melfaドライバを導入
~~~
apt install ros-kinetic-melfa-robot
~~~
melfa_robotリポジトリをclone
~~~
git clone https://github.com/tork-a/melfa_robot.git
~~~
catkin_makeにてBuild

## Launch  
ロボットのアドレスが192.168.1.10とすると
~~~
roslaunch rovi_industrial melfa_rv4f.launch robot_ip:=192.168.1.15
~~~
