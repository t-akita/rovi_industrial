# MELFA対応

TORKの[melfa_robotリポジトリ](https://github.com/tork-a/melfa_robot)を参考に  

## インストール  
Melfaドライバを導入
~~~
apt install ros-kinetic-melfa-robot
~~~

## Launch  
Launchする前に、ロボットのアドレスをパラメータの/melfa_robot/robot_ipにセットしておく。
~~~
roslaunch rovi_industrial melfa_rv4f.launch
~~~
