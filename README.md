# README.md
vison文件夹是主文件，用vscode打开

1.使用
ros2 launch referee_pkg referee_pkg_launch.xml \
    TeamName:="TEAMENAME" \
    StageSelect:=0 \
    ModeSelect:=0
启动裁判系统
2.使用ros2 launch camera_sim_pkg camera.launch.py来启动摄像头
3.利用ros2 run player_pkg TestNode改代码即可启动节点。

