# turtlebot3_test_movement

本パッケージは、Gazeboの標準World上に生成したTurtlebot3のモデルに対し、
move_baseを使用して、障害物にぶつからないように動作し続ける指令を送信します。

プログラミング言語はPythonを選択しました。

## 動作確認環境

項目        | 内容
---------- | ------- 
OS          | Ubuntu 20.04.6 LTS
Shell       | Bash
ROS Version | ROS Noetic Ninjemys

また、.bashrcには、ROSに関わる以下の設定が追記されているものとします。

```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## パッケージの導入方法

### Turtlebot3 共通環境の構築

以下リンクの3.1.3, 3.1.4を参考に、Turtlebot3の共通環境を構築してください。
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#install-dependent-ros-packages

### Turtlebot3 シミュレーション環境の構築

以下リンクの6.1.1を参考に、Turtlebot3のシミュレーション環境を構築してください。
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package

### 本パッケージの配置

「(ROSのワークスペース)/src」にこの「turtlebot3_test_movement」フォルダを配置してください。

フォルダ配置後、ROSのワークスペース直下でcatkin_makeを実行してください。

## 実行方法

以下のコマンドを実行してください。

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_test_movement turtlebot3_test_movement.launch
```

以上