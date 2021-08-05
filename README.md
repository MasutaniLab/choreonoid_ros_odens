# 【公開】WRS2020トンネル競技に出場するODENSチーム用のChoreonoidをROS1で使うノード

大阪電気通信大学 升谷研究室  
2021年8月  

## 概要

- WRS2020トンネル競技に出場するODENSチーム用の[choreonoid_ros](https://github.com/choreonoid/choreonoid_ros)．
- [choreonoid_ros](https://github.com/choreonoid/choreonoid_ros)の最新ではなく，[2021年1月20日のコミット](https://github.com/choreonoid/choreonoid_ros/commit/fd00249a83cd5c6409360800847ffc1d8cf6ae09)からフォーク．
- 一緒にビルドするChoreonoidは，最新版ではなく[WRS2020](https://github.com/choreonoid/choreonoid/tree/WRS2020)ブランチを使うこと．
- このリポジトリは公開なので注意すること．

## 参考

- [WRS-TDRRC-2020SG1 RELEASE VERSION](https://github.com/WRS-TDRRC/WRS-TDRRC-2020SG1)
- [Choreonoid開発版ドキュメント「ROSとの連携」](https://choreonoid.org/ja/documents/latest/ros/index.html)

## 導入

- Ubuntu 18.04，ROS Melodicは導入済みとする．

- 端末で以下を実行．
  ```
  sudo apt-get install python-catkin-tools qt5-default libqt5x11extras5-dev qt5-style-plugins ros-melodic-joy
  cd ~  
  mkdir -p wrs_ws/src  
  cd wrs_ws  
  catkin init  
  cd src
  git clone -b WRS2020 https://github.com/choreonoid/choreonoid.git  
  git clone https://github.com/MasutaniLab/choreonoid_ros_odens.git  
  git clone https://github.com/WRS-TDRRC/WRS-TDRRC-2020SG1.git choreonoid/ext/WRS2020SG  
  git clone https://github.com/MasutaniLab/choreonoid_ext_ODENS.git choreonoid/ext/ODENS
  git clone https://github.com/MasutaniLab/double_arm_game_controller.git # 非公開リポジトリ
  choreonoid/misc/script/install-requisites-ubuntu-18.04.sh  
  cd ..    
  catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_COMPETITION_PLUGIN=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DCMAKE_CXX_STANDARD=14 -DUSE_PYTHON3=OFF  
  catkin build 
  ```

- `~/.bashrc` の最後に以下を追加
  ```
  source ~/wrs_ws/devel/setup.bash
  ```

## 動作確認

- 端末1
  ```
  roscore
  ```
- 端末2
  - AGXなしの場合
    ```
    cd ~/wrs_ws
    rosrun choreonoid_ros_odens choreonoid devel/share/choreonoid-1.8/ODENS/script/SG1L-DoubleArmV7S-ROS_odens.py
    ```
  - AGXありの場合
    ```
    cd ~/wrs_ws
    rosrun choreonoid_ros_odens choreonoid devel/share/choreonoid-1.8/ODENS/script/SG1L-DoubleArmV7A-ROS_odens.py
    ```
  - 端末3（Dualshock4をUSB接続）
  ```
  roslaunch double_arm_game_controller joy.launch
  ```

## オリジナルからの改変

- [RosBodyItemをPythonから使えるようにした．](https://github.com/MasutaniLab/choreonoid_ros_odens/commit/e2c889dba408a94d3d245012c79255ee31d1445c)

- [DoubleArmV7用のコントローラ ROSDoubleArmV7Controller](https://github.com/MasutaniLab/choreonoid_ros_odens/blob/odens/src/controller/ROSDoubleArmV7Controller.cpp)


## 既知の問題点・TODO

- COLOR_DEPTHのカメラが出力するPointCloud2メッセージは大きいので，別の手段を使う．
