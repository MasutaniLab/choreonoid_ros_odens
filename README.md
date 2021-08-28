# 【公開】WRS2020トンネル競技に出場するODENSチーム用のChoreonoidをROS1で使うノード

大阪電気通信大学 升谷研究室  
2021年8月  

## 概要

- WRS2020トンネル競技に出場するODENSチーム用の[choreonoid_ros](https://github.com/choreonoid/choreonoid_ros)．
- [choreonoid_ros](https://github.com/choreonoid/choreonoid_ros)の[2021年1月20日のコミット](https://github.com/choreonoid/choreonoid_ros/commit/fd00249a83cd5c6409360800847ffc1d8cf6ae09)からフォークし，その後，[2021年5月22日のコミット](https://github.com/choreonoid/choreonoid_ros/commit/a5d09da882904095b08483439920ac828c4f7faa)をマージした．
- 一緒にビルドするChoreonoidは，[WRS2020](https://github.com/choreonoid/choreonoid/tree/WRS2020)ブランチを使うこと．
- このリポジトリは公開なので注意すること．

## 参考

- [WRS-TDRRC-2020SG1 RELEASE VERSION](https://github.com/WRS-TDRRC/WRS-TDRRC-2020SG1)
- [Choreonoid開発版ドキュメント「ROSとの連携」](https://choreonoid.org/ja/documents/latest/ros/index.html)

## 導入

- Ubuntu 18.04，ROS Melodicは導入済みとする．

- 端末で以下を実行．
  ```
  sudo apt-get install python-catkin-tools qt5-default libqt5x11extras5-dev qt5-style-plugins ros-melodic-joy libzbar-dev
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
  git clone https://github.com/MasutaniLab/double_arm_around_viewer.git # 非公開リポジトリ
  git clone https://github.com/MasutaniLab/image_viewer_qr.git # 非公開リポジトリ
  choreonoid/misc/script/install-requisites-ubuntu-18.04.sh  
  cd ..    
  catkin config --cmake-args -DBUILD_COMPETITION_PLUGIN=ON -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON -DBUILD_CHOREONOID_EXECUTABLE=OFF -DUSE_PYTHON3=OFF -DCMAKE_BUILD_TYPE=Release -DENABLE_INSTALL_RPATH_USE_LINK_PATH=ON
  catkin build 
  ```

- `~/.bashrc` の最後に以下を追加
  ```
  source ~/wrs_ws/devel/setup.bash
  ```

## 動作確認

- DUALSHOCK4をUSBポートに接続．
- 端末1： 路面にQRコードの入った筒がある環境でDoubleArmのシミュレーション
  - AGXありの場合
    ```
    roslaunch choreonoid_ros_odens qr_a.launch
    ```
  - AGXなしの場合
    ```
    roslaunch choreonoid_ros_odens qr_s.launch
    ```

- 端末2： DUALSHOCK4で操縦
  ```
  roslaunch double_arm_game_controller joy.launch
  ```

- 端末3： 台車固定の前向きカメラ画像の表示
  ```
  roslaunch image_viewer_qr frame_front.launch
  ```

- 端末4： アッパーアーム指先のカメラ画像の表示
  ```
  roslaunch image_viewer_qr handtip.launch
  ```

- 端末5： 疑似鳥瞰画像の表示
  ```
  rosrun double_arm_around_viewer double_arm_around_viewer_node
  ```

- 全て準備ができたら，Choreonoidのウィンドウの緑三角（初期位置からシミュレーション開始）をリックする．

## オリジナルからの改変

- [BodyROSItemをPythonから使えるようにした．](https://github.com/MasutaniLab/choreonoid_ros_odens/commit/e2c889dba408a94d3d245012c79255ee31d1445c)

- [DoubleArmV7用のコントローラ ROSDoubleArmV7Controller](https://github.com/MasutaniLab/choreonoid_ros_odens/blob/odens/src/controller/ROSDoubleArmV7Controller.cpp)

- [BodyROSItemで深度画像を出力できるようにした](https://github.com/MasutaniLab/choreonoid_ros_odens/commit/0f88b74fdf938f487a4769c024e9e85c5b98b43d)

- launchファイルを追加．

## 既知の問題点・TODO

- BodyROSItemでデバイスのオン・オフできるROSのサービスを追加する．