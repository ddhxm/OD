darknet_ros的安装方法参考上级目录中的readme进行安装，总结为如下步骤：
1.  终端命令行输入如下指令，提取github上的代码：
    cd catkin_ws_fixed/src
    git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
    cd ../
2.  根据GPU算粒更改CMakelists.txt中的Find CUDA:
	例如本机为3070，其对应算粒为-O3-gencode arch=compute_86,code=sm_86
3.  在相应空间中输入catkin_make进行编译，其他权重等下载方法参考上级目录中的README.md
注：若算粒等没有改变，则可以直接将此上级文件darknet_ros复制到工作空间/src中进行编译，若有更改，则根据上述提示进行更改。

目录描述：此darknet_ros文件包含darknet、darknet_ros、darknet_ros_msgs
1.    其中darknet文件夹下为yolo相关的原始文件，包括网络等，此处可以不管，只保证其文件全部都下载完成，否者在编译过程中容易出现找不到文件的情况，若出现cant find某文件，多为在下载代码的过程中没有下载完全，因此在 git clone时务必加上--recursive。
2.    darknet_ros为darknet在ros中节点的相关封装。包括launch文件等，其中config文件中包含ros系统中darknet_ros节点发布和订阅消息的设置，ros.yaml为总的节点发布和订阅设置，因此，当摄像头接入的节点发生变化时，更改此文件中相应的订阅节点名称即可。同时，其发布三个话题，分别为object_detector、bounding_boxes、detection_image，bounding_boxes为检测得到的目标检测框，具体消息形式看darknet_ros_msgs中的消息设置。
3.    darknet_ros_msgs为定义的消息格式，以BoundingBox.msg为例，其分别包含目标框的概率、两角点坐标、类型等，检测得到结果之后，通过发布话题bounding_boxes发布检测结果。
4.    此处主要讲一下darknet_ros目录下的几个py文件。其摄像头和激光雷达的标定转换在roszed_depth.py文件中实现：
	1、此文件中通过订阅摄像头话题消息、激光雷达话题消息、darknet_ros检测框消息，并通过其对应的回调函数进行消息的处理。
	2、通过函数calib（）读入标定文件，进行摄像头和激光雷达的坐标系转换，主要通过检测框内的几个激光雷达点的深度作为此box框的深度。
	注：后期还需进行更改，直接将框内的N个点进行消息发布。此消息发布模块还未完成。因此，此文件目前实现了单个框的深度信息获取，并进行了可视化，弹出test框，显示实时检测的目标框，其中除了含有目标框，还在右上角标注了深度信息。



上述对整个框架目录进行了梳理，后续讲解其在ros上的运行流程：
1.  开启一个新终端输入roscore，启动ros
2.  另开终端，source ~/catkin_ws_fixed/devel/setup.bash(每次在新终端roslaunch前都需输入此命令行)，启动摄像头节点，若使用手机摄像头，则roslaunch android_cam-imu.launch，后期根据实际使用的摄像头，对应进行roslaunch即可。
3.  另开终端，启动激光雷达节点，输入roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar64" frame_id:="Pandar64"
4.  同理，启动darknet_ros节点，roslaunch darknet_ros yolo_v3.launch
5.  同理，在darknet_ros/darknet_ros目录下开启终端，输入python roszed_depth.py，实现深度提取。

注：可使用.sh文件进行整合，此文件在darknet_ros/darknet_ros目录下，命名为fusion_detect.sh，命令行输入 ./fusion_detect.sh替代上述步骤1-4.
