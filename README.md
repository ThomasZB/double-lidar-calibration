# multi_lidar_calibration
two single-line lasers use icp to calibrate the relative position relationship
![image](https://github.com/liuzm-slam/multi_lidar_calibration/blob/master/img/display.gif)
## 修改
修改了大部分代码逻辑（之前作者的有部分没看明白，后面发现是自己理解错了）
1. 修改了标定逻辑：假设第一个雷达位姿是绝对准确的，调整第二颗雷达让两颗雷达点云重合
## 缺点
1. 使用ICP标定，两个激光重复点云不多时无法标定
2. 出来的位姿只有两颗雷达的相对位姿是准确的，两颗雷达的实际位姿（和base_link间的）并没有标定
## 使用
1. 修改`params.yaml`，其中source为准确的雷达
2. 修改`launch`文件，根据需要发布两颗雷达到base_link的先验位姿


