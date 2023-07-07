<h1 align="center">
  <br>
  <b>multi-lidar-calibration</b>
  <br>
  <br>
</h1>
## 说明

本项目fork自[liuzm-slam/multi_lidar_calibration: two single-line lasers use icp to calibrate the relative position relationship (github.com)](https://github.com/liuzm-slam/multi_lidar_calibration)，在此基础上做了一些修改，包括：修改了标定逻辑，更换为pl-icp，添加时间平均等。感谢liuzm-slam的开源。

## 使用

1. 修改`params.yaml`，其中source为准确的雷达
2. 修改`launch`文件，根据需要发布两颗雷达到base_link的先验位姿

## 原理简介

假设A为位姿准确的雷达，B为位姿有误差的雷达，首先得到两个雷达间的坐标变换 $_B^AT$ ，即B相对于A的位姿，用改坐标变换将雷达A转换到B坐标系下，理论上此时两个雷达是绝对对齐的，但由于有误差，此时两颗雷达未对齐。

使用PL-ICP，将坐标变换后的A作为参考坐标系，设此坐标系为C坐标系。通过匹配得到此时B和C的坐标变换 $T_e$ ，该坐标变换即为雷达B的误差，将该坐标变换叠加在雷达B的TF上即可得到B的准确位姿：

$$^O_BT\times T_e$$

原理为自己的理解，如有错误请见谅。

## 缺点
1. 出来的位姿只有两颗雷达的相对位姿是准确的，两颗雷达的实际位姿（和base_link间的）并没有标定

