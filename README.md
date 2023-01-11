<h1 align="center">
  <br>
  <b>multi-lidar-calibration</b>
  <br>
  <br>
</h1>

两个雷达间的标定
![image](https://github.com/liuzm-slam/multi_lidar_calibration/blob/master/img/display.gif)

## 使用

1. 修改`params.yaml`，其中source为准确的雷达
2. 修改`launch`文件，根据需要发布两颗雷达到base_link的先验位姿

## 原理简介

假设A为位姿准确的雷达，B为位姿有误差的雷达，首先得到两个雷达间的坐标变换 $_B^AT$ ，即B相对于A的位姿，用改坐标变换将雷达A转换到B坐标系下，理论上此时两个雷达是绝对对齐的，但由于有误差，此时两颗雷达未对齐。

使用PL-ICP，将坐标变换后的A作为参考坐标系，设此坐标系为C坐标系。通过匹配得到此时B和C的坐标变换 $T_e$ ，该坐标变换即为雷达B的误差，将该坐标变换叠加在雷达B的TF上即可得到B的准确位姿：

$$^O_BT\times T_e$$

原理为自己的理解，如有错误请见谅。

## 修改

目前将ICP匹配更改为了PLICP，精度更高。
1. 修改了标定逻辑：假设第一个雷达位姿是绝对准确的，调整第二颗雷达让两颗雷达点云重合

## 缺点
1. 出来的位姿只有两颗雷达的相对位姿是准确的，两颗雷达的实际位姿（和base_link间的）并没有标定

