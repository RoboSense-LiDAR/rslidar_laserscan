
# Rslidar Laserscan Tool

## 1 Introduction
Extract a horizontal ring of rslidar PointCloud2 and publish it as a LaserScan message.  

For now, only support 10 Hz `RS16/RS32/RSBP/RSHELIOS/RSHELIOS_16P/RS80/RS128`.    

## 2 Compile
```
catkin_make
```

## 3 Run
Please set the right model and the sub_topic in the `rslidar_laserscan.launch`.
```
source devel/setup.bash
roslaunch rslidar_laserscan rslidar_laserscan.launch
```
Launch rviz, subscribe the topic "rslidar_laserscan", and view the laserscan messages.
