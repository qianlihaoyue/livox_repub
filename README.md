# livox_repub-ros2
官方livox_driver2驱动livox雷达发出的点云topic有两种，一种是大疆览沃定制的格式CustomMsg格式，另一种是将CustomMsg格式 转换过的pointcloud2格式，参见 Livox雷达驱动程序发布点云格式CustomMsg、PointCloud2、pcl::PointXYZI、pcl::PointXYZINormal解析 现在将转换这部分的代码提取出来，方便 随时使用

```
git clone https://github.com/qianlihaoyue/livox_repub.git

ros2 launch livox_repub launch.py
```

注意：这个包是要先订阅CustomMsg的话题/livox/lidar，然后发布PointCloud2格式的"/livox_pcl0"话题，所以不论是实时驱动livox-driver还是通过bag包发布/livox/lidar，都需要确保有/livox/lidar才能有转换结果
转换后的PointCloud2点云可以通过rviz2显示：
        rviz2
