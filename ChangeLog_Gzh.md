# 修改方案

​	目前LK光流法已经在基本图片上通过测试。

​	需要数据：关键帧的关联的地图点->投影为像素坐标->在当前图像上跟踪->删掉跟踪失败的点

​	PnPRansac ：关键帧地图点3D->当前帧跟踪到的2D关键点->3D-2D PnP->得到位姿

​	优化：只能关键帧全局优化，或关键帧局部地图优化。

## 关键帧地图点

```c++
// MapPoints associated to keypoints
std::vector<MapPoint*> mvpMapPoints;
//这个是关键帧关联的地图点，需要
```

```c++
// MapPoint observation functions
void AddMapPoint(MapPoint* pMP, const size_t &idx);
void EraseMapPointMatch(const size_t &idx);
void EraseMapPointMatch(MapPoint* pMP);
void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
std::set<MapPoint*> GetMapPoints();
std::vector<MapPoint*> GetMapPointMatches();
int TrackedMapPoints(const int &minObs);
MapPoint* GetMapPoint(const size_t &idx);
//以上为操作观测到的地图点的操作，可以使用
```

```c++
// KeyPoints, stereo coordinate and descriptors (all associated by an index)
const std::vector<cv::KeyPoint> mvKeys;
const std::vector<cv::KeyPoint> mvKeysUn;
const std::vector<float> mvuRight; // negative value for monocular points
const std::vector<float> mvDepth; // negative value for monocular points
const cv::Mat mDescriptors;
//mvKeys为未去畸变的关键点
//mvKeysUn为去畸变的关键点
```



# Change Log

### V1.0

​	使用光流法跟踪代替了非关键帧间的特征点匹配法跟踪。

#### to do

1. image draw：目前只能在关键帧时候画图，非关键帧不能画，解决这个问题。
2. 提高效率(内存空间、算法速度、准确率)
3. 与原版对比时间、准确率、占用空间等。并且可视化。



### V1.01

​	添加了部分注释，删除了无用代码。

### V1.1

修改了类型转换错误，**增加了g2o优化部分**。

### V1.2

修改了图像显示(还差显示当前关键点的图)，存储了相机和关键帧轨迹，修改了关键帧生成原理(增大了inliners点数阈值)





