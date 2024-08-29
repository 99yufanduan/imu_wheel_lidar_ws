# TODO
  - 使用twist 解决ndt退化现象
    看下fast_lio是如何做的
  - ndt_matcher node 有几率挂掉

# progress
  - 2024-08-29 融合了imu和wheel数据得到了pose 和twist。通过了这个pose作为ndt的初始pose进行了匹配

# bug
- 融合ntd pose 和imu-wheel pose时 pose 跳变
  由于传感器的频率不一样，不同算法计算时间不一样，需要同步pose的时间，使用队列来保存pose

- ndt在长直走廊匹配时出现退化现象
  - 与twist 融合

- 欧拉角存在奇异性
  同一个旋转可能有不同的欧拉角，对于室内平地，只需要限制横滚角和俯仰角在$\pi/2，-\pi/2$即可保证唯一性

# note
- 不同传感器数据输入时间不一致，经过算法之后时间变得更不一致，如何进行融合 
  - 一个高频一个低频
    - 使用插值法
    - 数据缓存与延迟处理
    - 消息滤波与时间窗同步
      高频数据通过时间窗滤波
  - 频率相同
    - 时间最近邻匹配
    - TimeDelayKalmanFilter