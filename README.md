# tur_amcl_ws

## 经典仓库建图及导航仿真

![仿真截图](frame/simulation.png)

## 部分调参优化说明

- `src/warehouse_navigation_exam/config/slam_mapping.yaml`：优化建图
- `src/warehouse_navigation_exam/config/amcl_localization.yaml`：定位参数优化

### slam-toolbox建图参数
建图精度提高主要是将建图机器人的线速度和角速度减小，0.20 m/s，0.30 rad/s 较为合适


### AMCL 参数调整

增加估计位姿的粒子，用于应对多重复货架场景：

```yaml
max_particles: 5000
min_particles: 1000  # 原值 100
laser_model_type: "likelihood_field_prob" # 筛选光束模式，用于动态避障
do_beamskip: true
```
```yaml
    # 减少雷达激光置信度，增大对里程计的置信度
    alpha1: 0.015
    alpha2: 0.015
    alpha3: 0.015
    alpha4: 0.015
```

实际效果：
将障碍物放置机器人膨胀范围内，放入障碍物，第一次脱困用时1分钟找到路径并启动移动，第二次用时10s找到合适路径，没有出现定位误差大的情况
