# navigation
 navigation注释版_V1.0

### Costmap 代价地图
#### costmap_comon_params.yaml
```YAML
max_obstacle_height: 1		#	最大障碍物高度。超过该高度的障碍物会被忽略？？？？为什么会有高度
footprint: [[0.38, 0.38], [-0.38, 0.38], [-0.38, -0.38], [0.38, -0.38]] 	# 底盘半径
map_type: voxel	# 体素地图

obstacle_layer:
  enabled:              true
  max_obstacle_height:  1
  origin_z:             0.0
  z_resolution:         0.2
  z_voxels:             2
  unknown_threshold:    15
  mark_threshold:       0
  combination_method:   1
  track_unknown_space:  true    
  obstacle_range: 10
  raytrace_range: 10
  origin_z: 0.0
  z_resolution: 0.2
  z_voxels: 2
  publish_voxel_map: false
  observation_sources:  scan 
  scan:
    data_type: LaserScan
    topic: /scan
    sensor_fram: laser
    marking: true
    clearing: true
    min_obstacle_height: -0.5
    max_obstacle_height: 0.5

inflation_layer:
  enabled:              true
  cost_scaling_factor:  8.0  #	成本缩放因子:决定膨胀层的成本衰减速度
  inflation_radius:     0.6	#	障碍物周围的安全区域 

static_layer:
  enabled:              true
```

#### global_costmap_params.yaml++++恢复模式
```YAML
global_costmap:
   global_frame: /map
   robot_base_frame: /base_link
   update_frequency: 2.0	#	把传感器观测到数据添加到地图中的频率
   publish_frequency: 1.0	#地图发送频率，在rviz中显示的频率
   static_map: true		#	是否将map_server发来的地图数据作为初始地图
   # ；false：会提供一张空地图，动态建图用作导航
   transform_tolerance: 1	#TF转换的容忍阈值
   
   plugins:
     - {name: static_layer,            type: "costmap_2d::StaticLayer"}
     - {name: obstacle_layer,          type: "costmap_2d::ObstacleLayer"}
     - {name: inflation_layer,         type: "costmap_2d::InflationLayer"}
```

```YAML
recovery_behaviors:
	-name:'rotate_recovery'	#	旋转清除
	 type:'rotate_recovery/RotateRecovery'
	-name:'reset_recovery'	#	重置行为
	 type:'clear_costamp_recovery/CLearCostmapRecovery'
reset_recovery:
	reset_distance:1.84
	layer_names:["obstacle_layer"]
type需要从这三个行为中选择：'rotate_recovery/RotateRecovery'、'clear_costamp_recovery/CLearCostmapRecovery'、'move_slow_and_clear/MoveSlowAndClear'
```
#### local_costmap_params.yaml
```YAML
local_costmap:
   global_frame: /odom
   robot_base_frame: /base_link
   update_frequency: 2.0	# 局部代价地图的更新频率，一般为激光雷达的扫描频率
   publish_frequency: 1.0
   static_map: false
   rolling_window: true
   width: 4	#	局部代价地图的宽
   height: 4	#	局部代价地图的高
   resolution: 0.05
   transform_tolerance: 0.5
   plugins:
    #- {name: static_layer,        type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer,      type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}
```

### 路径规划器

|      全局规划器|变量名称      |
| ---- | ---- |
|   Navfn   |  navfn/NavfnROS|
|   Global_planner   |   global_planner/GlobalPlanner   |
|   Carrot_planner   |   carrot_planner/CarrotPLanner    |

```YAML
#base_global_planner: "global_planner/GlobalPlanner"
base_global_planner: "navfn/NavfnROS"
base_local_planner: "dwa_local_planner/DWAPlannerROS"
#    "navfn/NavfnROS" 
```
使用Dijkstra or` A*`
```YAML
<param name="GlobalPlanner/use_dijkstra" value="false"/>
<param name="GlobalPlanner/use_grid_path" value="true"/>
```
 #### 全局路径规划器
 #### 局部路径规划器
 RVIZ：先将局部Path的Line Style更改为BillBoards
      显示所有生成的备选轨迹：添加PointCloud2，将话题名称设置为`/move_base/DWAPlannerROS/trajectory_cloud`
动态调参：
rosrun rqt_reconfigure rqt_reconfigure
```YAML
DWAPlannerROS:

  max_vel_x: 0.1 	# 最大x方向参数
  min_vel_x: 0.0 	# 最大y方向参数（设置负数将允许倒车）

  max_vel_y: 0.0	# 差分驱动机器人的最大y方向速度
  min_vel_y: 0.0	# 差分驱动机器人的最大x方向速度


  max_trans_vel: 0.5 	# 最大平移速度
  min_trans_vel: 0.1 	# 最小平移速度
  trans_stopped_vel: 0.1	# 当平移速度小于这个值，停止
  
  acc_lim_trans:2.5		# 最大平移加速度
  acc_lim_x:2.5		# x方向的最大加速度上限
  acc_lim_y: 0.0		# y方向的加速度上限（差分机器人设置为0）
  acc_lim_theta:6.0		# 旋转的加速度上限
  
  max_vel_theta: 1.0 	# 最大旋转速度
  min_vel_theta: -0.01 	# 当平移速度可以忽略时的最小角速度
  trans_stopped_vel: 0.1	# 当旋转速度小于这个值，停止
  
  max_rot_vel: 0.5 		
  min_rot_vel: 0.0  
  rot_stopped_vel: 0.1
  

  yaw_goal_tolerance: 1.0  #(double, default: 0.05) # 目标航向容差
  xy_goal_tolerance: 1.0  #(double, default: 0.10) # 目标xy容差
  latch_xy_goal_tolerance:false # 到达目标容差范围内，停止移动，只旋转航向
	
  # 前向模拟参数
  sim_time: 2.0	# 模拟时间     
  vx_samples: 20       # x方向速度采样数
  vy_samples: 0      # 差分机器人y方向速度采样数
  vtheta_samples: 40 # 旋转速度采样数

  # 轨迹评分参数**
  path_distance_bias: 64.0  #(double, default: 32.0)权重越大小车走的路线越接近全局路径  
  goal_distance_bias: 24.0   #(double, default: 24.0)   接近导航目标点
  occdist_scale: 0.5      #(double, default: 0.325)   控制器避障的权重
  forward_point_distance: 0.325 	# 从机器人到评分点的位置
  stop_time_buffer: 0.5        # 0.2	在碰撞前机器人必须停止的时间长度，留出缓冲空间
  scaling_speed: 0.25          # 缩放机器人速度的绝对值
  max_scaling_factor: 0.2      # 机器人足迹在高速时能缩放的最大系数

  oscillation_reset_dist: 0.1  # 重置振动标志前需要行进的距离

  publish_traj_pc : true	#是否在Rviz里发布轨迹
  publish_cost_grid_pc: true # Rviz里发布代价网格
  global_frame_id: odom	# 基础坐标系

  holonomic_robot: false	# 是否为全向机器人
```
