# 构建
```bash
mkdir -p g1_nav_ros2_ws/src
cd g1_nav_ros2_ws/src
git clone https://github.com/jie0110/unitree_g1_nav.git
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
# 使用
```bash
cd g1_nav_ros2_ws/src
source install/setup.bash
# 全局路径规划+地图发布
ros2 launch global_planner global_planner.launch.py
# 定位
ros2 launch super_lio map_localization.py
# 局部轨迹规划+速度控制
ros2 launch scan_planner run.launch.py 