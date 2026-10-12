# 使用方法
mkdir -p g1_nav_ros2_ws/src
cd g1_nav_ros2_ws/src
git clone https://github.com/jie0110/unitree_g1_nav.git
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
