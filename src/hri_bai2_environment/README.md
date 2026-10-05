# Bài 2: môi trường UR3e

Package môi trường dành cho launch UR Humble đã được cung cấp: ros_gz_sim,
sim_ignition:=true, world_file. Không cần sửa package UR.

## Chạy

Đặt thư mục hri_bai2_environment trong ~/workspaces/ur_gz_humble/src/.

```bash
source /opt/ros/humble/setup.bash
cd ~/workspaces/ur_gz_humble
colcon build --packages-select hri_bai2_environment --symlink-install
source install/setup.bash
ros2 launch hri_bai2_environment bai2_sim.launch.py
```

Đóng phiên Gazebo/MoveIt cũ trước khi chạy. Robot ở gốc world theo launch UR.
Bàn thấp có mặt trên z=0.08 m, đặt về phía x dương để không chồng lên đế robot.
Cube cạnh 0.03 m, có khối lượng và collision, không static.
Zone là dấu màu trên bàn, chỉ có visual, không có collision.
Tên zone chưa quy định mapping màu theo MSSV; phải bổ sung mapping đúng đề.
config/scene.json ghi vị trí ban đầu trong world; đây chưa phải state provider.

## Phạm vi hiện tại

Đã kiểm tra cú pháp Python, XML và sự nhất quán kích thước/vị trí.
Chưa chạy ROS/Gazebo trên máy người dùng. Kiểm tra cube nằm trên bàn,
UR3e xuất hiện, controller active trước khi triển khai skills.
Vật trong Gazebo chưa tự xuất hiện trong planning scene MoveIt.
Bước tiếp theo cần đồng bộ collision objects, kiểm tra frame world/base_link,
thêm gripper và triển khai home/pick/place với validator/executor hiện có.
Không chạy gắp–đặt trước khi bổ sung các phần trên.
