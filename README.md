# Bài thực hành 03 — UR3e, camera và LLM

## 1. Chuẩn bị

- Ubuntu 22.04 và ROS 2 Humble Desktop.
- Gazebo Fortress, MoveIt 2; máy hỗ trợ rendering Ogre2.
- 9Router đang chạy, có model sử dụng được và key 9Router.

## 2. Tải mã nguồn

```bash
git clone --branch assignments_3 --single-branch \
  https://github.com/dhung68/ur3e-llm.git \
  ~/workspaces/ur_gz_humble_bai3

cd ~/workspaces/ur_gz_humble_bai3
source /opt/ros/humble/setup.bash
```

## 3. Cài dependency

```bash
sudo apt-get update
sudo apt-get install git python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-pil python3-yaml python3-opencv \
  ros-humble-cv-bridge ros-humble-ur-description \
  ros-humble-ur-moveit-config ros-humble-ur-controllers \
  ros-humble-moveit ros-humble-ign-ros2-control ros-humble-ros-gz \
  libignition-gazebo6-plugins libignition-rendering6-ogre2 \
  libignition-sensors6-rgbd-camera
```

Tải package mô phỏng UR:

```bash
git clone --branch humble --single-branch \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
  src/ur_simulation_gz

git -C src/ur_simulation_gz checkout \
  e49336eb369a3e75fd31753512d4afb3c0c1eb6f
```

Nếu chưa khởi tạo rosdep, chạy `sudo rosdep init` một lần. Sau đó:

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

## 4. Build

```bash
colcon build --symlink-install --packages-select \
  ur_simulation_gz hri_ur3e_description hri_bai2_environment \
  hri_bai3_perception hri_bai3_environment ur3_llm_control

source install/setup.bash
```

## 5. Chạy mô phỏng — terminal 1

```bash
cd ~/workspaces/ur_gz_humble_bai3
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=221
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=bai3_manual_demo221

ros2 launch hri_bai3_environment bai3_sim.launch.py \
  moveit_launch_rviz:=false gazebo_gui:=true
```

Giữ terminal mở, chờ robot và năm khối xuất hiện.
Ban đầu khối xanh dương nằm trong vùng B.
Đổi `moveit_launch_rviz:=true` nếu muốn mở thêm RViz.

## 6. Kiểm tra và gửi lệnh — terminal 2

```bash
cd ~/workspaces/ur_gz_humble_bai3
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=221
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=bai3_manual_demo221

ros2 action info /move_action
ros2 topic info /bai3/camera/image
ros2 topic echo /bai3/table_state --once --full-length
```

Trước khi chạy, cần có:

- `/move_action`: một action server.
- Topic ảnh: một publisher và một subscription.
- `table_state`: nhận đủ năm khối, không có trường `error`.

Chạy demo xử lý vùng B đang bị chiếm:

```bash
ros2 run ur3_llm_control bai3_task \
  --prompt-key \
  --command "Cho khối đỏ vào B" \
  --evidence "$HOME/ur3e_demo_results/bai3-$(date +%Y%m%d-%H%M%S)"
```

Nhập **key 9Router** khi được hỏi rồi nhấn Enter.
Ký tự không hiện trên màn hình.

Robot cần chuyển xanh dương đến chỗ tạm, đặt đỏ vào B
và về tư thế ban đầu. Đợi `TASK SUCCESS` trước thao tác tiếp theo.

## 7. Các tùy chọn

### Key 9Router

`--prompt-key` hỏi key nếu biến `ROUTER_API_KEY` chưa được đặt.
Key nhập bằng cách này chỉ dùng cho lần chạy hiện tại.

Muốn nhập một lần cho nhiều lượt trong cùng terminal:

```bash
read -rsp "9Router key: " ROUTER_API_KEY
echo
export ROUTER_API_KEY
```

Sau đó có thể bỏ `--prompt-key`. Mở terminal mới cần đặt key lại.
Không lưu key vào mã nguồn hoặc đưa lên Git.

### Thay model và URL

Mặc định:

- URL: `http://localhost:20128/v1`
- Model: `gemini/gemini-3.5-flash-lite`
- Thời gian chờ: 45 giây

Đổi model bằng tên được 9Router hỗ trợ:

```bash
ros2 run ur3_llm_control bai3_task \
  --prompt-key \
  --model "TEN_MODEL_TRONG_9ROUTER" \
  --command "Cho khối đỏ vào B" \
  --evidence "$HOME/ur3e_demo_results/bai3-$(date +%Y%m%d-%H%M%S)"
```

Thay `TEN_MODEL_TRONG_9ROUTER` bằng tên trong danh sách model
của router. Model cần được cấu hình và có quyền truy cập.

Đổi URL hoặc thời gian chờ bằng:

```text
--base-url "URL_9ROUTER" --timeout 60
```

### Chỉ xem kế hoạch

Thêm `--plan-only` để lập và kiểm tra kế hoạch, robot không di chuyển:

```bash
ros2 run ur3_llm_control bai3_task \
  --prompt-key --plan-only \
  --command "Cho khối đỏ vào B" \
  --evidence "$HOME/ur3e_demo_results/bai3-plan-$(date +%Y%m%d-%H%M%S)"
```

Bài 3 vẫn cần mô phỏng, camera và MoveIt trong chế độ này
để lấy trạng thái vật và kiểm tra chỗ đặt tạm.

### Lưu kết quả và chụp ảnh

- `--evidence` bắt buộc trong bản Bài 3 hiện tại.
  Thư mục phải chưa tồn tại.
- Thêm `--screenshots` nếu muốn tự chụp cửa sổ khi hoàn thành
  hoặc gặp lỗi. Đây không phải camera nhận diện vật.
- Kết quả lưu ngoài repository, không đưa lên Git.

### Mã sinh viên

Ngữ cảnh mặc định sử dụng MSSV `23020744`.
`bai3_task` hiện chưa có tùy chọn `--student-id` hoặc
`--student-name` như chương trình Bài 2.

## 8. Lưu ý vận hành và giới hạn

- Hai terminal phải dùng cùng domain và partition.
- Chỉ chạy một phiên mô phỏng và một chương trình điều khiển.
  Launch đã mở camera và nhận diện; không chạy perception riêng.
- Dùng “xanh dương” hoặc `blue_cube` để tránh nhầm với xanh lá.
- Để quay lại demo ban đầu, dừng launch bằng `Ctrl+C`,
  chờ kết thúc rồi mở world mới. Không dừng khi đang mang vật.
- Nếu có `TASK FAILED`, kiểm tra lỗi trước khi chạy tiếp.
- Hai world mới đã hoàn thành demo LLM thật 7/7 thao tác.
  Lượt nhiều vật tiếp nối vẫn có lỗi dữ liệu camera cũ;
  hệ thống chưa ổn định trong mọi tình huống.
- Bộ mã rút gọn chưa được build/chạy lại độc lập trên máy sạch.
