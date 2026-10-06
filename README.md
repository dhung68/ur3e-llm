# Bài thực hành 02 — Điều khiển UR3e bằng LLM

## 1. Chuẩn bị

- Ubuntu 22.04 và ROS 2 Humble Desktop.
- Gazebo Fortress, MoveIt 2.
- 9Router đang chạy, có model sử dụng được và key 9Router.

## 2. Tải mã nguồn

```bash
git clone --branch assignments_2 --single-branch \
  https://github.com/dhung68/ur3e-llm.git \
  ~/workspaces/ur_gz_humble_bai2

cd ~/workspaces/ur_gz_humble_bai2
source /opt/ros/humble/setup.bash
```

## 3. Cài dependency

```bash
sudo apt-get update
sudo apt-get install git python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-pil python3-yaml \
  ros-humble-ur-description ros-humble-ur-moveit-config \
  ros-humble-ur-controllers ros-humble-moveit \
  ros-humble-ign-ros2-control ros-humble-ros-gz
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
  ur_simulation_gz hri_ur3e_description \
  hri_bai2_environment ur3_llm_control

source install/setup.bash
```

## 5. Chạy mô phỏng — terminal 1

```bash
cd ~/workspaces/ur_gz_humble_bai2
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=231
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=bai2_submission

ros2 launch hri_bai2_environment bai2_sim.launch.py \
  moveit_launch_rviz:=false
```

Giữ terminal mở, chờ robot và các khối xuất hiện.
Đổi `moveit_launch_rviz:=true` nếu muốn mở thêm RViz.

## 6. Gửi câu lệnh — terminal 2

```bash
cd ~/workspaces/ur_gz_humble_bai2
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=231
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=bai2_submission

ros2 action info /move_action
```

Khi có `Action servers: 1`, chạy:

```bash
ros2 run ur3_llm_control llm_task \
  --prompt-key \
  --command "Arrange all objects according to my student ID"
```

Nhập **key 9Router** khi được hỏi rồi nhấn Enter.
Ký tự không hiện trên màn hình.

MSSV mặc định `23020744`: vàng → A, đỏ → B, xanh dương → C.
Chờ `TASK SUCCESS` và robot về tư thế ban đầu.

Để thử câu khác, thay nội dung `--command`, ví dụ:

```bash
ros2 run ur3_llm_control llm_task \
  --prompt-key \
  --command "Cho khối đỏ vào B"
```

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
ros2 run ur3_llm_control llm_task \
  --prompt-key \
  --model "TEN_MODEL_TRONG_9ROUTER" \
  --command "Cho khối đỏ vào B"
```

Thay `TEN_MODEL_TRONG_9ROUTER` bằng tên trong danh sách model
của router. Model cần được cấu hình và có quyền truy cập.
Có thể thêm `--plan-only` để xem kế hoạch trước khi chạy robot.

Đổi URL hoặc thời gian chờ bằng:

```text
--base-url "URL_9ROUTER" --timeout 60
```

### Thay mã sinh viên

MSSV mặc định là `23020744`. Đổi bằng `--student-id`:

```bash
ros2 run ur3_llm_control llm_task \
  --prompt-key \
  --student-id "MSSV_CUA_BAN" \
  --student-name "Ho ten cua ban" \
  --command "Arrange all objects according to my student ID"
```

Thay mã và họ tên bằng thông tin thực tế.
Hệ thống dùng hai chữ số cuối MSSV để chọn thứ tự sắp xếp
theo quy tắc của bài.

### Chỉ xem kế hoạch

Thêm `--plan-only` để gọi LLM và kiểm tra kế hoạch,
robot không di chuyển:

```bash
ros2 run ur3_llm_control llm_task \
  --prompt-key --plan-only \
  --command "Cho khối đỏ vào B"
```

### Lưu kết quả

`--evidence` không bắt buộc. Muốn lưu kết quả, thêm:

```text
--evidence "$HOME/ur3e_demo_results/bai2-$(date +%Y%m%d-%H%M%S)"
```

Thư mục phải chưa tồn tại. Không có `--evidence`,
chương trình chỉ hiển thị kết quả trên terminal và topic trạng thái.

## 8. Lưu ý vận hành

- Hai terminal phải dùng cùng domain và partition.
- Chỉ chạy một phiên mô phỏng và một chương trình điều khiển.
- Bài 2 yêu cầu cả ba vùng A/B/C trống trước nhiệm vụ.
  Sau lượt gắp–đặt, khởi động lại mô phỏng để thử nhiệm vụ mới.
- Dừng mô phỏng bằng `Ctrl+C` ở terminal 1, chờ kết thúc rồi
  chạy lại lệnh launch. Không dừng khi robot đang mang vật.
- Nếu có `TASK FAILED`, kiểm tra nguyên nhân trước khi chạy tiếp.
- Bản đầy đủ đã chạy nhiệm vụ theo MSSV thành công 10/10 thao tác.
  Bộ mã rút gọn chưa được build/chạy lại độc lập trên máy sạch.
