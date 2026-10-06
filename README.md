# Bài thực hành 03 — UR3e, camera RGB-D và LLM

Bộ nộp trên nhánh `assignments_3`, lấy mã từ commit nền
`05b12d27c4a358db15d91b1a26550006368e9193`. README và năm package tự xây là toàn bộ
mã cần nộp; package UR tải riêng. Giữ `.gitignore` và LICENSE; không kèm test,
runner kiểm thử, báo cáo, log/evidence, ảnh debug, cache hoặc build/install.

```text
src/
  hri_ur3e_description/  # UR3e/gripper/TCP, ros2_control và cấu hình
  hri_bai2_environment/  # MoveIt overlay/SRDF/RViz và tài nguyên nền Bài 02
  hri_bai3_environment/  # world 5 cube, camera cố định, launch, scene.json
  hri_bai3_perception/   # OpenCV/RGB-D, quan sát và pose vật trong frame world
  ur3_llm_control/       # prompt/9Router, validator, executor và robot skills
```

Luồng: RGB + depth + CameraInfo → `/bai3/table_state` → context occupancy → LLM
qua 9Router → JSON `home/pick/place` → validator toàn plan → executor tuần tự →
MoveIt/Gazebo → camera xác nhận gắp/giữ/thả và cập nhật scene. Arm và gripper
Bài 03 đều qua MoveIt; PID/gravity/collision/gắp bằng contact vật lý được giữ.
LLM không sinh joint command, trajectory hoặc tọa độ tự do; không teleport/weld.

## Dependency và phiên bản

Dùng **Ubuntu 22.04 Jammy, ROS 2 Humble, Gazebo Fortress (Ignition Gazebo 6)**,
desktop X11 và GPU/rendering hỗ trợ Ogre2. Máy kiểm chứng dùng:

| Thành phần | Phiên bản đã dùng |
|---|---|
| Ubuntu / Python | 22.04.5 / 3.10.12 |
| Ignition Gazebo / Sensors / Rendering | 6.18.0 / 6.9.0 / 6.6.4, backend Ogre2 |
| MoveIt 2 / moveit_msgs | 2.5.10 / 2.2.3 |
| ur_description / ur_moveit_config / ur_controllers | 2.13.0 / 2.14.0 / 2.14.0 |
| ros_gz_bridge, ros_gz_sim / ign_ros2_control | 0.244.26 / 0.7.21 |
| OpenCV / cv_bridge | 4.5.4 / 3.2.1 |
| NumPy / SciPy / Pillow / PyYAML | 1.21.5 / 1.8.0 / 9.0.1 / 5.4.1 |
| UR simulation source | humble, package 0.5.0, commit ghim bên dưới |

Cần ROS desktop, kho apt ROS Humble, colcon/rosdep, xacro, controller_manager,
ros2_controllers, TF2, ROS message/action packages, RViz2, KDL/OMPL.
Gazebo Sensors phải có RGB-D và rendering Ogre2; không dùng Classic/Harmonic
hoặc ROS Jazzy thay phiên bản nền. Không cần YOLO hoặc OpenAI SDK.
Ảnh tùy chọn dùng Xlib/Pillow, `xwininfo`/`xprop`; không dùng ảnh tạo sinh.

## Tải và build trong workspace riêng

```bash
git clone --branch assignments_3 --single-branch \
  https://github.com/dhung68/ur3e-llm.git ~/workspaces/ur_gz_humble_bai3
cd ~/workspaces/ur_gz_humble_bai3
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install git python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-pil python3-yaml python3-opencv x11-utils \
  ros-humble-cv-bridge ros-humble-ur-description ros-humble-ur-moveit-config \
  ros-humble-ur-controllers ros-humble-moveit ros-humble-ign-ros2-control \
  ros-humble-ros-gz libignition-gazebo6-plugins libignition-rendering6-ogre2 \
  libignition-sensors6-rgbd-camera
git clone --branch humble --single-branch \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
  src/ur_simulation_gz
git -C src/ur_simulation_gz checkout e49336eb369a3e75fd31753512d4afb3c0c1eb6f
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select ur_simulation_gz hri_ur3e_description hri_bai2_environment hri_bai3_perception hri_bai3_environment ur3_llm_control --symlink-install
source install/setup.bash
```

ROS Humble phải được cài trước. Nếu chưa khởi tạo rosdep, chạy `sudo rosdep init`
trước `rosdep update`. UR description/MoveIt/controllers dùng bản apt Humble;
chỉ clone UR simulation như trên, không sửa source UR. Không source overlay cũ
Bài 02/Bài 03 vào workspace mới. Quy trình cài trên máy sạch và lệnh build gộp
trên chưa được chạy lại trong lượt đóng gói.

## Launch — terminal 1

Chỉ một phiên Gazebo/RViz và một client điều khiển. Bắt đầu **world mới**, tay
trống; blue ban đầu chiếm zone_b. Kết thúc phiên cũ đúng cây tiến trình khi robot
đã về home/tay trống; không ngắt đột ngột khi đang mang vật.

```bash
cd ~/workspaces/ur_gz_humble_bai3
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=221 IGN_PARTITION=bai3_submission DISPLAY=:0
ros2 launch hri_bai3_environment bai3_sim.launch.py
```

Giữ terminal mở; chờ ba controller ACTIVE, MoveIt sẵn sàng và camera quan sát
đủ năm vật ở home. Camera cố định tại (0,3; −0,5; 0,75) m, nhìn chếch xuống toàn
bàn, 400×300/5 Hz. Chạy trên desktop thật, không Xephyr. Mọi terminal phải dùng
cùng domain/partition; không dùng domain 233 (đã lỗi DDS).

## Gọi LLM thật — terminal 2

9Router phải đang chạy. Key Google được cấu hình trong provider Gemini của
router; chương trình chỉ dùng key **9Router**, không đọc credential store.

```bash
cd ~/workspaces/ur_gz_humble_bai3
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=221 IGN_PARTITION=bai3_submission DISPLAY=:0
ros2 run ur3_llm_control bai3_task --prompt-key --screenshots \
  --base-url http://localhost:20128/v1 \
  --model gemini/gemini-3.5-flash-lite --timeout 45 \
  --command 'Đặt red_cube vào zone_b. Nếu có vật chiếm vùng, chuyển vật đó tới một vị trí trống trước.' \
  --evidence "$HOME/ur3e_demo_results/bai3-$(date +%Y%m%d-%H%M%S)"
```

`--prompt-key` dùng `ROUTER_API_KEY` nếu đã có; nếu chưa có, nhập tại
`9Router key (hidden):` rồi Enter, ký tự không hiện. HTTP 401 nghĩa là xác thực
bị từ chối; nếu biến đang giữ key cũ/sai, `unset ROUTER_API_KEY` rồi nhập kín.
Không in key, đưa key vào command hoặc lưu key trong repo/log/chat.
URL/model/timeout chỉnh bằng các cờ trên. Output demo đặt ngoài bộ mã nộp;
thư mục mỗi lần phải mới. Lỗi ảnh không đổi kết quả điều khiển.

Thêm `--plan-only` để gọi LLM/validate mà không gửi motion goal. Bài 03 plan-only
vẫn cần world/camera/MoveIt vì context lấy từ quan sát thật và kiểm tra IK vùng tạm.
`--listen` thay `--command` nhận một `std_msgs/String` trên `/bai3/command`;
trạng thái ở `/bai3/task_status`. Không dùng plan lưu làm demo chính.

## Kết quả đã xác nhận và phạm vi bộ rút gọn

**Chương trình gốc đã chạy LLM thật trên cấu hình một camera thành công 7/7
skill**, blue_cube → temp_1, red_cube → zone_b, cuối home/tay trống,
**`scene_verified=true`**. Phiên gốc ngày 06/10/2026 có `source="9Router live"`;
đây là luồng tích hợp đầy đủ, không chỉ executor replay.

Plan đã kiểm chứng: `home → pick(blue) → place(blue,temp_1) → home → pick(red) →
place(red,zone_b) → home`. LLM tự chọn từ occupancy camera và ID vùng tạm hợp lệ;
plan mới có thể khác. Kỳ vọng terminal: `USER COMMAND`, `LLM PLAN`, `VALIDATION:
PASS`, SUCCESS từng skill, `POSITION CHECK` rồi `TASK SUCCESS` với 7 skills và
scene_verified cho plan trên. Camera nhìn lại đủ năm vật, held/pending null.

**Bộ rút gọn chỉ được kiểm tra cấu trúc, import, entrypoint, cài đặt tài nguyên,
launch, URDF/SRDF và đường dẫn; chưa build/chạy lại bộ rút gọn hoặc robot.**
Không coi kiểm tra tĩnh là một demo PASS mới. Tài liệu/ảnh/log gốc được giữ ngoài
bộ nộp; chưa tạo video/PDF hoặc tự upload sản phẩm báo cáo.

## Thành phần và giới hạn

Prompt: `src/ur3_llm_control/ur3_llm_control/bai3_prompt.txt`.
`bai3_task.py` nhận lệnh/executor; `planning.py` và `router_response.py` gọi/đọc
9Router; `bai3_state.py` là camera provider/occupancy/validator;
`bai3_skills.py` tái sử dụng `skills.py` qua MoveIt. Perception nằm trong
`src/hri_bai3_perception/hri_bai3_perception/{node,geometry}.py`.
World/config/launch Bài 03 nằm trong `src/hri_bai3_environment`.
Giữ tài nguyên Bài 02 vì launch Bài 03 tái sử dụng MoveIt overlay và mô tả robot.

RGB/depth ghép cùng timestamp và kiểm tra frame. Cube pose phục vụ điều khiển
chỉ lấy từ camera; `/bai3/validation/gazebo_poses` là đối chiếu độc lập, không là
fallback của perception/executor. Vật tĩnh bị che giữ collision từ camera cũ với
margin, không coi là vùng trống. Gắp/giữ/thả phải xác nhận bằng quan sát thật ở
checkpoint; không bắt camera thấy vật liên tục trong mọi chuyển động. Một tư
thế quan sát bổ sung được kiểm tra collision có thể dùng khi cube đang giữ bị
che; không xác nhận được thì dừng. Home sau mỗi place để nhìn lại/sync scene.

Vùng tạm là ID ứng viên, được lọc theo footprint/clearance/mép bàn/IK; không
mapping cố định theo màu hoặc đặt chồng. Validator mô phỏng toàn chuỗi holding/
occupancy trước motion; executor dừng ở lỗi, CLI !=0, không retry mù.

Giới hạn: cube 30 mm/màu riêng/song song trục, extrinsic camera mô phỏng đã biết;
chưa xác nhận yaw tùy ý hoặc camera thật. Green/purple đã nhận diện, chưa kiểm
chứng gắp. Mất vật khi bị che có thể chỉ phát hiện ở checkpoint kế tiếp; chưa
có recovery tổng quát/nhiều client. MoveIt dependency từng lỗi shutdown,
chưa sửa dependency. MSSV 23020744, họ tên còn trống và cần bổ sung trước nộp.
