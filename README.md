# Bài thực hành 02 — UR3e và LLM qua 9Router

Bộ mã lấy từ nhánh `assignments_2`, commit
`440c5757ad8e9a4fdbf78158df10855421c3e384`. Chỉ gồm README và ba package tự xây;
không kèm package UR, test, báo cáo, log/evidence hoặc output build/install.

```text
src/
  hri_ur3e_description/  # UR3e + gripper, TCP, ros2_control/config
  hri_bai2_environment/  # world, scene.json, launch, SRDF, MoveIt/RViz config
  ur3_llm_control/       # prompt, 9Router client, validator, executor, skills
```

Luồng: câu lệnh → 9Router → JSON `home/pick/place` → kiểm tra toàn plan →
MoveIt/skills → Gazebo → xác nhận giữ/thả từ pose vật thật và cập nhật scene.
LLM không sinh joint command/trajectory. Gắp bằng lực/contact vật lý; attachment
chỉ phục vụ planning scene. Bộ này không có camera hoặc logic Bài 03.

## Dependency và phiên bản

Dùng **Ubuntu 22.04 Jammy, ROS 2 Humble, Gazebo Fortress (Ignition Gazebo 6)**;
không dùng Gazebo Classic/Harmonic hoặc ROS Jazzy thay thế. Máy kiểm chứng dùng:

| Thành phần | Phiên bản đã dùng |
|---|---|
| Ubuntu / Python | 22.04.5 / 3.10.12 |
| Ignition Gazebo | 6.18.0 |
| MoveIt 2 | 2.5.10; moveit_msgs 2.2.3 |
| ur_description / ur_moveit_config / ur_controllers | 2.13.0 / 2.14.0 / 2.14.0 |
| ros_gz_bridge, ros_gz_sim | 0.244.26 |
| ign_ros2_control | 0.7.21 |
| UR simulation source | nhánh humble, package 0.5.0, commit bên dưới |
| NumPy / SciPy / Pillow / PyYAML | 1.21.5 / 1.8.0 / 9.0.1 / 5.4.1 |

Cần ROS desktop đã cài, kho apt ROS Humble đã cấu hình, colcon, rosdep, xacro,
controller_manager/ros2_controllers, KDL/OMPL, TF2, ROS message/action packages,
RViz2. `package.xml` khai báo dependency ROS; các thư viện Python dùng apt,
không cần OpenAI SDK. Chụp ảnh tùy chọn cần desktop X11, Xlib, `xwininfo`, `xprop`.

## Tải package UR và build

Clone nhánh nộp vào workspace riêng (bên trong có `src/`):

```bash
git clone --branch assignments_2 --single-branch \
  https://github.com/dhung68/ur3e-llm.git ~/workspaces/ur_gz_humble_bai2
```

Dùng terminal mới, không source overlay Bài 03.
`ur_description`, `ur_moveit_config`, `ur_controllers` dùng bản apt Humble;
`ur_simulation_gz` tải từ repository chính thức, giữ nguyên source:

```bash
cd ~/workspaces/ur_gz_humble_bai2
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install git python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-pil python3-yaml x11-utils \
  ros-humble-ur-description ros-humble-ur-moveit-config ros-humble-ur-controllers \
  ros-humble-moveit ros-humble-ign-ros2-control ros-humble-ros-gz
git clone --branch humble --single-branch \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
  src/ur_simulation_gz
git -C src/ur_simulation_gz checkout e49336eb369a3e75fd31753512d4afb3c0c1eb6f
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select ur_simulation_gz hri_ur3e_description hri_bai2_environment ur3_llm_control --symlink-install
source install/setup.bash
```

Nếu chưa từng khởi tạo rosdep, chạy `sudo rosdep init` trước `rosdep update`.
Trên máy đã có dependency/UR simulation trong chính workspace này, chỉ build lại
ba package tự xây. Không tải các nhánh UR mặc định dành cho distro mới hơn.
Quy trình cài trên máy sạch và lệnh build gộp bốn package chưa chạy lại ở lượt
đóng gói; các phiên bản/phần mềm nền và ba package đã được dùng ở bản đầy đủ.

## Launch — terminal 1

Chỉ chạy một world/client; cần world mới với ba zone trống. Trước đổi phiên,
xử lý grasp còn dở; không ngắt khi robot đang mang vật.

```bash
cd ~/workspaces/ur_gz_humble_bai2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=231 IGN_PARTITION=bai2_submission DISPLAY=:0
ros2 launch hri_bai2_environment bai2_sim.launch.py moveit_launch_rviz:=true
```

Giữ terminal mở; chờ ba controller ACTIVE, MoveIt/action và pose/joint states
sẵn sàng. Gazebo/RViz chạy trên desktop thật, không Xephyr. Mọi terminal phải dùng
cùng `ROS_DOMAIN_ID`/`IGN_PARTITION`; không dùng domain 233 (đã lỗi DDS).

## Gọi LLM thật — terminal 2

9Router cần đang chạy. Key Google cấu hình trong provider Gemini của router;
chương trình **chỉ dùng key 9Router**, không đọc credential store.
URL/model/timeout được chỉnh bằng các cờ CLI sau:

```bash
cd ~/workspaces/ur_gz_humble_bai2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=231 IGN_PARTITION=bai2_submission DISPLAY=:0
ros2 run ur3_llm_control llm_task --prompt-key \
  --base-url http://localhost:20128/v1 \
  --model gemini/gemini-3.5-flash-lite --timeout 45 \
  --command 'Arrange all objects according to my student ID'
```

Nếu `ROUTER_API_KEY` chưa có, nhập key 9Router ở `9Router key (hidden):`, Enter;
ký tự không hiện. Nếu biến cũ gây HTTP 401, `unset ROUTER_API_KEY` rồi chạy lại
để nhập kín. Không đưa key thật vào command, README, repo, log hoặc chat.
`--evidence` là **tùy chọn cho `llm_task`**, cả khi chạy task, `--plan-only`
hoặc `--serve`. Lệnh trên chỉ hiển thị kết quả ở terminal và topic status:
không tạo thư mục, log, JSON hoặc ảnh. Backend ghi log file ROS của client cũng
được tắt; log do launch Gazebo/MoveIt riêng sinh ra không thuộc tùy chọn này.
Khi không có evidence, `--screenshots` không chụp ảnh. Các kiểm tra plan,
collision, gắp/giữ, zone trống và vị trí cuối vẫn chạy đầy đủ.

Muốn lưu như trước, thêm các tham số sau vào lệnh trên; thư mục phải chưa tồn tại
và nằm **ngoài bộ mã nộp**:

```bash
--evidence "$HOME/ur3e_demo_results/bai2-$(date +%Y%m%d-%H%M%S)" --screenshots
```

Có evidence: lưu `task.log`, `router_response.json`, `plan.json`, `task.json` và
ảnh hoàn thành/lỗi khi có `--screenshots`; thiếu ảnh không đổi kết quả motion.
Lỗi mạng, JSON hoặc skill trả mã thoát khác 0 và không tự retry chuyển động.

MSSV mặc định `23020744`, P=2: yellow_cube → zone_a, red_cube → zone_b,
blue_cube → zone_c. Kỳ vọng `USER COMMAND`, `LLM PLAN`, `VALIDATION: PASS`,
SUCCESS từng skill, kiểm tra vị trí cuối rồi `TASK SUCCESS`, robot home/tay trống.
Bản đầy đủ đã xác nhận luồng LLM thật MSSV **10/10 skills, scene_verified=true**.
Plan do LLM sinh có thể khác thứ tự, vẫn phải được validator chấp nhận.
Họ tên còn trống: bổ sung `--student-name 'Họ tên đầy đủ của bạn'` trước khi nộp.

Demo cơ bản trên **world mới khác**: thay `--command` bằng
`'Hãy gắp khối đỏ, đặt vào vùng B, rồi về home.'`.
Chỉ lập/validate plan, không motion: thêm `--plan-only`; chế độ này không cần
Gazebo/MoveIt. Câu cơ bản đã được LLM lập plan và robot thực hiện bằng replay;
chưa xác nhận một lượt LLM mới → robot cho câu này.

## Thành phần và giới hạn

`llm_task` là node `ur3_llm_control`; có thể dùng `--serve` nhận
`std_msgs/String` trên `/ur3_llm_control/command`, trạng thái ở
`/ur3_llm_control/status`. `run_skills` gọi home/pick/place không qua LLM.
Ở chế độ serve, chỉ dùng một client, cùng domain/partition của launch:

```bash
ros2 run ur3_llm_control llm_task --serve --prompt-key
# Terminal khác, sau khi source và export cùng ROS_DOMAIN_ID/IGN_PARTITION:
ros2 topic pub --once /ur3_llm_control/command std_msgs/msg/String \
  "{data: 'Hãy gắp khối đỏ, đặt vào vùng B, rồi về home.'}"
ros2 topic echo /ur3_llm_control/status
```

`run_skills` vẫn yêu cầu `--evidence`; thay đổi tùy chọn chỉ áp dụng `llm_task`
và API `Skills(evidence_dir=None)`. Cùng một instance Skills lưu trạng thái giữ
vật qua các bước. Schema plan ví dụ:

```json
{"steps":[{"skill":"home"},{"skill":"pick","object":"red_cube"},{"skill":"place","object":"red_cube","zone":"zone_b"},{"skill":"home"}]}
```

Validator từ chối field thừa, key trùng, skill/object/zone không hợp lệ,
`place` trước `pick`, `home` khi đang giữ hoặc kết thúc còn giữ vật. MSSV là
ngữ cảnh cho LLM; các skill không áp đặt mapping màu–zone cố định.

Prompt: `src/ur3_llm_control/ur3_llm_control/planner_prompt.txt`; client/validator: `planning.py`,
parser: `router_response.py`; executor: `task_executor.py`; motion/state:
`skills.py`, `state.py`. Launch dùng cấu hình riêng rồi tham chiếu cấu hình UR gốc:
`bai2_sim.launch.py` khởi động Gazebo/control, gripper, MoveIt, scene initializer
và bridge `/bai2/gazebo_poses`; `bai2_moveit.launch.py` bổ sung SRDF và mapping
controller. Group `ur_grasp` dùng KDL tới `grasp_tcp`, `gripper` có states
`open/closed`; `ur_manipulator` gốc giữ tip `tool0`.

Bàn có mặt trên z=0,08 m; cube cạnh 30 mm có mass/collision; zone là visual.
`config/scene.json` mô tả ban đầu, Skills đọc pose Gazebo có freshness và cập nhật
planning scene sau mỗi lần thả. Nâng 50 mm và xác nhận giữ hơn 3,2 giây mô phỏng;
khi mang/hạ kiểm tra giữ vật liên tục, mất vật thì hủy action. Sau thả phải xác
nhận toàn bộ footprint cube nằm trong zone. Giữ nguyên PID, gravity và collision,
không teleport/weld; planning attachment không thay gắp vật lý.

Validator kiểm tra schema/allowlist/thứ tự/held object trước motion; executor
kiểm tra zone trống và trạng thái thật, dừng ngay ở lỗi, CLI !=0, không retry mù.
Chưa hỗ trợ buffer/đặt chồng/zone bị chiếm, nhiều client hoặc phục hồi tự động.
Segfault MoveIt dependency khi shutdown từng xuất hiện, chưa sửa dependency.

Bộ rút gọn được kiểm tra import, file cài đặt, tài nguyên, cấu hình và URDF/SRDF;
**chưa build hoặc chạy lại robot với bộ rút gọn**. Riêng sửa evidence tùy chọn đã
qua 25 kiểm tra offline tập trung ở workspace gốc (success/error, serve,
file/ảnh và guard chuyển động); không coi đây là một lượt chạy robot mới. Bằng chứng/báo cáo gốc được giữ
ngoài bộ nộp. Không coi kiểm tra tĩnh là một demo robot PASS mới.
