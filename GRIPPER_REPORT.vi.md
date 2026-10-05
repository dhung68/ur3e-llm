# Báo cáo sửa và kiểm thử gripper UR3e

Đã sửa và kiểm thử trực tiếp bằng terminal trong Gazebo thật. Cấu hình cuối đạt sáu chu kỳ mở–đóng bằng action ở hai hướng cổ tay đối diện; MoveIt Plan và Execute tay UR3e thành công. Chưa chứng minh gripper giữ hoặc mang được cube.

## Môi trường và tình trạng ban đầu

- Ubuntu 22.04.5, ROS 2 Humble; Gazebo Fortress 6.18.0; ignition-physics 5.4.0, DART 6.12.1.
- `ign_ros2_control` và `gz_ros2_control`: 0.7.21. Tên `ign_ros2_control/IgnitionSystem` và `libign_ros2_control-system.so` hiện là đường tương thích với gz_ros2_control; nạp được trong log. Giữ nguyên tên plugin hiện có.
- JointTrajectoryController: 2.54.0; controller_manager: 2.54.2.
- Không tìm thấy AGENTS.md áp dụng. Đã thử `git status`: workspace gốc không có metadata Git khả dụng (`.git` nhìn thấy nhưng rỗng). Repo dependency `src/ur_simulation_gz` có Git và status sạch trước/sau. Không commit, push hoặc đổi nhánh.
- Khi bắt đầu, không có Gazebo/MoveIt/robot_state_publisher hoạt động và không có `/joint_states`. Chỉ thấy daemon ROS CLI có sẵn. Vì vậy đã tạo phiên kiểm thử mới; không có server cũ để kiểm tra trạng thái goal từng bị Ctrl+C. Các action trong lần kiểm thử này đều có kết quả cuối.
- File `simple_gripper.xacro` ban đầu không có `<gravity>false</gravity>`. Không tắt trọng lực hoặc collision trong quá trình chẩn đoán.
- Bản sao các file trước sửa ở `artifacts/gripper_debug/before/`. Thư mục artifacts có `COLCON_IGNORE` để bản sao package.xml không bị colcon nhận thành package trùng.

## Nguyên nhân và bằng chứng

**1. Đường điều khiển position của Gazebo bị kẹt khi khớp trượt chịu trọng lực và đã về chặn dưới.**

Controller active, interface claimed và action server tồn tại chỉ xác nhận ROS đã kết nối. Chúng không xác nhận ngón chuyển động. Mã nguồn đúng phiên bản 0.7.21 cho thấy plugin biến sai số vị trí thành `JointVelocityCmd`, thay vì trực tiếp điều khiển lực: `v = gain × update_rate × (q_command − q_actual)`. Xem [mã nguồn gz_system.cpp 0.7.21](https://github.com/ros-controls/gz_ros2_control/blob/0.7.21/gz_ros2_control/src/gz_system.cpp).

Tái hiện trên cấu hình ban đầu:

| Phép thử | Ngón trái, m | Ngón phải, m | Kết quả |
|---|---:|---:|---|
| Mở tới 0.015 khi robot vừa khởi tạo | 0.015000 | 0.015000 | SUCCESS, code 0 |
| Cổ tay ≈ +π/2; đóng rồi mở | 0.004866426 | ≈ 0 | ABORTED, code −4 |
| Cổ tay ≈ −π/2; đóng rồi mở | ≈ 0 | 0.004866426 | ABORTED, code −4 |

Khi đổi chiều trọng lực dọc theo ray trượt, bên bị kẹt đổi từ phải sang trái. Pose lấy qua Gazebo xác nhận hướng ray trượt tương ứng, không có dấu axis bị khai báo ngược. Một lần đưa cổ tay về ngang sau khi bị kẹt cũng chưa giải phóng được ngón đang ở chặn dưới. Các phép thử khoanh vùng lỗi ở cơ chế position → velocity servo của tổ hợp Fortress/DART với khớp trượt có giới hạn; chưa xác định một dòng lỗi nội bộ cụ thể của solver. Có [báo cáo upstream với triệu chứng prismatic không đi ngược trọng lực](https://github.com/ros-controls/gz_ros2_control/issues/192), nhưng kết luận ở đây dựa trên số đo tại workspace này.

Log ghi sai số khớp phải `0.005008 m > 0.005 m`, nên controller abort đúng chức năng. Sau abort, controller giữ vị trí lúc dừng. Khi đó reference/output trở thành vị trí dừng, error gần 0 là sai số của lệnh giữ, không phải sai số so với đích mở 0.015 m. Hành vi giữ vị trí sau lỗi tolerance được mô tả trong [tài liệu JTC Humble](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).

**2. Mặt collision của chân ngón trùng mặt đế, làm MoveIt báo tự va chạm.**

Đế có mặt trên z=0.020 m; chân ngón cũ cũng bắt đầu tại z=0.020 m. `/check_state_validity` trả `valid=False`, contact `left_finger ↔ gripper_base` và `gripper_base ↔ right_finger`, độ xuyên chỉ khoảng 4–5×10⁻¹⁷ m. Plan+Execute lúc đó thất bại với 0 điểm quỹ đạo. Đây là tiếp xúc giữa các mặt đồng phẳng trong mô hình collision, không phải chứng cứ ngón đâm sâu vào đế.

Sau khi nâng vị trí gắn ngón thêm 1 mm, MoveIt trả `valid=True`, `contacts=[]`; Plan+Execute thành công. Không thêm bất kỳ cặp bỏ qua va chạm nào để né lỗi.

## File sửa và lý do

| File | Thay đổi |
|---|---|
| `src/hri_ur3e_description/urdf/ur3e_gripper.urdf.xacro` | Đổi command interface của riêng hai ngón từ position sang effort, phạm vi lực ±20 N; bổ sung effort state. Giữ IgnitionSystem, vị trí khởi tạo 0.015 m và toàn bộ tay UR. |
| `src/hri_ur3e_description/config/ur_gripper_controllers.yaml` | Gripper JTC xuất effort, PID mỗi ngón `p=800`, `i=100`, `d=10`, `i_clamp=1`, `ff_velocity_scale=0`. Vẫn nhận quỹ đạo vị trí qua action cũ. Giữ path tolerance 0.005 m, goal tolerance 0.002 m và goal_time 2 s. |
| `src/hri_ur3e_description/urdf/simple_gripper.xacro` | Đổi z của hai joint origin từ 0.020 sang 0.021 m để tạo khe hở 1 mm. Giữ kích thước, khối lượng, inertia, axis, collision, giới hạn `[0, 0.02] m`, effort 20 N và velocity 0.02 m/s. |
| `src/hri_bai2_environment/launch/bai2_sim.launch.py` | Thêm spawner để tự nạp/activate gripper_controller; thêm tham số launch_rviz mặc định true, phục vụ kiểm thử headless. |
| `src/hri_bai2_environment/scripts/test_gripper.py` | Script kiểm thử thật qua action: chuẩn bị đóng, ba lần mở–đóng; kiểm tra SUCCESS, error_code, vị trí ngay lúc có kết quả và sau 0.5 s, giới hạn khớp, một publisher joint_states; có deadline và yêu cầu hủy goal nếu chờ kết quả quá hạn. Ghi JSON cả reference, feedback, error, velocity và force command. |
| `src/hri_bai2_environment/CMakeLists.txt`, `package.xml` | Cài script bằng ros2 run và khai báo các dependency sử dụng. |

PID biến sai số vị trí/vận tốc thành lực thực trong mô phỏng. Mỗi ngón nặng 0.04 kg nên lực cần để chống trọng lực tối đa khoảng `0.04 × 9.81 = 0.3924 N`. Thành phần tích phân duy trì lực giữ khi vị trí đã gần đích; giới hạn phần tích phân là 1 N. Các phép thử ghi lực lệnh lớn nhất khoảng 0.4183 N, thấp hơn nhiều so với giới hạn 20 N. Đây không phải fake hardware hoặc dịch chuyển trực tiếp pose link/cube.

Không sửa `/opt/ros`, repo dependency, world, scene.json hay initialize_scene.py. Không triển khai LLM, camera, 9Router hoặc pick/place.

## Kiểm thử cấu hình cuối

Build hai package thành công, kiểm tra cú pháp Python thành công, xacro xuất URDF và SDF thành công; `check_urdf` thành công. Phiên tích hợp nạp robot `ur` đúng một lần, có gripper_controller, joint_trajectory_controller và joint_state_broadcaster active.

Hai `finger_joint/effort` available/claimed. `/joint_states` có đúng một publisher là joint_state_broadcaster; `/clock` có đúng một publisher là ros_gz_bridge; action gripper có một server.

Mỗi quỹ đạo kéo dài 3 s, đích mở `[0.015, 0.015] m`, đóng `[0, 0] m`. Không gửi qua topic để né giám sát action, không ghi đè tolerance trong goal.

**Ba chu kỳ ở tư thế ngón phải nâng ngược trọng lực** (wrist_3 ≈ +π/2 modulo 2π). Số đo tại thời điểm action trả kết quả:

| Chu kỳ | Lệnh | Vị trí trái, m | Vị trí phải, m | Sai số trái, mm | Sai số phải, mm |
|---|---|---:|---:|---:|---:|
| 1 | Mở | 0.015003482 | 0.014303791 | 0.003482 | 0.696209 |
| 1 | Đóng | 0.000000291 | ≈ 0 | 0.000291 | ≈ 0 |
| 2 | Mở | 0.015002131 | 0.014676728 | 0.002131 | 0.323272 |
| 2 | Đóng | ≈ 0 | ≈ 0 | ≈ 0 | ≈ 0 |
| 3 | Mở | 0.015001550 | 0.014852824 | 0.001550 | 0.147176 |
| 3 | Đóng | ≈ 0 | ≈ 0 | ≈ 0 | ≈ 0 |

Tất cả 6 action trả SUCCEEDED, error_code 0. Sau 0.5 s, sai số lớn nhất 0.653802 mm. Sai số bám quỹ đạo lớn nhất 0.965638 mm, vẫn dưới path tolerance 5 mm.

**Ba chu kỳ ở hướng đối diện**, ngón trái nâng ngược trọng lực: tất cả 6 action SUCCESS/code 0; sai số lớn nhất ngay khi trả kết quả 0.000998 mm, sau ổn định 0.000935 mm. Thành phần tích phân đã có thời gian cân bằng tải trước chuỗi này, nên sai số nhỏ hơn chuỗi sau khi đổi hướng trọng lực.

Mỗi chuỗi có thêm một action chuẩn bị đóng, cũng SUCCESS. Tổng cộng 14 action trong hai chuỗi kiểm thử cuối đều thành công. Khoảng vị trí quan sát chung xấp xỉ `[-9.5×10⁻¹⁰, 0.015013] m`. Giá trị âm rất nhỏ ở chặn dưới là sai số số học của solver, không phải hành trình vượt giới hạn đáng kể. Vận tốc lớn nhất đo được 0.020000 m/s. Script dùng biên số học 1 µm để kiểm tra giới hạn vật lý, độc lập với goal tolerance 2 mm của controller.

**Đối chiếu mô hình đang chạy:** SDF xuất từ `/world/bai2/generate_world_sdf` có gravity `0 0 -9.81`, collision của hai ngón và giới hạn ban đầu. Link không khai báo tắt gravity; schema SDFormat cài tại `/usr/share/sdformat12/1.9/link.sdf` quy định gravity mặc định true. Khi mở, tọa độ X của link trong hệ cổ tay khoảng +0.033002 và −0.032945 m; khi đóng khoảng +0.018000 và −0.018000 m. Suy ra hai ngón thực sự đi ra hai phía. Vị trí suy ra từ pose Gazebo khớp joint_states trong 0.934 µm ở mẫu mở và 0.451 µm ở mẫu đóng.

**MoveIt và tay UR3e:** Đọc trực tiếp robot_description của move_group xác nhận gripper_base, left_finger, right_finger đều có collision. Planning scene có trạng thái hai khớp gripper khớp với ROS, cùng bàn, sàn và ba cube. State validity hợp lệ khi đóng và khi mở sau thử nghiệm. Plan+Execute qua `/move_action` thành công hai lần: 39 và 64 điểm quỹ đạo, status SUCCEEDED, MoveIt error_code 1 (SUCCESS). Sai số khớp tay lớn nhất lần lượt 0.000943 và 0.000951 rad. Lần thứ hai chạy sau chuỗi gripper đầu, xác nhận tay vẫn hoạt động. Wrist_3 là continuous joint nên sai số góc phải tính modulo 2π; −4.7116 rad tương đương khoảng +π/2.

Kiểm thử chạy Gazebo server và MoveIt headless để đo dữ liệu; chưa kiểm tra hình ảnh RViz/Gazebo GUI trong lần này. Launch mặc định vẫn mở cả hai GUI.

Đã dừng các phiên kiểm thử do tôi tạo bằng SIGINT gửi đúng PID launch (32388, 33556, 34440), để lần chạy lại không bị trùng dữ liệu. Phiên cuối kết thúc sạch với exit code 0. Không dừng daemon ROS CLI có sẵn hoặc dùng lệnh kill hàng loạt.

## Chạy lại từ terminal mới

Terminal 1: build và mở mô phỏng. Chỉ chạy một phiên; dừng launch cũ bằng Ctrl+C trong terminal của phiên đó trước.

```bash
cd /home/hung/workspaces/ur_gz_humble
source /opt/ros/humble/setup.bash
timeout 90s colcon build --packages-select hri_ur3e_description hri_bai2_environment --symlink-install
source install/setup.bash
ros2 launch hri_bai2_environment bai2_sim.launch.py
```

Launch này tự khởi động gripper_controller. Có thể kiểm thử headless bằng:

```bash
ros2 launch hri_bai2_environment bai2_sim.launch.py gazebo_gui:=false launch_rviz:=false
```

Terminal 2: source riêng terminal này, kiểm tra controller và chạy ba chu kỳ bằng action:

```bash
cd /home/hung/workspaces/ur_gz_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 10s ros2 control list_controllers
timeout 90s ros2 run hri_bai2_environment test_gripper.py --output /tmp/gripper_test.json
```

Script phải in `PASS`, gồm ba chu kỳ mở–đóng và một lần đóng chuẩn bị. Không chạy đồng thời với client khác đang điều khiển gripper.

Nếu dùng launch `ur_sim_control.launch.py` trực tiếp thay cho launch bài 2, cần chạy spawner thủ công (terminal đã source như trên):

```bash
timeout 75s ros2 run controller_manager spawner gripper_controller \
  -c /controller_manager --controller-manager-timeout 60 \
  --service-call-timeout 10 --switch-timeout 10
```

Muốn lặp lại phép thử MoveIt và hai hướng cổ tay tại tư thế khởi tạo của môi trường này:

```bash
timeout 90s python3 artifacts/gripper_debug/moveit_check.py /tmp/arm_minus.json -1.57079632679
timeout 90s ros2 run hri_bai2_environment test_gripper.py --output /tmp/gripper_minus.json
timeout 90s python3 artifacts/gripper_debug/moveit_check.py /tmp/arm_plus.json 1.57079632679
timeout 90s ros2 run hri_bai2_environment test_gripper.py --output /tmp/gripper_plus.json
```

Script MoveIt thay đổi shoulder_pan thêm 0.1 rad và đặt wrist_3 tới góc chỉ định bằng Plan+Execute có kiểm tra va chạm, phục vụ kiểm thử này. Các script thu pose, probe ban đầu và số đo JSON nằm trong `artifacts/gripper_debug/`. Hai file chính là `final_cycles_reverse.json`, `final_cycles_tilt.json`; bằng chứng tay là `final_moveit_check.json`, `final_arm_after_cycles.json`; mô hình thực là `final_world_live.sdf`; chênh lệch file có trong `changes.patch`.

## Giới hạn và cách trình bày

Kiểm thử này chứng minh điều khiển mở–đóng gripper trong Gazebo ở các tư thế đã thử, với trọng lực và collision bật. **Chưa chứng minh ngón kẹp giữ được cube hoặc robot mang được cube.** Muốn chứng minh cần thử tiếp xúc, ma sát, tải, lực kẹp và độ ổn định trong chuyển động tay; không thể suy ra từ open/close SUCCESS.

MoveIt đã nạp hình học và trạng thái gripper để kiểm tra va chạm của tay. SRDF và cấu hình controller MoveIt gốc vẫn chỉ phục vụ tay UR; chưa thêm planning group/end-effector riêng để điều khiển gripper từ giao diện MoveIt. Hiện gripper được điều khiển bằng FollowJointTrajectory action trực tiếp. Không thực hiện camera hoặc pick/place; cảnh báo thiếu 3D sensor cho Octomap trong log không ngăn các kiểm thử collision vật thể đã nạp.

Cách giải thích khi trình bày: “ROS đã gửi lệnh đúng, nhưng cơ chế servo vị trí của bộ mô phỏng không kéo được một ngón khỏi chặn dưới khi hướng trượt chịu trọng lực. Controller phát hiện sai số và dừng đúng quy định. Tôi đổi riêng truyền động gripper sang PID tạo lực thực, giữ cùng đích vị trí và cùng tolerance. Đồng thời tạo khe hở hình học 1 mm để bộ kiểm tra va chạm không nhận hai mặt chạm nhau là tự va chạm. Kết quả được xác nhận bằng action SUCCESS, số đo khớp và pose Gazebo, rồi kiểm thử lại Plan+Execute của tay.”
