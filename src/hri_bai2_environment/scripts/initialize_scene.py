#!/usr/bin/env python3

import json
from pathlib import Path

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def create_box(name, position, dimensions, frame):
    """Tạo thông tin một vật hình hộp cho MoveIt."""

    box = CollisionObject()

    # Tên vật và hệ tọa độ dùng để đặt vật.
    box.id = name
    box.header.frame_id = frame

    # Hình dạng và kích thước.
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = [float(value) for value in dimensions]

    # Vị trí tâm vật và hướng của vật.
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.w = 1.0

    box.primitives = [shape]
    box.primitive_poses = [pose]

    # Dùng hằng số ADD nên không cần viết !!binary như trong terminal.
    box.operation = CollisionObject.ADD

    return box


def main():
    rclpy.init()
    node = Node("bai2_scene_initializer")
    exit_code = 0

    try:
        # Tìm thư mục package đã được ROS cài sau khi build.
        package_directory = Path(
            get_package_share_directory("hri_bai2_environment")
        )

        # Đọc kích thước và tọa độ từ file cấu hình.
        config_path = package_directory / "config" / "scene.json"

        with config_path.open(encoding="utf-8") as file:
            config = json.load(file)

        # Tạo client để gửi yêu cầu tới dịch vụ MoveIt.
        client = node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        node.get_logger().info("Dang cho dich vu MoveIt...")

        # Chờ tối đa 60 giây; không gửi khi dịch vụ chưa sẵn sàng.
        if not client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError(
                "Khong tim thay /apply_planning_scene sau 60 giay."
            )

        request = ApplyPlanningScene.Request()

        # Bổ sung vào scene hiện có.
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True

        frame = config["frame"]
        table = config["table"]
        
        request.scene.world.collision_objects.append(
            create_box(
                name="ground_plane",
                position=[0.0, 0.0, -0.05],
                dimensions=[2.0, 2.0, 0.10],
                frame=frame,
            )
        )

        # Thêm bàn.
        request.scene.world.collision_objects.append(
            create_box(
                name="work_table",
                position=table["center"],
                dimensions=table["size"],
                frame=frame,
            )
        )

        # Thêm ba cube.
        for name, cube in config["cubes"].items():
            position = list(cube["initial_center"])

            # Tâm cube khi đã nằm ổn định trên mặt bàn.
            position[2] = (
                table["surface_z"] + cube["size"][2] / 2.0
            )

            request.scene.world.collision_objects.append(
                create_box(
                    name=name,
                    position=position,
                    dimensions=cube["size"],
                    frame=frame,
                )
            )

        # Màu hiển thị trong RViz.
        colors = {
            "work_table": (0.55, 0.40, 0.25),
            "red_cube": (0.90, 0.08, 0.08),
            "yellow_cube": (0.95, 0.80, 0.05),
            "blue_cube": (0.08, 0.15, 0.90),
        }

        for name, rgb in colors.items():
            color = ObjectColor()
            color.id = name
            color.color.r = rgb[0]
            color.color.g = rgb[1]
            color.color.b = rgb[2]
            color.color.a = 1.0

            request.scene.object_colors.append(color)

        # Gửi yêu cầu và chờ câu trả lời.
        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=30.0,
        )

        if not future.done():
            future.cancel()
            raise RuntimeError(
                "MoveIt khong tra loi sau 30 giay."
            )

        response = future.result()

        if response is None or not response.success:
            raise RuntimeError(
                "MoveIt khong chap nhan cap nhat scene."
            )

        node.get_logger().info(
            "SCENE READY: work_table, red_cube, yellow_cube, blue_cube"
        )

    except KeyboardInterrupt:
        pass

    except Exception as error:
        node.get_logger().error(str(error))
        exit_code = 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())