import argparse
from dataclasses import asdict
import json
from pathlib import Path

import rclpy

from .skills import Skills


def main():
    parser = argparse.ArgumentParser(description="Run physical skills without an LLM")
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--zone", default="zone_a", choices=["zone_a", "zone_b", "zone_c"])
    parser.add_argument("--object", default="red_cube")
    parser.add_argument("--mode", default="cycle", choices=["cycle", "sequence", "home", "inspect", "gripper", "moveit_gripper"])
    parser.add_argument("--sequence", nargs="+", metavar="OBJECT:ZONE")
    parser.add_argument("--screenshots", action="store_true")
    args = parser.parse_args()
    # Each invocation must have its own directory; existing evidence is never overwritten.
    path = Path(args.evidence)
    path.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = None
    success = False
    try:
        node = Skills(path, screenshots=args.screenshots)
        if args.mode == "inspect":
            node.sync_scene()
            node.snapshot("initial")
            success = True
        elif args.mode in ("gripper", "moveit_gripper"):
            from .skills import FINGERS
            if args.mode == "moveit_gripper":
                node.move_joints([0.0, 0.0], "moveit_gripper_closed", "gripper", FINGERS, 0.5)
            else:
                node.set_gripper(0.0)
            node.snapshot("gripper_closed_empty")
            if args.mode == "moveit_gripper":
                node.move_joints([0.015, 0.015], "moveit_gripper_open", "gripper", FINGERS, 0.5)
            else:
                node.set_gripper(0.015)
            node.snapshot("gripper_open_empty")
            success = True
        elif args.mode in ("cycle", "sequence"):
            pairs = [(args.object, args.zone)] if args.mode == "cycle" else [tuple(p.split(":")) for p in (args.sequence or [])]
            if not pairs or any(len(p) != 2 or p[0] not in node.config["cubes"] or p[1] not in node.config["zones"] for p in pairs):
                raise ValueError("Supply known OBJECT:ZONE pairs for --sequence")
            if args.mode == "sequence" and (len({p[0] for p in pairs}) != len(pairs) or len({p[1] for p in pairs}) != len(pairs)):
                raise ValueError("Sequence must use distinct objects and destination zones")
            node.spin(0.2)
            node.record("trial_initial", pairs=pairs, world_reset=False)
            for obj, zone in pairs:
                node.verify_zone_empty(obj, zone)
            placed = {}
            success = True
            for obj, zone in pairs:
                node.record("sequence_step", object=obj, zone=zone, previously_placed=placed)
                result = node.home()
                if result.success:
                    result = node.pick(obj)
                if result.success:
                    result = node.place(obj, zone)
                if not result.success:
                    success = False
                    break
                placed[obj] = zone
                # Recheck every earlier placement after each scene update and motion.
                node.spin(0.2)
                for prior, prior_zone in placed.items():
                    actual = node.provider.get(prior).pose.position
                    dest = node.config["zones"][prior_zone]["placement_cube_center"]
                    width = node.config["zones"][prior_zone]["size_xy"]
                    size = node.config["cubes"][prior]["size"]
                    if abs(actual.x-dest[0]) > (width[0]-size[0])/2 or abs(actual.y-dest[1]) > (width[1]-size[1])/2 or abs(actual.z-dest[2]) > .003:
                        raise RuntimeError(f"Previously placed {prior} moved outside {prior_zone}")
                node.record("sequence_placement_verified", placed=placed)
            node.record("trial_final", success=success, placed=placed)
            print(json.dumps(asdict(result)), flush=True)
        else:
            result = node.home()
            success = result.success
            print(json.dumps(asdict(result)), flush=True)
    except Exception as error:
        success = False
        (path / "fatal_error.json").write_text(json.dumps({"error": str(error)}, indent=2))
        print(f"FAIL: {error}", flush=True)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
