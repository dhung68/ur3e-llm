"""Restart an owned simulation for each trial; never teleport objects to reset."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def stop_owned(process):
    # Popen starts a new session. Its group contains only this trial's launch tree,
    # including Ruby Gazebo children that can outlive ROS launch's shell wrapper.
    for sig, delay in [(signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 2)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.2)
    process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description="Gazebo physical pick/place validation")
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--domain", type=int, default=146)
    parser.add_argument("--object", default="red_cube")
    parser.add_argument("--zone", default="zone_a", choices=["zone_a", "zone_b", "zone_c"])
    parser.add_argument("--mode", default="cycle", choices=["cycle", "sequence", "moveit_gripper", "gripper", "inspect", "home"])
    parser.add_argument("--sequence", nargs="+", metavar="OBJECT:ZONE")
    args = parser.parse_args()
    root = Path(args.evidence).resolve()
    root.mkdir(parents=True, exist_ok=False)
    outcomes = []
    for index in range(1, args.cycles + 1):
        trial = root / f"cycle{index:02d}"
        trial.mkdir()
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(args.domain + index - 1)
        env["ROS_LOCALHOST_ONLY"] = "1"
        env["IGN_PARTITION"] = f"ur3e_{root.name}_{index}"
        env["ROS_LOG_DIR"] = str(trial / "ros_log")
        launch_log = (trial / "launch.log").open("w")
        process = subprocess.Popen(
            ["ros2", "launch", "hri_bai2_environment", "bai2_sim.launch.py",
             f"gazebo_gui:={'true' if args.gui else 'false'}",
             f"moveit_launch_rviz:={'true' if args.gui else 'false'}"],
            env=env, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        (trial / "session.json").write_text(json.dumps({
            "launch_pid": process.pid, "process_group": process.pid,
            "ROS_DOMAIN_ID": env["ROS_DOMAIN_ID"], "IGN_PARTITION": env["IGN_PARTITION"],
            "initial_state": "Fresh world from bai2.sdf; no pose reset/teleport",
            "object": args.object, "zone": args.zone, "mode": args.mode,
            "sequence": args.sequence,
            "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in Path(__file__).parent.glob("*.py")},
        }, indent=2))
        print(f"Cycle {index}: launch PID {process.pid}, domain {env['ROS_DOMAIN_ID']}", flush=True)
        client = None
        try:
            # Let GUI initialize. Skill itself waits for action/pose/TF readiness.
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline and process.poll() is None:
                time.sleep(0.2)
            if args.gui:
                req = 'pose: {position: {x: 0.55 y: -0.50 z: 0.45} orientation: {x: -0.1870 y: 0.1543 z: 0.7487 w: 0.6169}}'
                camera = subprocess.run([
                    "ign", "service", "-s", "/gui/move_to/pose", "--reqtype", "ignition.msgs.GUICamera",
                    "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", req,
                ], env=env, capture_output=True, text=True, timeout=5)
                (trial / "camera.log").write_text(camera.stdout + camera.stderr)
            command = ["ros2", "run", "ur3_llm_control", "run_skills",
                       "--evidence", str(trial / "skills"), "--mode", args.mode,
                       "--object", args.object, "--zone", args.zone]
            if args.gui:
                command.append("--screenshots")
            if args.sequence:
                command += ["--sequence", *args.sequence]
            with (trial / "skills.log").open("w") as log:
                client = subprocess.Popen(command, env=env, stdout=log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    code = client.wait(timeout=900 if args.mode == "sequence" else 300)
                except subprocess.TimeoutExpired:
                    stop_owned(client)
                    code = 124
            outcomes.append({"cycle": index, "object": args.object, "zone": args.zone,
                             "exit_code": code, "success": code == 0})
            (root / "summary.json").write_text(json.dumps(outcomes, indent=2))
            print(f"Cycle {index}: {'PASS' if code == 0 else 'FAIL'} ({code})", flush=True)
        finally:
            if client and client.poll() is None:
                stop_owned(client)
            stop_owned(process)
            launch_log.close()
    return 0 if all(r["success"] for r in outcomes) else 1
