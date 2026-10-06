"""Two owned fresh worlds, at most three LLM requests; no automatic retries."""
import argparse
import getpass
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from .planning import SECRET_HELP, RouterPlanner, profile, validate_ros_domain
from .validation import stop_owned

BASIC = 'Hãy gắp khối đỏ, đặt vào vùng B, rồi về home.'
ENGLISH = 'Pick the blue cube, place it in zone A, and return home.'
ADVANCED = 'Arrange all objects according to my student ID'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', required=True)
    parser.add_argument('--prompt-key', action='store_true')
    parser.add_argument('--base-url', default='http://localhost:20128/v1')
    parser.add_argument('--model', default='gemini/gemini-3.5-flash-lite')
    parser.add_argument('--timeout', type=float, default=45)
    parser.add_argument('--student-id', default='23020744')
    parser.add_argument('--student-name', default='')
    parser.add_argument('--domain', type=int, default=220)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--basic-plan-file', help='Reuse validated basic LLM plan; no basic LLM request')
    parser.add_argument('--skip-english-plan', action='store_true',
                        help='Do not make another plan-only request during a focused executor retest')
    parser.add_argument('--nested-display', help='Optional private Xephyr display, e.g. :88 (requires --gui)')
    args = parser.parse_args()
    root = Path(args.evidence).resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root/'COLCON_IGNORE').touch()
    summary = {'success': False, 'llm_calls': 0, 'attempts': [], 'advanced': 'not attempted'}
    display = None
    generated_log = Path.cwd()/'log'/root.name
    generated_log.mkdir(parents=True, exist_ok=False)
    child = None
    launch = None
    env = os.environ.copy()
    try:
        validate_ros_domain(args.domain)
        validate_ros_domain(args.domain+1)
        if not env.get('ROUTER_API_KEY') and args.prompt_key:
            if not sys.stdin.isatty():
                raise RuntimeError('--prompt-key requires a real terminal')
            env['ROUTER_API_KEY'] = getpass.getpass('9Router key (hidden): ')
        if not env.get('ROUTER_API_KEY'):
            raise RuntimeError(SECRET_HELP)
        if args.nested_display:
            if not args.gui:
                raise RuntimeError('--nested-display requires --gui')
            display = subprocess.Popen(['Xephyr', args.nested_display, '-ac', '-screen', '1280x960',
                                        '-nolisten', 'tcp', '-noreset'], stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL, start_new_session=True)
            env.update(DISPLAY=args.nested_display, LIBGL_ALWAYS_SOFTWARE='1')
            time.sleep(2)
            if display.poll() is not None:
                raise RuntimeError('Private Xephyr display failed to start; check display number')
            subprocess.run(['xset', 's', 'off'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        common = ['--base-url', args.base_url, '--model', args.model, '--timeout', str(args.timeout),
                  '--student-id', args.student_id, '--student-name', args.student_name]
        for index, (label, command) in enumerate([('basic', BASIC), ('advanced', ADVANCED)]):
            env.update(ROS_DOMAIN_ID=str(args.domain+index), ROS_LOCALHOST_ONLY='1',
                       IGN_PARTITION='ur3e_'+root.name+'_'+label, ROS_LOG_DIR=str(generated_log/label))
            with (generated_log/(label+'_launch.log')).open('w') as output:
                launch = subprocess.Popen(['ros2', 'launch', 'hri_bai2_environment', 'bai2_sim.launch.py',
                                           'gazebo_gui:='+str(args.gui).lower(),
                                           'moveit_launch_rviz:='+str(args.gui).lower()],
                                          env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                session = {'domain': env['ROS_DOMAIN_ID'], 'partition': env['IGN_PARTITION'],
                           'launch_pid': launch.pid, 'fresh_world': True}
                print('DEMO SESSION:', label, json.dumps(session), flush=True)
                time.sleep(6)
                if launch.poll() is not None:
                    raise RuntimeError('Simulation launch exited before client startup')
                if args.gui:
                    try:
                        subprocess.run(['ign', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'ignition.msgs.GUICamera',
                                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000', '--req',
                                        'pose: {position: {x: 0.85 y: -0.65 z: 0.65} orientation: {x: -0.1870 y: 0.1543 z: 0.7487 w: 0.6169}}'],
                                       env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
                    except subprocess.TimeoutExpired:
                        print('SCREENSHOT NOTE: camera setup timed out; existing view retained', flush=True)
                command_line = ['ros2', 'run', 'ur3_llm_control', 'llm_task', '--evidence', str(root/label),
                                '--require-final-home', *common]
                saved_basic = args.basic_plan_file if label == 'basic' else None
                command_line += ['--plan-file', saved_basic] if saved_basic else ['--command', command]
                expected = ({'red_cube': 'zone_b'} if label == 'basic' else
                            {obj: zone for zone, obj in profile(args.student_id, args.student_name)['zone_assignment'].items()})
                command_line += ['--expected-placements', *[obj+':'+zone for obj,zone in expected.items()]]
                if args.gui:
                    command_line.append('--screenshots')
                if not saved_basic:
                    summary['llm_calls'] += 1
                child = subprocess.Popen(command_line, env=env, start_new_session=True)
                # No outer movement timeout: each action has bounded cancellation
                # in Skills. Wait for the client to stop movement before teardown.
                code = child.wait()
                result_file = root/label/'task.json'
                result = json.loads(result_file.read_text()) if result_file.exists() else {'success': False, 'error': 'Client exited without task report'}
                result.update(session=session, exit_code=code, label=label)
                summary['attempts'].append(result)
                # Child has completed all action waits. Keep any unresolved grasp
                # explicit; do not perform autonomous recovery or another task.
                stop_owned(launch); launch = None; child = None
                if code != 0 or not result.get('success'):
                    raise RuntimeError(label+' demo failed; no automatic movement retry')
            if label == 'basic' and not args.skip_english_plan:
                # Request 2 is genuinely plan-only with a different object/zone.
                english = root/'english_plan_only'
                summary['llm_calls'] += 1
                code = subprocess.run(['ros2', 'run', 'ur3_llm_control', 'llm_task', '--evidence', str(english),
                                       '--plan-only', '--command', ENGLISH, *common], env=env).returncode
                if code != 0:
                    raise RuntimeError('English plan-only failed; advanced not attempted')
            else:
                summary['advanced'] = 'PASS: all placements, final scene and home checked'
        summary['success'] = True
        return 0
    except Exception as exc:
        summary['error'] = str(exc)
        print('TASK FAILED:', str(exc), flush=True)
        return 1
    except KeyboardInterrupt:
        # Do not SIGINT a client while it may carry a cube. Its current task runs
        # to terminal action results before we stop the owned world.
        print('Stopping after the current client finishes; no new task will start.', flush=True)
        if child is not None:
            child.wait()
        summary['error'] = 'Operator interrupted runner; no new task started'
        return 130
    finally:
        if launch is not None:
            stop_owned(launch)
        if display is not None:
            stop_owned(display)
        # Do not duplicate full client reports; link to compact per-demo evidence.
        summary['attempts'] = [{k: a.get(k) for k in ('label','success','exit_code','error','session','final_objects')}
                               for a in summary['attempts']]
        (root/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
