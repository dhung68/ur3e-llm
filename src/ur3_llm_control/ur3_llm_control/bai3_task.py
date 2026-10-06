"""ROS 2 camera -> 9Router -> occupancy validator -> physical MoveIt executor."""
import argparse
from dataclasses import asdict
import getpass
import json
import os
from pathlib import Path
import time
import rclpy
from std_msgs.msg import String
from .planning import RouterPlanner, profile, parse_plan, validate_ros_domain
from .bai3_skills import CameraSkills
from .bai3_state import validate_bai3, occupants
from .task_executor import verify_placements, ExecutionError


def execute(plan, skills, emit):
    positions = skills.provider.snapshot(skills.config['cubes'])
    validate_bai3(plan, skills.config, positions, skills.held_object)
    if skills.contact_object or skills.holding_verified:
        raise ExecutionError('Existing unresolved grasp; manual recovery required')
    placed, results = {}, []
    for i, step in enumerate(plan['steps'], 1):
        skills.spin(.2)
        skill = step['skill']
        if skill in ('home', 'pick'):
            if skills.contact_object or skills.held_object:
                raise ExecutionError('Hand occupied/unresolved grasp')
            if skill == 'pick':
                skills.provider.snapshot(skills.config['cubes'])
                placed.pop(step['object'], None)
            # home retains occluded collision geometry, then requires a full new observation.
        else:
            if skills.held_object != step['object'] or not skills.holding_verified:
                raise ExecutionError('Physical holding precondition failed')
            skills.prepare_place_visibility()
            skills.check_carried(step['object'])
            skills.verify_zone_empty(step['object'], step['zone'])
        emit('EXECUTION', {'step': i, **step, 'status': 'START'})
        args = [] if skill == 'home' else [step['object']]
        if skill == 'place': args.append(step['zone'])
        result = getattr(skills, skill)(*args)
        results.append(asdict(result))
        emit('EXECUTION', {'step': i, **step, **asdict(result)})
        if not result.success:
            raise ExecutionError(f'Step {i} {skill}: {result.reason}')
        if skill == 'pick' and (skills.held_object != step['object'] or not skills.holding_verified):
            raise ExecutionError('Pick result not physically verified')
        if skill == 'place':
            if skills.contact_object or skills.held_object:
                raise ExecutionError('Release left unresolved grasp')
            placed[step['object']] = step['zone']
        if skill == 'home':
            # At observation pose require all five measured again, then sync collision scene.
            skills.spin(.3)
            skills.provider.snapshot(skills.config['cubes'])
            skills.sync_scene()
            emit('POSITION CHECK', verify_placements(skills, placed))
    final = verify_placements(skills, placed)
    skills.sync_scene()
    return {'success': True, 'results': results, 'final_objects': final,
            'all_camera_objects': skills.provider.snapshot(skills.config['cubes']),
            'held_object': skills.held_object, 'scene_verified': True}


def run(args):
    validate_ros_domain(os.environ.get('ROS_DOMAIN_ID', '0'))
    directory = Path(args.evidence)
    directory.mkdir(parents=True, exist_ok=False)
    (directory/'COLCON_IGNORE').touch()
    report = {'command': args.command, 'plan_only': args.plan_only, 'success': False,
              'source': 'saved plan executor replay' if args.plan_file else '9Router live'}
    log = (directory/'task.log').open('w', encoding='utf-8')
    skills = None
    planner = None
    def emit(label, value):
        line = f'{label}: ' + (value if isinstance(value, str) else json.dumps(value, ensure_ascii=False))
        print(line, flush=True)
        log.write(line+'\n'); log.flush()
        if skills:
            status.publish(String(data=line))
    try:
        if args.prompt_key and not args.plan_file and not os.environ.get('ROUTER_API_KEY'):
            os.environ['ROUTER_API_KEY'] = getpass.getpass('9Router key (hidden): ')
        rclpy.init()
        skills = CameraSkills(directory)
        status = skills.create_publisher(String, '/bai3/task_status', 10)
        if args.listen:
            commands = []
            subscription = skills.create_subscription(String, '/bai3/command', lambda msg: commands.append(msg.data) if not commands else None, 1)
            emit('READY', 'Waiting for one command on /bai3/command')
            while rclpy.ok() and not commands: skills.spin(.1)
            if not commands: raise ExecutionError('ROS shutdown before command')
            args.command = commands[0]
            report['command'] = args.command
            skills.destroy_subscription(subscription)
        emit('USER COMMAND', args.command)
        positions, temps = skills.available_temporaries()
        context = {'student': profile(), 'objects': list(skills.config['cubes']),
                   'zones': ['zone_a','zone_b','zone_c'], 'held_object': None,
                   'camera_state': {'frame_id':'world','objects': positions,
                                    'zone_occupancy': {z: occupants(positions, skills.config, z) for z in ('zone_a','zone_b','zone_c')}},
                   'available_temporaries': temps}
        report['context'] = context
        if args.plan_file:
            plan = parse_plan(Path(args.plan_file).read_text())
        else:
            planner = RouterPlanner(args.base_url, args.model, args.timeout)
            plan = planner.generate_context(args.command, context, Path(__file__).with_name('bai3_prompt.txt'))
        report['plan'] = plan
        (directory/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2))
        emit('LLM PLAN' if not args.plan_file else 'SAVED PLAN (executor replay)', plan)
        # Network can block callbacks; use fresh camera state for preflight.
        skills.spin(.3)
        validate_bai3(plan, skills.config, skills.provider.snapshot(skills.config['cubes']), skills.held_object)
        emit('VALIDATION', 'PASS')
        if args.plan_only:
            report['success'] = True
            emit('TASK SUCCESS', 'plan-only; no motion goals sent')
        else:
            report.update(execute(plan, skills, emit))
            emit('TASK SUCCESS', {'skills':len(report['results']), 'scene_verified':True})
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'Interrupted'
        emit('TASK FAILED', report['error'])
    finally:
        if planner and planner.last_response_fixture:
            (directory/'router_response.json').write_text(json.dumps(planner.last_response_fixture, indent=2))
        if skills:
            report['held_object'] = skills.held_object
            report['pending_object'] = skills.contact_object
            report['holding_verified'] = skills.holding_verified
            report['motion_terminal_unconfirmed'] = bool(getattr(skills, 'motion_unresolved', False))
            report['recovery_required'] = bool(skills.contact_object) or report['motion_terminal_unconfirmed']
            report['skill_events'] = skills.events
            if args.screenshots and not args.plan_only:
                try:
                    from .bai3_screenshots import capture
                    report['screenshots'] = capture(directory, 'bai3_completed' if report['success'] else 'bai3_error')
                except Exception as exc:
                    report['screenshot_error'] = str(exc)
                skills.spin(.2)
            skills.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        (directory/'task.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        log.close()
    return 0 if report['success'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    command = parser.add_mutually_exclusive_group(required=True)
    command.add_argument('--command')
    command.add_argument('--listen', action='store_true')
    parser.add_argument('--evidence', required=True)
    parser.add_argument('--plan-file')
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--prompt-key', action='store_true')
    parser.add_argument('--screenshots', action='store_true')
    parser.add_argument('--base-url', default='http://localhost:20128/v1')
    parser.add_argument('--model', default='gemini/gemini-3.5-flash-lite')
    parser.add_argument('--timeout', type=float, default=45.)
    return run(parser.parse_args())
