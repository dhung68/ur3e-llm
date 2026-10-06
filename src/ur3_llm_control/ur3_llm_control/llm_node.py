"""ROS command node and one-shot terminal entrypoint for validated LLM tasks."""
import argparse
from collections import deque
import getpass
import json
import os
from pathlib import Path
import sys

from .planning import RouterPlanner, parse_plan, profile, validate_plan, validate_ros_domain
from .task_executor import execute_plan


def capture_completion(node, evidence, emit, label):
    """Optional GUI evidence, bounded camera/capture timeout; no effect on success."""
    if evidence is None:
        return
    try:
        from .screenshots import capture_windows
        # Keep the existing wide view: completion needs all three zones visible.
        files = capture_windows(evidence, label)
        emit('SCREENSHOT', {'files': files})
    except Exception as exc:
        emit('SCREENSHOT UNAVAILABLE', {'reason': str(exc)})
    finally:
        node.spin(0.2)


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String
    from .skills import Skills
    parser = argparse.ArgumentParser(description='Natural-language UR3e tasks via 9Router')
    parser.add_argument('--command')
    parser.add_argument('--plan-file', help='Validate/execute saved raw JSON steps; no LLM call')
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--serve', action='store_true', help='Read std_msgs/String on ~/command; fail-stop after any task error')
    parser.add_argument('--evidence', help='Optional new output directory; omitted means terminal only')
    parser.add_argument('--base-url', default='http://localhost:20128/v1')
    parser.add_argument('--model', default='gemini/gemini-3.5-flash-lite')
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--student-id', default='23020744')
    parser.add_argument('--student-name', default='')
    parser.add_argument('--screenshots', action='store_true')
    parser.add_argument('--expected-placements', nargs='+', metavar='OBJECT:ZONE',
                        help='Optional explicit task acceptance pairs, checked before motion')
    parser.add_argument('--require-final-home', action='store_true')
    parser.add_argument('--prompt-key', action='store_true', help='Read router key secretly from a real terminal if missing')
    args = parser.parse_args(remove_ros_args(args=sys.argv if argv is None else ['llm_task', *argv])[1:])
    if sum(bool(x) for x in (args.command, args.plan_file, args.serve)) != 1:
        parser.error('Choose exactly one of --command, --plan-file, --serve')
    evidence = Path(args.evidence).resolve() if args.evidence is not None else None
    log = None
    if evidence is not None:
        evidence.mkdir(parents=True, exist_ok=False)
        log = (evidence / 'task.log').open('w', encoding='utf-8')
    node = None
    report = {'success': False, 'mode': 'plan-only' if args.plan_only else 'execute',
              'source': 'saved-plan executor (no LLM request)' if args.plan_file else '9Router LLM'}
    queue = deque()
    busy = False
    task_index = 0
    publisher = None

    def emit(label, data):
        line = label + ': ' + (data if isinstance(data, str) else json.dumps(data, ensure_ascii=False))
        print(line, flush=True)
        if log is not None:
            log.write(line + '\n'); log.flush()
        if publisher is not None:
            publisher.publish(String(data=json.dumps({'event': label, 'data': data}, ensure_ascii=False)))

    def write_json(name, value):
        if evidence is not None:
            (evidence / name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')

    def incoming(msg):
        if busy or queue:
            publisher.publish(String(data=json.dumps({'event': 'TASK FAILED', 'data': 'Node busy; command rejected, not queued'})))
        else:
            queue.append(msg.data)

    def task(command=None, saved=None):
        nonlocal node, publisher, busy, report, task_index
        busy = True
        task_index += 1
        suffix = f'-{task_index:03d}' if args.serve else ''
        report = {'success': False, 'mode': 'plan-only' if args.plan_only else 'execute',
                  'source': 'saved-plan executor (no LLM request)' if saved is not None else '9Router LLM',
                  'command': command, 'student': student}
        emit('USER COMMAND', command or '[saved plan: executor only]')
        if saved is None:
            try:
                plan = planner.generate(command, student)
            finally:
                diagnostic = {'diagnostic': planner.last_response_diagnostic,
                              'normalized_response': planner.last_response_fixture,
                              'request_options': {'stream': False, 'accept': 'application/json',
                                                  'accept_encoding': 'identity'}}
                write_json(f'router_response{suffix}.json', diagnostic)
        else:
            plan = parse_plan(Path(saved).read_text(encoding='utf-8'))
        emit('LLM PLAN' if saved is None else 'SAVED PLAN', plan)
        validate_plan(plan)
        if args.expected_placements:
            expected = dict(pair.split(':') for pair in args.expected_placements)
            actual = {s['object']: s['zone'] for s in plan['steps'] if s['skill'] == 'place'}
            if expected != actual:
                raise ValueError(f'Plan does not match explicit task acceptance pairs: {expected}')
        if args.require_final_home and plan['steps'][-1] != {'skill': 'home'}:
            raise ValueError('Task acceptance requires final home')
        report['plan'] = plan
        emit('VALIDATION', 'PASS (complete JSON schema and holding sequence)')
        write_json(f'plan{suffix}.json', plan)
        if args.plan_only:
            report.update(success=True, motion_executed=False)
            emit('TASK SUCCESS', 'Plan validated; no motion executed')
        else:
            if not isinstance(node, Skills):
                publisher = None
                node.destroy_node()
                node = None
                node = Skills(evidence, compact_evidence=True, node_name='ur3_llm_control')
                publisher = node.create_publisher(String, '~/status', 10)
                if args.serve:
                    node.create_subscription(String, '~/command', incoming, 10)
            report.update(execute_plan(plan, node, emit))
            report['motion_executed'] = True
            # Verified final home values are checked by Skills.move_joints.
            report['checks'] = [e for e in node.events if e['stage'] in
                                ('action', 'holding_pass', 'scene_consistency', 'joint_target_check')]
            emit('FINAL POSITION CHECK', report['final_objects'])
            emit('TASK SUCCESS', {'skills': len(report['results']), 'scene_verified': True})
            if args.screenshots and evidence is not None:
                capture_completion(node, evidence, emit, 'completed'+suffix)
        write_json(f'task{suffix}.json', report)
        busy = False

    try:
        validate_ros_domain(os.environ.get('ROS_DOMAIN_ID', '0'))
        if args.prompt_key and not os.environ.get('ROUTER_API_KEY'):
            if not sys.stdin.isatty():
                raise RuntimeError('--prompt-key requires a real terminal; use ROUTER_API_KEY otherwise')
            os.environ['ROUTER_API_KEY'] = getpass.getpass('9Router key (hidden): ')
        init_args = list(sys.argv if argv is None else ['llm_task', *argv])
        if evidence is None:
            # Disable ROS's external file logger as well as application evidence.
            init_args += ['--ros-args', '--disable-external-lib-logs']
        rclpy.init(args=init_args)
        node = Node('ur3_llm_control')
        settings = {}
        for name, value in [('base_url', args.base_url), ('model', args.model), ('timeout', args.timeout),
                            ('student_id', args.student_id), ('student_name', args.student_name)]:
            settings[name] = node.declare_parameter(name, value).value
        student = profile(settings['student_id'], settings['student_name'])
        planner = RouterPlanner(settings['base_url'], settings['model'], settings['timeout'])
        report['student'] = student
        if not student['student_name']:
            emit('SUBMISSION NOTE', 'student_name is empty; add your full name before submission')
        publisher = node.create_publisher(String, '~/status', 10)
        if args.serve:
            node.create_subscription(String, '~/command', incoming, 10)
            emit('READY', 'Send commands on /ur3_llm_control/command; one client only')
            while rclpy.ok():
                if queue:
                    task(command=queue.popleft())
                else:
                    rclpy.spin_once(node, timeout_sec=0.1)
        else:
            task(args.command, args.plan_file)
        return 0
    except Exception as exc:
        report.update(success=False, error=str(exc))
        emit('VALIDATION' if 'plan' not in report else 'EXECUTION', 'FAILED')
        emit('TASK FAILED', str(exc))
        if node is not None and hasattr(node, 'held_object'):
            report.update(held_object=node.held_object, contact_object=node.contact_object,
                          holding_verified=node.holding_verified, completed_results=node.results,
                          checks=[e for e in node.events if e['stage'] in
                                  ('action', 'holding_pass', 'scene_consistency', 'joint_target_check')])
            if args.screenshots and evidence is not None:
                capture_completion(node, evidence, emit, f'error-{task_index:03d}')
        suffix = f'-{task_index:03d}' if args.serve else ''
        write_json(f'task{suffix}.json', report)
        return 1
    except KeyboardInterrupt:
        # No autonomous recovery movement. Use only one client and interrupt
        # between tasks; SIGINT during an action is handled by Skills cancellation.
        emit('TASK FAILED', 'Interrupted; do not restart motion if a grasp is unresolved')
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log is not None:
            log.close()
