"""Sequential, injectable executor. It never generates or retries movement."""
from dataclasses import asdict
import math

from .planning import validate_plan


class ExecutionError(RuntimeError):
    pass


def verify_placements(skills, placed):
    skills.spin(0.2)  # all blocking operations must refresh ROS observations
    observed = {}
    for name, zone in placed.items():
        pose = skills.provider.get(name).pose
        p = [pose.position.x, pose.position.y, pose.position.z]
        target = skills.config['zones'][zone]['placement_cube_center']
        width = skills.config['zones'][zone]['size_xy']
        size = skills.config['cubes'][name]['size']
        if not all(math.isfinite(x) for x in p) or any(abs(p[i]-target[i]) > (width[i]-size[i])/2 for i in (0, 1)) or abs(p[2]-target[2]) > 0.003:
            raise ExecutionError(f'{name} is not fully settled in {zone}: {p}')
        q = pose.orientation
        observed[name] = {'zone': zone, 'position': p, 'quaternion': [q.x,q.y,q.z,q.w]}
    return observed


def execute_plan(plan, skills, emit=lambda *args: None):
    validate_plan(plan, skills.held_object)
    if skills.contact_object or skills.held_object or skills.holding_verified:
        raise ExecutionError('Existing grasp/unresolved attachment; recover before starting a new task')
    skills.spin(0.2)
    # Preflight ALL targets before even home/pick. Conservative: reject any
    # occupied target, even when another step would supposedly clear it.
    for step in plan['steps']:
        if step['skill'] == 'place':
            skills.verify_zone_empty(step['object'], step['zone'])
            # Also refuse a target occupied by the requested object itself.
            # There is no explicit no-op/buffer skill in this task executor.
            target = skills.config['zones'][step['zone']]
            actual = skills.provider.get(step['object']).pose.position
            size = skills.config['cubes'][step['object']]['size']
            if (abs(actual.x-target['placement_cube_center'][0]) < (target['size_xy'][0]+size[0])/2 and
                abs(actual.y-target['placement_cube_center'][1]) < (target['size_xy'][1]+size[1])/2):
                raise ExecutionError(f"Target {step['zone']} already occupied by {step['object']}; no buffer support")
    placed, results = {}, []
    for index, step in enumerate(plan['steps'], 1):
        skills.spin(0.2)
        skill = step['skill']
        if skill in ('home', 'pick') and (skills.contact_object or skills.held_object):
            raise ExecutionError(f'Step {index}: hand occupied or unresolved grasp')
        if skill == 'place':
            if skills.held_object != step['object'] or not skills.holding_verified:
                raise ExecutionError(f'Step {index}: physical holding precondition failed')
            skills.verify_zone_empty(step['object'], step['zone'])
            skills.check_carried(step['object'])
        emit('EXECUTION', {'step': index, **step, 'status': 'START'})
        args = [] if skill == 'home' else [step['object']]
        if skill == 'place':
            args.append(step['zone'])
        result = getattr(skills, skill)(*args)  # allowlist already validated
        data = asdict(result)
        emit('EXECUTION', {'step': index, **step, **data})
        results.append(data)
        if not result.success:
            raise ExecutionError(f'Step {index} {skill} failed: {result.reason}')
        if skill == 'pick' and (skills.held_object != step['object'] or not skills.holding_verified):
            raise ExecutionError(f'Step {index}: pick reported success without verified physical holding')
        if skill == 'place':
            if skills.contact_object or skills.held_object or skills.holding_verified:
                raise ExecutionError(f'Step {index}: place left an unresolved grasp')
            placed[step['object']] = step['zone']
        observed = verify_placements(skills, placed)
        if placed:
            emit('POSITION CHECK', observed)
    skills.sync_scene()  # compare actual poses to final MoveIt scene
    observed = verify_placements(skills, placed)
    if skills.held_object or skills.contact_object:
        raise ExecutionError('Task ended with unresolved grasp')
    return {'success': True, 'results': results, 'final_objects': observed,
            'held_object': skills.held_object, 'scene_verified': True}
