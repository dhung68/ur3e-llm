"""Independent fresh-world trials for arbitrary configured object/zone pairs."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--evidence', required=True)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--domain', type=int, default=180)
    parser.add_argument('--pairs', nargs='+', metavar='OBJECT:ZONE')
    args = parser.parse_args()
    pairs = [tuple(p.split(':')) for p in args.pairs] if args.pairs else [
        ('red_cube', 'zone_b'), ('red_cube', 'zone_c'), ('red_cube', 'zone_a'),
        *itertools.product(['yellow_cube', 'blue_cube'], ['zone_a', 'zone_b', 'zone_c'])]
    if any(len(p) != 2 for p in pairs) or not 0 <= args.domain <= 232-len(pairs)+1:
        parser.error('Invalid pair or DDS domain range')
    root = Path(args.evidence).resolve()
    root.mkdir(parents=True, exist_ok=False)
    outcomes = []
    for index, (obj, zone) in enumerate(pairs):
        trial = root / f'{index+1:02d}_{obj}_{zone}'
        command = ['ros2', 'run', 'ur3_llm_control', 'validate_cycles',
                   '--evidence', str(trial), '--cycles', '1', '--domain', str(args.domain+index),
                   '--object', obj, '--zone', zone]
        if args.gui:
            command.append('--gui')
        with (root/f'{index+1:02d}_runner.log').open('w') as log:
            code = subprocess.call(command, stdout=log, stderr=subprocess.STDOUT)
        outcomes.append({'object': obj, 'zone': zone, 'exit_code': code,
                         'success': code == 0, 'evidence': str(trial)})
        (root/'summary.json').write_text(json.dumps(outcomes, indent=2))
        print(f'{obj} -> {zone}: {"PASS" if code == 0 else "FAIL"}', flush=True)
    return 0 if all(o['success'] for o in outcomes) else 1


if __name__ == '__main__':
    raise SystemExit(main())
