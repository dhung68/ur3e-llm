"""Pure JSON validation and one-shot OpenAI-compatible 9Router planning."""
import itertools
import json
import os
from pathlib import Path
import socket
from urllib import error, request
from urllib.parse import urlsplit

from .router_response import decode_response, ResponseError

OBJECTS = ('red_cube', 'yellow_cube', 'blue_cube')
ZONES = ('zone_a', 'zone_b', 'zone_c')
SECRET_HELP = ('ROUTER_API_KEY is missing. In the running terminal: '
               "read -rsp '9Router key: ' ROUTER_API_KEY; echo; export ROUTER_API_KEY. "
               'Do not paste the key into chat or save it in files.')


class PlanError(ValueError):
    pass


class RouterError(RuntimeError):
    pass


def validate_ros_domain(value):
    """Default DDS ports cannot represent domain IDs beyond 232."""
    try:
        domain = int(value)
    except (ValueError, TypeError):
        raise PlanError('ROS_DOMAIN_ID must be an integer in 0..232') from None
    if not 0 <= domain <= 232:
        raise PlanError('ROS_DOMAIN_ID must be in 0..232; higher values exceed DDS port limits')
    return domain


def profile(student_id='23020744', student_name=''):
    if not isinstance(student_id, str) or not student_id.isdecimal() or len(student_id) < 2:
        raise PlanError('student_id must be a string of at least two decimal digits')
    p = int(student_id[-2:]) % 6
    colors = list(itertools.permutations(OBJECTS))[p]
    return {'student_id': student_id, 'student_name': student_name, 'P': p,
            'zone_assignment': dict(zip(ZONES, colors))}


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PlanError(f'Duplicate JSON key: {key}')
        result[key] = value
    return result


def parse_plan(text):
    if not isinstance(text, str) or len(text.encode('utf-8')) > 65536:
        raise PlanError('JSON plan must be text <=64 KiB')
    try:
        return json.loads(text, object_pairs_hook=_unique_object,
                          parse_constant=lambda _: (_ for _ in ()).throw(PlanError('Non-finite JSON number')))
    except (ValueError, RecursionError) as exc:
        raise PlanError('Invalid JSON plan') from exc


def validate_plan(plan, held_object=None):
    """Check the complete plan before constructing a robot client or moving."""
    if type(plan) is not dict or set(plan) != {'steps'}:
        raise PlanError('Root must contain only steps')
    if type(plan['steps']) is not list or not 1 <= len(plan['steps']) <= 24:
        raise PlanError('steps must be a list with 1..24 entries')
    held = held_object
    destinations = {}  # conservative: no buffer/rearrangement support
    picked = set()
    for i, step in enumerate(plan['steps'], 1):
        if type(step) is not dict or type(step.get('skill')) is not str:
            raise PlanError(f'Step {i}: expected skill object')
        skill = step['skill']
        keys = {'home': {'skill'}, 'pick': {'skill', 'object'},
                'place': {'skill', 'object', 'zone'}}
        if skill not in keys or set(step) != keys[skill]:
            raise PlanError(f'Step {i}: skill/parameters not allowed')
        if skill != 'home' and (type(step['object']) is not str or step['object'] not in OBJECTS):
            raise PlanError(f'Step {i}: unknown object')
        if skill == 'home':
            if held is not None:
                raise PlanError(f'Step {i}: home while holding is forbidden')
        elif skill == 'pick':
            if held is not None or step['object'] in picked:
                raise PlanError(f'Step {i}: duplicate pick or hand already occupied; no buffer support')
            held = step['object']
            picked.add(held)
        else:
            zone = step['zone']
            if type(zone) is not str or zone not in ZONES:
                raise PlanError(f'Step {i}: unknown zone')
            if held != step['object']:
                raise PlanError(f'Step {i}: place requires matching pick')
            if zone in destinations:
                raise PlanError(f'Step {i}: destination reused; stacking/buffer moves unsupported')
            destinations[zone] = held
            held = None
    if held is not None:
        raise PlanError('Plan ends holding an object; include matching place')
    return plan


class RouterPlanner:
    def __init__(self, base_url='http://localhost:20128/v1',
                 model='gemini/gemini-3.5-flash-lite', timeout=45.0, opener=None):
        parts = urlsplit(base_url)
        if parts.scheme not in ('http', 'https') or not parts.hostname or parts.username or parts.password or parts.query or parts.fragment:
            raise RouterError('Invalid base URL (no credentials/query allowed)')
        if not 0 < timeout <= 300 or not model:
            raise RouterError('Invalid model/timeout')
        self.base_url, self.model, self.timeout = base_url.rstrip('/'), model, timeout
        self.opener = opener or request.urlopen
        self.last_response_diagnostic = {}
        self.last_response_fixture = None

    def generate(self, command, student, scene=None):
        self.last_response_diagnostic = {}
        self.last_response_fixture = None
        if not isinstance(command, str) or not command.strip() or len(command) > 4000:
            raise PlanError('User command must contain 1..4000 characters')
        key = os.environ.get('ROUTER_API_KEY')
        if not key:
            raise RouterError(SECRET_HELP)
        prompt = Path(__file__).with_name('planner_prompt.txt').read_text(encoding='utf-8')
        context = {'student': student, 'objects': OBJECTS, 'zones': ZONES,
                   'held_object': None, 'scene': scene or 'Execution will check live Gazebo state before motion.'}
        payload = {'model': self.model, 'temperature': 0, 'stream': False,
                   'messages': [{'role': 'system', 'content': prompt},
                                {'role': 'user', 'content': json.dumps({'context': context, 'command': command}, ensure_ascii=False)}]}
        req = request.Request(self.base_url + '/chat/completions',
                              data=json.dumps(payload).encode(),
                              headers={'Authorization': 'Bearer ' + key, 'Content-Type': 'application/json', 'Accept': 'application/json',
                                       'Accept-Encoding': 'identity'}, method='POST')
        # Never log request headers, keys, provider bodies, or exception reprs.
        try:
            with self.opener(req, timeout=self.timeout) as response:
                raw = response.read(131073)
                content_type = response.headers.get('Content-Type', 'application/json')
                encoding = response.headers.get('Content-Encoding', '')
                # Offline opener mocks may omit real HTTP header objects.
                content_type = content_type if isinstance(content_type, str) else 'application/json'
                encoding = encoding if isinstance(encoding, str) else ''
            if len(raw) > 131072:
                raise RouterError('Router response exceeds 128 KiB')
            content = decode_response(raw, content_type, encoding, self.last_response_diagnostic)
            # Store only normalized text for offline replay; no headers, IDs,
            # provider fields, usage, credentials or raw error bodies.
            safe_content = content.replace(key, '[REDACTED]')
            self.last_response_fixture = {'choices': [{'message': {'content': safe_content}}]}
        except error.HTTPError as exc:
            raise RouterError(f'9Router HTTP {exc.code}; no request retried') from None
        except (TimeoutError, socket.timeout):
            raise RouterError('9Router timeout; no request retried') from None
        except error.URLError:
            raise RouterError('9Router connection failed; no request retried') from None
        except ResponseError as exc:
            raise RouterError('9Router response: ' + str(exc)) from None
        return parse_plan(content)
