import json
from dataclasses import dataclass
from types import SimpleNamespace as NS
from unittest.mock import Mock
from urllib.error import URLError, HTTPError

import pytest
from ur3_llm_control.planning import (PlanError, RouterError, RouterPlanner,
                                      parse_plan, profile, validate_plan)
from ur3_llm_control.task_executor import ExecutionError, execute_plan


def plan(obj='red_cube', zone='zone_b'):
    return {'steps': [{'skill':'home'}, {'skill':'pick','object':obj},
                      {'skill':'place','object':obj,'zone':zone}, {'skill':'home'}]}


@pytest.mark.parametrize('raw', ['not json', '```json\n{}\n```', '{"steps":[],"steps":[]}', '{"steps":NaN}'])
def test_bad_json(raw):
    with pytest.raises(PlanError): parse_plan(raw)


@pytest.mark.parametrize('value', [[], {}, {'steps':[]}, {'steps':True},
    {'steps':[{'skill':'exec','code':'danger'}]}, {'steps':[{'skill':'home','joint':0}]},
    {'steps':[{'skill':'pick','object':'green_cube'}]},
    {'steps':[{'skill':'pick','object':['red_cube']}]},
    {'steps':[{'skill':'place','object':'red_cube','zone':'zone_a'}]},
    {'steps':[{'skill':'pick','object':'red_cube'},{'skill':'home'}]},
    {'steps':[{'skill':'pick','object':'red_cube'}]},
    {'steps':[{'skill':'pick','object':'red_cube'},{'skill':'place','object':'blue_cube','zone':'zone_a'}]},
    plan(zone='zone_d'), plan(zone=['zone_a']),
    {'steps':[{'skill':'pick','object':'red_cube'}, {'skill':'pick','object':'blue_cube'}]},
    {'steps':plan()['steps']+plan('blue_cube')['steps']}])
def test_schema_and_sequence_rejected(value):
    with pytest.raises(PlanError): validate_plan(value)


def test_student_and_generic_plans():
    assert profile()['P'] == 2
    assert profile()['student_name'] == ''
    assert profile()['zone_assignment'] == {'zone_a':'yellow_cube','zone_b':'red_cube','zone_c':'blue_cube'}
    for obj in ('red_cube','yellow_cube','blue_cube'):
        for zone in ('zone_a','zone_b','zone_c'):
            assert validate_plan(plan(obj,zone))  # schema only, no robot cycles


@pytest.mark.parametrize('exc', [TimeoutError('secret-key'), URLError('secret-key'),
                                  HTTPError('url',401,'secret-key',{},None)])
def test_network_failure_no_retry_no_secret(monkeypatch, exc):
    monkeypatch.setenv('ROUTER_API_KEY','secret-key')
    opener = Mock(side_effect=exc)
    with pytest.raises(RouterError) as found:
        RouterPlanner(opener=opener).generate('Gắp khối đỏ', profile())
    assert 'secret-key' not in str(found.value)
    opener.assert_called_once()


def test_missing_key_no_request(monkeypatch):
    monkeypatch.delenv('ROUTER_API_KEY',raising=False)
    opener = Mock()
    with pytest.raises(RouterError,match='ROUTER_API_KEY'):
        RouterPlanner(opener=opener).generate('Pick red',profile())
    opener.assert_not_called()


def test_llm_receives_untranslated_user_command_and_context(monkeypatch):
    monkeypatch.setenv('ROUTER_API_KEY','private-key')
    response = Mock()
    response.__enter__ = Mock(return_value=response)
    response.__exit__ = Mock(return_value=False)
    response.read.return_value=json.dumps({'choices':[{'message':{'content':json.dumps(plan())}}]}).encode()
    opener=Mock(return_value=response)
    command='Arrange all objects according to my student ID'
    assert RouterPlanner(opener=opener).generate(command,profile()) == plan()
    req=opener.call_args.args[0]
    payload=json.loads(req.data)
    assert payload['stream'] is False
    assert req.get_header('Accept') == 'application/json'
    assert req.get_header('Accept-encoding') == 'identity'
    user=json.loads(payload['messages'][1]['content'])
    assert user['command']==command and user['context']['student']['P']==2
    assert 'private-key' not in req.data.decode()


@dataclass
class Result:
    success: bool
    skill: str
    reason: str='mock'


def fake():
    node=NS(held_object=None,contact_object=None,holding_verified=False,
            spin=Mock(),verify_zone_empty=Mock(),check_carried=Mock(),sync_scene=Mock())
    node.home=Mock(return_value=Result(True,'home'))
    def pick(name):
        node.held_object=node.contact_object=name;node.holding_verified=True
        return Result(True,'pick')
    def place(name,zone):
        node.held_object=node.contact_object=None;node.holding_verified=False
        return Result(True,'place')
    node.pick=Mock(side_effect=pick);node.place=Mock(side_effect=place)
    node.config={'cubes':{'red_cube':{'size':[.03]*3}},
                 'zones':{'zone_b':{'placement_cube_center':[.38,0,.095],'size_xy':[.08]*2}}}
    node.provider=NS(get=Mock(return_value=NS(pose=NS(position=NS(x=.38,y=0,z=.095),orientation=NS(x=0,y=0,z=0,w=1)))))
    # Initial fresh-world cube; later calls observe the completed placement.
    first=NS(pose=NS(position=NS(x=.22,y=-.14,z=.095),orientation=NS(x=0,y=0,z=0,w=1)))
    settled=node.provider.get.return_value
    node.provider.get.side_effect=lambda name: first if node.place.call_count==0 else settled
    return node


def test_executor_stops_on_skill_error():
    node=fake();node.pick=Mock(return_value=Result(False,'pick','lost object'))
    with pytest.raises(ExecutionError,match='lost object'):execute_plan(plan(),node)
    node.place.assert_not_called();assert node.home.call_count==1


def test_whole_plan_invalid_has_no_motion():
    node=fake();bad=plan();bad['steps'].append({'skill':'unsafe'})
    with pytest.raises(PlanError):execute_plan(bad,node)
    node.home.assert_not_called();node.pick.assert_not_called()


def test_occupied_target_refused_before_home_or_pick():
    node=fake();node.verify_zone_empty.side_effect=RuntimeError('zone_b occupied')
    with pytest.raises(RuntimeError,match='occupied'):execute_plan(plan(),node)
    node.home.assert_not_called();node.pick.assert_not_called()


def test_executor_success_and_final_physical_check():
    node=fake();result=execute_plan(plan(),node)
    assert result['success'] and result['final_objects']['red_cube']['zone']=='zone_b'
    assert node.home.call_count==2 and node.place.call_count==1
    node.sync_scene.assert_called_once()


def test_final_pose_error_is_failure():
    node=fake();node.provider.get.return_value.pose.position.x=.9
    with pytest.raises(ExecutionError,match='not fully settled'):execute_plan(plan(),node)
    assert node.home.call_count==1


def test_false_pick_success_blocks_transport():
    node=fake();node.pick=Mock(return_value=Result(True,'pick'))
    with pytest.raises(ExecutionError,match='without verified'):execute_plan(plan(),node)
    node.place.assert_not_called()


def test_own_object_already_in_target_is_refused():
    node=fake()
    node.provider.get.side_effect=None
    with pytest.raises(ExecutionError,match='already occupied'):execute_plan(plan(),node)
    node.home.assert_not_called()


def test_screenshot_failure_does_not_interrupt_result(monkeypatch,tmp_path):
    from ur3_llm_control import llm_node, screenshots
    node=NS(spin=Mock())
    emit=Mock()
    monkeypatch.setattr(screenshots,'capture_windows',Mock(side_effect=TimeoutError('X11 timeout')))
    llm_node.capture_completion(node,tmp_path,emit,'done')
    assert emit.call_args.args[0]=='SCREENSHOT UNAVAILABLE'
    node.spin.assert_called_once()


def test_new_cli_executor_exception_returns_nonzero(monkeypatch,tmp_path):
    import rclpy
    from rclpy import node as ros_node
    from ur3_llm_control import llm_node, skills
    class FakeNode:
        def __init__(self,*args,**kwargs):pass
        def declare_parameter(self,name,value):return NS(value=value)
        def create_publisher(self,*args):return Mock()
        def destroy_node(self):pass
    class FakeSkills(FakeNode):
        held_object=None;contact_object=None;holding_verified=False
        results=[];events=[]
        def spin(self,*args):pass
    monkeypatch.setattr(ros_node,'Node',FakeNode)
    monkeypatch.setattr(skills,'Skills',FakeSkills)
    monkeypatch.setattr(rclpy,'init',Mock());monkeypatch.setattr(rclpy,'ok',lambda:False)
    monkeypatch.setattr(llm_node,'execute_plan',Mock(side_effect=ExecutionError('final pose stale')))
    saved=tmp_path/'saved.json';saved.write_text(json.dumps(plan()))
    root=tmp_path/'failure'
    assert llm_node.main(['--evidence',str(root),'--plan-file',str(saved)])==1
    report=json.loads((root/'task.json').read_text())
    assert report['success'] is False and 'final pose stale' in report['error']
    assert 'TASK FAILED' in (root/'task.log').read_text()
