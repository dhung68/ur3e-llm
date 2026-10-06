"""Focused assignment 3 occupancy/staleness/fail-stop checks only."""
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from ur3_llm_control.bai3_state import validate_bai3, free_temporaries, with_temporaries, CameraStateProvider
from ur3_llm_control.bai3_task import execute
from ur3_llm_control.skills import SkillResult
from ur3_llm_control.planning import PlanError

@pytest.fixture
def state():
    from pathlib import Path
    cfg = json.loads((Path(__file__).parents[2]/'hri_bai3_environment/config/scene.json').read_text())
    # Test fixture only, never imported by runtime planner/control.
    positions = dict(zip(cfg['cubes'], [[.22,-.14,.095],[.22,0,.095],[.38,0,.095],[.18,.15,.095],[.16,-.21,.095]]))
    return with_temporaries(cfg, free_temporaries(positions,cfg)), positions


def plan(blocker=True):
    steps=[{'skill':'home'}]
    if blocker: steps += [{'skill':'pick','object':'blue_cube'}, {'skill':'place','object':'blue_cube','zone':'temp_1'}, {'skill':'home'}]
    return {'steps':steps+[{'skill':'pick','object':'red_cube'},{'skill':'place','object':'red_cube','zone':'zone_b'},{'skill':'home'}]}


def test_transitions_free_occupied_zone(state):
    cfg, p = state
    assert validate_bai3(plan(),cfg,p)
    with pytest.raises(PlanError,match='occupied'): validate_bai3(plan(False),cfg,p)


def test_missing_object_not_empty(state):
    cfg,p=state; p.pop('blue_cube')
    with pytest.raises(PlanError,match='five'): validate_bai3(plan(),cfg,p)


def test_buffer_dynamically_occupied(state):
    cfg,p=state; p['green_cube']=cfg['temporary_candidates']['temp_1']
    assert 'temp_1' not in free_temporaries(p,cfg)
    with pytest.raises(PlanError,match='occupied'): validate_bai3(plan(),cfg,p)

@pytest.mark.parametrize('replacement',[
 {'skill':'teleport','object':'red_cube'},
 {'skill':'pick','object':'unknown_cube'},
 {'skill':'place','object':'blue_cube','zone':'temp_99'},
 {'skill':'place','object':'red_cube','zone':'zone_a'},
 {'skill':'pick','object':'blue_cube','joints':[0]},
])
def test_bad_schema_and_order(state,replacement):
    cfg,p=state; q=plan();q['steps'][1]=replacement
    with pytest.raises(PlanError):validate_bai3(q,cfg,p)


def test_camera_stale_error_and_missing():
    provider=object.__new__(CameraStateProvider)
    provider.received=time.monotonic()-2;provider.message={}
    with pytest.raises(RuntimeError,match='stale'):provider.get('red_cube')
    provider.received=time.monotonic();provider.message={'error':'camera unplugged'}
    with pytest.raises(RuntimeError,match='unplugged'):provider.get('red_cube')
    provider.message={'objects':{},'errors':{'red_cube':'occluded'}}
    with pytest.raises(RuntimeError,match='occluded'):provider.get('red_cube')


def test_executor_stops_first_error(state):
    cfg,p=state;skills=Mock();skills.config=cfg;skills.held_object=None
    skills.contact_object=None;skills.holding_verified=False;skills.provider.snapshot.return_value=p
    skills.home.return_value=SkillResult(False,'home','planning failed',None,False,None,False)
    with pytest.raises(RuntimeError,match='planning failed'):execute(plan(),skills,Mock())
    skills.pick.assert_not_called();skills.place.assert_not_called()
