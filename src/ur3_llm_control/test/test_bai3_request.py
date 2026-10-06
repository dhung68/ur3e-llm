import json
from pathlib import Path
from unittest.mock import patch
from urllib.error import URLError
import pytest
from ur3_llm_control.planning import RouterPlanner, RouterError


def test_camera_context_custom_prompt_and_single_request():
    seen=[]
    class Response:
        headers={'Content-Type':'application/json'}
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def read(self,n):return json.dumps({'choices':[{'message':{'content':'{"steps":[{"skill":"home"}]}'}}]}).encode()
    def opener(req,timeout):seen.append(json.loads(req.data));return Response()
    context={'objects':['red_cube','yellow_cube','blue_cube','green_cube','purple_cube'],
             'camera_state':{'zone_occupancy':{'zone_b':['blue_cube']}},'available_temporaries':{'temp_1':[.31,-.08,.095]}}
    with patch.dict('os.environ',{'ROUTER_API_KEY':'offline-test-only'}):
        result=RouterPlanner(opener=opener).generate_context('Đặt red vào zone_b',context,
                  Path(__file__).parents[1]/'ur3_llm_control/bai3_prompt.txt')
    assert result=={'steps':[{'skill':'home'}]}
    assert len(seen)==1 and seen[0]['stream'] is False
    assert 'green_cube' in seen[0]['messages'][1]['content']
    assert 'temp_1' in seen[0]['messages'][1]['content']


def test_network_failure_no_retry():
    calls=[]
    def opener(*args,**kwargs):calls.append(1);raise URLError('offline network error')
    with patch.dict('os.environ',{'ROUTER_API_KEY':'offline-test-only'}),pytest.raises(RouterError,match='connection failed'):
        RouterPlanner(opener=opener).generate_context('Place red in b',{},Path(__file__).parents[1]/'ur3_llm_control/bai3_prompt.txt')
    assert len(calls)==1
