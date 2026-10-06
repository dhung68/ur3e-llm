import gzip
import json
import pytest
from ur3_llm_control.router_response import ResponseError, decode_response
from ur3_llm_control.planning import parse_plan, validate_plan

PLAN={'steps':[{'skill':'home'},{'skill':'pick','object':'red_cube'}, {'skill':'place','object':'red_cube','zone':'zone_b'},{'skill':'home'}]}


def test_standard_choices_message_content():
    fixture={'choices':[{'message':{'content':json.dumps(PLAN)}}]}
    info={}
    assert validate_plan(parse_plan(decode_response(json.dumps(fixture).encode(),diagnostic=info)))==PLAN
    assert info['content_path']=='choices[0].message.content'


def sse():
    text=json.dumps(PLAN,ensure_ascii=False)
    parts=[text[:40],text[40:]]
    frames=[{'choices':[{'delta':{'role':'assistant'}}]}]+[{'choices':[{'delta':{'content':part}}]} for part in parts]+[{'choices':[{'delta':{},'finish_reason':'stop'}]}]
    return ''.join('data: '+json.dumps(frame)+'\n\n' for frame in frames)+'data: [DONE]\n\n'


def test_sse_delta_is_reassembled_before_json_validation():
    info={}
    assert validate_plan(parse_plan(decode_response(sse().encode(),'text/event-stream',diagnostic=info)))==PLAN
    assert info['wire_format']=='sse' and info['frame_count']==4
    assert info['content_path']=='choices[0].delta.content'


def test_sse_without_content_type_and_gzip_json():
    assert parse_plan(decode_response(sse().encode()))==PLAN
    fixture=json.dumps({'choices':[{'message':{'content':json.dumps(PLAN)}}]}).encode()
    assert parse_plan(decode_response(gzip.compress(fixture),content_encoding='gzip'))==PLAN


@pytest.mark.parametrize('wire', [b'{"error":{"message":"SECRET"}}',b'{"choices":[]}',
    b'{"choices":[{"message":{"content":null}}]}', b'data: {"choices":[{"delta":{"content":"{}"}}]}\n\n',
    b'data: {"error":{"message":"SECRET"}}\n\n',
    b'data: {"choices":[{"delta":{},"finish_reason":"length"}]}\n\n'])
def test_error_or_truncated_frames_are_refused_without_body_echo(wire):
    with pytest.raises(ResponseError) as exc:decode_response(wire)
    assert 'SECRET' not in str(exc.value)
