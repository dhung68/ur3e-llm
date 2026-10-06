from types import SimpleNamespace
from unittest.mock import Mock
import json,time
from hri_bai3_perception.node import Perception


def test_missing_images_publish_error_not_empty_scene():
 n=SimpleNamespace(config={'frame':'world'},rgb={},depth={},info=None,last_pair=None,last_received=0,publisher=Mock())
 Perception.process(n)
 state=json.loads(n.publisher.publish.call_args[0][0].data)
 assert 'error' in state and state['objects']=={}


def test_frame_mismatch_rejected():
 stamp=SimpleNamespace(sec=1,nanosec=0)
 rgb=SimpleNamespace(header=SimpleNamespace(stamp=stamp,frame_id='front'))
 depth=SimpleNamespace(header=SimpleNamespace(stamp=stamp,frame_id='wrong_frame'))
 n=SimpleNamespace(config={'frame':'world'},rgb={1:(rgb,time.monotonic())},depth={1:(depth,time.monotonic())},
    info=SimpleNamespace(header=SimpleNamespace(frame_id='front')),last_pair=None,last_received=0,publisher=Mock())
 Perception.process(n)
 assert 'frame mismatch' in json.loads(n.publisher.publish.call_args[0][0].data)['error']
