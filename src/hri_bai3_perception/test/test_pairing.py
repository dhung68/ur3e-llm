import time
from types import SimpleNamespace
import pytest
from hri_bai3_perception.node import pair


def msg(sec):return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=0)))


def test_exact_pair_ignores_new_unpaired_rgb():
 now=time.monotonic();r={1:(msg(1),now),2:(msg(2),now)};d={1:(msg(1),now)}
 assert pair(r,d,object(),None)[0]==1
 assert pair(r,d,object(),1) is None


def test_stale_matched_pair_is_rejected():
 r={1:(msg(1),time.monotonic()-2)}
 with pytest.raises(ValueError,match='stale'):pair(r,r,object(),None)
