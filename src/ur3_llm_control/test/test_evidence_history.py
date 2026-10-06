"""Evidence must describe the state when recorded, even after later placements."""
import json
from types import SimpleNamespace
from unittest.mock import Mock
from ur3_llm_control.skills import Skills


def test_later_placements_do_not_rewrite_previous_event(tmp_path):
    node = SimpleNamespace(
        events=[], evidence=tmp_path, held_object=None, holding_verified=False,
        config={"cubes": {}}, joints={},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        tcp_pose=Mock(side_effect=RuntimeError("No TF required for evidence test")),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    placed = {"red_cube": "zone_b"}
    Skills.record(node, "sequence_placement_verified", placed=placed)
    placed["yellow_cube"] = "zone_c"
    Skills.record(node, "sequence_placement_verified", placed=placed)
    stored = json.loads((tmp_path / "events.json").read_text())
    assert stored[0]["placed"] == {"red_cube": "zone_b"}
    assert stored[1]["placed"] == {"red_cube": "zone_b", "yellow_cube": "zone_c"}
