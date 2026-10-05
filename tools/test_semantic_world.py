"""Checking bench counts, wall contact, object sets, and world expansion."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_bench_contract_and_wall_contact():
    manifest = json.loads((ROOT / "tb_worlds/worlds/semantic_benches.json").read_text())
    world = (
        ET.parse(ROOT / "tb_worlds/worlds/sim_house_semantic.sdf.xacro")
        .getroot()
        .find("world")
    )
    house = world.find("model[@name='sim_house']")
    house_pose = list(map(float, house.findtext("pose").split()))
    sets = set()
    for bench in manifest["benches"]:
        sets.add(tuple(sorted(bench["objects"])))
        assert 3 <= len(bench["objects"]) <= 5
        assert world.find(f"model[@name='{bench['id']}']") is not None
        wall = house.find(f"link[@name='{bench['wall']}']")
        pose = list(map(float, wall.findtext("pose").split()))
        dx = bench["pose"][0] - (house_pose[0] + pose[0])
        dy = bench["pose"][1] - (house_pose[1] + pose[1])
        distance = abs(-math.sin(pose[5]) * dx + math.cos(pose[5]) * dy)
        assert abs(distance - (0.15 / 2 + 0.36 / 2)) < 1e-4
        for kind in bench["objects"]:
            assert world.find(f"model[@name='{bench['id']}_{kind}']") is not None
    assert len(sets) == len(manifest["benches"]) == 3
    assert world.find("state") is None
