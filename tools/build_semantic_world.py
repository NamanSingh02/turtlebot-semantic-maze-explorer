#!/usr/bin/env python3
"""Building wall-side benches and recognizable composite object geometry."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")


def element(parent, tag, text=None, **attributes):
    item = ET.SubElement(parent, tag, attributes)
    if text is not None:
        item.text = str(text)
    return item


def shape(link, name, kind, size, pose, color):
    for tag in ("visual", "collision"):
        item = element(link, tag, name=name)
        element(item, "pose", " ".join(map(str, pose)))
        geometry = element(item, "geometry")
        primitive = element(geometry, kind)
        if kind == "box":
            element(primitive, "size", " ".join(map(str, size)))
        elif kind == "cylinder":
            element(primitive, "radius", size[0])
            element(primitive, "length", size[1])
        else:
            element(primitive, "radius", size[0])
        if tag == "visual":
            material = element(item, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def model(world, name, pose):
    item = element(world, "model", name=name)
    element(item, "static", "true")
    element(item, "pose", " ".join(map(str, pose)))
    return element(item, "link", name="body")


def hollow(link, radius, height, color):
    shape(link, "bottom", "cylinder", [radius, 0.006], [0, 0, 0.003, 0, 0, 0], color)
    for i in range(24):
        angle = 2 * math.pi * i / 24
        shape(
            link,
            f"rim_{i}",
            "box",
            [0.008, radius * 0.29, height],
            [
                radius * math.cos(angle),
                radius * math.sin(angle),
                height / 2,
                0,
                0,
                angle,
            ],
            color,
        )


def object_shapes(link, kind):
    white, blue = ".92 .92 .88 1", ".08 .3 .8 1"
    if kind in ("cup", "bowl"):
        radius, height = (0.035, 0.075) if kind == "cup" else (0.055, 0.035)
        hollow(link, radius, height, white)
        if kind == "cup":
            for i, (size, pose) in enumerate(
                [
                    ([0.025, 0.009, 0.009], [0.047, 0, 0.060, 0, 0, 0]),
                    ([0.009, 0.009, 0.040], [0.059, 0, 0.040, 0, 0, 0]),
                    ([0.025, 0.009, 0.009], [0.047, 0, 0.020, 0, 0, 0]),
                ]
            ):
                shape(link, f"handle_{i}", "box", size, pose, white)
    elif kind in ("bottle", "vase"):
        shape(link, "body", "cylinder", [0.032, 0.095], [0, 0, 0.0475, 0, 0, 0], blue)
        shape(link, "shoulder", "sphere", [0.032], [0, 0, 0.094, 0, 0, 0], blue)
        shape(link, "neck", "cylinder", [0.012, 0.045], [0, 0, 0.125, 0, 0, 0], blue)
        if kind == "bottle":
            shape(
                link, "cap", "cylinder", [0.014, 0.012], [0, 0, 0.153, 0, 0, 0], white
            )
        else:
            hollow(link, 0.020, 0.012, blue)
    elif kind == "book":
        shape(link, "pages", "box", [0.135, 0.09, 0.016], [0, 0, 0.011, 0, 0, 0], white)
        for i, z in enumerate((0.002, 0.021)):
            shape(
                link,
                f"cover_{i}",
                "box",
                [0.14, 0.095, 0.004],
                [0, 0, z, 0, 0, 0],
                ".6 .08 .03 1",
            )
    elif kind == "laptop":
        shape(
            link,
            "keyboard",
            "box",
            [0.19, 0.14, 0.012],
            [0, 0, 0.006, 0, 0, 0],
            ".15 .15 .16 1",
        )
        shape(
            link,
            "lid",
            "box",
            [0.19, 0.010, 0.13],
            [0, 0.058, 0.069, -0.2, 0, 0],
            ".12 .12 .13 1",
        )
        shape(
            link,
            "screen",
            "box",
            [0.17, 0.002, 0.11],
            [0, 0.049, 0.069, -0.2, 0, 0],
            ".1 .45 .7 1",
        )
    elif kind == "mouse":
        shape(
            link,
            "base",
            "box",
            [0.045, 0.065, 0.025],
            [0, 0, 0.0125, 0, 0, 0],
            ".2 .2 .2 1",
        )
        shape(
            link,
            "wheel",
            "box",
            [0.006, 0.013, 0.004],
            [0, 0.013, 0.027, 0, 0, 0],
            white,
        )
    else:
        raise ValueError(kind)


def build():
    tree = ET.parse(ROOT / "tb_worlds/worlds/sim_house.sdf.xacro")
    world = tree.getroot().find("world")
    for state in list(world.findall("state")):
        world.remove(state)
    manifest = json.loads((ROOT / "tb_worlds/worlds/semantic_benches.json").read_text())
    for bench in manifest["benches"]:
        pose = bench["pose"]
        link = model(world, bench["id"], pose)
        shape(
            link,
            "top",
            "box",
            [1.2, 0.36, 0.025],
            [0, 0, 0.1175, 0, 0, 0],
            ".45 .25 .1 1",
        )
        for i, (x, y) in enumerate(
            ((-0.52, -0.12), (-0.52, 0.12), (0.52, -0.12), (0.52, 0.12))
        ):
            shape(
                link,
                f"leg_{i}",
                "box",
                [0.035, 0.035, 0.105],
                [x, y, 0.0525, 0, 0, 0],
                ".25 .15 .08 1",
            )
        for index, kind in enumerate(bench["objects"]):
            offset = (index - 1) * 0.36
            x = pose[0] + offset * math.cos(pose[5])
            y = pose[1] + offset * math.sin(pose[5])
            link = model(world, f"{bench['id']}_{kind}", [x, y, 0.13, 0, 0, pose[5]])
            object_shapes(link, kind)
    ET.indent(tree, space="  ")
    tree.write(
        ROOT / "tb_worlds/worlds/sim_house_semantic.sdf.xacro",
        encoding="utf-8",
        xml_declaration=True,
    )


if __name__ == "__main__":
    build()
