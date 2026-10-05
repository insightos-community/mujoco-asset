# Copyright 2026 InsightOS
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""生成 R1 Pro 周转箱与专用夹具的工程评审模型。

脚本只读取 engineering-parameters.yaml。照片推断值集中在该文件，实测后替换
参数即可重新生成 STEP/STL/GLB，不需要改几何流程。视觉模型保留中空箱体与主要
加强筋；碰撞模型由少量凸体组成，适合动态 MuJoCo 物体。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "generated"


def _value(raw):
    return raw["value"] if isinstance(raw, dict) and "value" in raw else raw


@dataclass(frozen=True)
class ToteParameters:
    key: str
    length: float
    width: float
    height: float
    wall: float
    bottom: float
    rim_height: float
    rim_overhang: float
    rib_width: float
    rib_depth: float
    horizontal_rib_height: float
    grasp_groove_width: float
    grasp_groove_height: float
    grasp_groove_depth: float
    grasp_groove_top_offset: float
    mass_kg: float
    color: tuple[float, float, float, float]


def load_parameters() -> tuple[dict, list[ToteParameters]]:
    raw = yaml.safe_load((ROOT / "engineering-parameters.yaml").read_text(encoding="utf-8"))
    common = raw["tote_family"]["common"]
    variants = []
    for key, variant in raw["tote_family"]["variants"].items():
        length, width, height = variant["outer_size_mm"]
        variants.append(
            ToteParameters(
                key=key,
                length=length,
                width=width,
                height=height,
                wall=_value(common["wall_thickness_mm"]),
                bottom=_value(common["bottom_thickness_mm"]),
                rim_height=_value(common["rim_height_mm"]),
                rim_overhang=_value(common["rim_overhang_mm"]),
                rib_width=_value(common["vertical_rib_width_mm"]),
                rib_depth=_value(common["vertical_rib_depth_mm"]),
                horizontal_rib_height=_value(common["horizontal_rib_height_mm"]),
                grasp_groove_width=_value(common["grasp_groove_width_mm"]),
                grasp_groove_height=_value(common["grasp_groove_height_mm"]),
                grasp_groove_depth=_value(common["grasp_groove_depth_mm"]),
                grasp_groove_top_offset=_value(common["grasp_groove_top_offset_mm"]),
                mass_kg=_value(variant["empty_mass_kg"]),
                color=tuple(variant["color_rgba"]),
            )
        )
    _validate_fixed_gripper_interface(raw, variants)
    return raw, variants


def _validate_fixed_gripper_interface(raw: dict, variants: list[ToteParameters]) -> None:
    """确认不同箱体的短边凹槽能够容纳同一固定夹具。"""
    gripper = raw["gripper"]
    expected = float(gripper["closed_contact_span_mm"])
    tolerance = float(gripper["interface_span_tolerance_mm"])
    hook_width = float(gripper["lower_hook_width_mm"])
    minimum_groove_width = hook_width + 2 * tolerance
    for variant in variants:
        # 闭合跨度直接对应“箱沿上表面到凹槽上沿内侧”的距离。这里不再
        # 通过不存在的箱体内部构件高度反推接触点。
        actual = variant.grasp_groove_top_offset
        if abs(actual - expected) > tolerance:
            raise ValueError(
                f"{variant.key} 凹槽夹持跨度 {actual:.1f} mm 与固定夹具 "
                f"{expected:.1f} mm 不兼容；请调整箱体接口，不得缩放夹具"
            )
        if variant.grasp_groove_width < minimum_groove_width:
            raise ValueError(
                f"{variant.key} 抓取凹槽宽度仅 {variant.grasp_groove_width:.1f} mm，"
                f"无法容纳 {hook_width:.1f} mm 钩爪及双侧装配余量"
            )


def _grasp_groove_vertical_range(p: ToteParameters) -> tuple[float, float]:
    """返回短边抓取凹槽的底部和顶部高度，单位为毫米。"""
    top = p.height - p.grasp_groove_top_offset
    return top - p.grasp_groove_height, top


def _short_wall_parts(p: ToteParameters, side: str) -> dict[str, cq.Workplane]:
    """把带水平凹槽的短边拆成凸盒，明确保留钩脚扣入空间。"""
    sign = 1.0 if side == "pos" else -1.0
    x = sign * (p.length - p.wall) / 2
    groove_bottom, groove_top = _grasp_groove_vertical_range(p)
    side_width = (p.width - 2 * p.wall - p.grasp_groove_width) / 2
    parts: dict[str, cq.Workplane] = {}
    for name, y_sign in (("left", -1.0), ("right", 1.0)):
        y = y_sign * (p.grasp_groove_width / 2 + side_width / 2)
        parts[f"wall_{side}_side_{name}"] = cq.Workplane("XY").box(
            p.wall, side_width, p.height
        ).translate((x, y, p.height / 2))
    parts[f"wall_{side}_below_groove"] = cq.Workplane("XY").box(
        p.wall, p.grasp_groove_width, groove_bottom
    ).translate((x, 0, groove_bottom / 2))
    upper_height = p.height - groove_top
    parts[f"wall_{side}_above_groove"] = cq.Workplane("XY").box(
        p.wall, p.grasp_groove_width, upper_height
    ).translate((x, 0, groove_top + upper_height / 2))
    # grasp_groove_depth 表达从箱体外表面到凹槽背板前表面的真实可用深度。
    # 背板自身仍有 wall 厚度，因此其中心必须再向箱体内部移动半个 wall；
    # 否则 26mm 槽深会只剩 24mm，22mm 钩脚按 2mm 余量插入时恰好顶住
    # 背板，双臂轨迹会以对称关节误差超时，而不是进入可承载的侧面凹槽。
    backing_x = sign * (
        p.length / 2 - p.grasp_groove_depth - p.wall / 2
    )
    parts[f"wall_{side}_groove_backing"] = cq.Workplane("XY").box(
        p.wall, p.grasp_groove_width, p.grasp_groove_height
    ).translate((backing_x, 0, (groove_bottom + groove_top) / 2))
    # 侧面凹槽是箱壁自身形成的完整腔体，不是外壁开孔后另加一根横条。
    # 顶面缺失时，下钩即使进入槽内也没有可钩住的物理表面；底面和两端面
    # 同时补齐，保证视觉Mesh与碰撞Mesh表达同一个凹槽。
    groove_depth_center_x = sign * (p.length / 2 - p.grasp_groove_depth / 2)
    parts[f"wall_{side}_groove_ceiling"] = cq.Workplane("XY").box(
        p.grasp_groove_depth, p.grasp_groove_width, p.wall
    ).translate((groove_depth_center_x, 0, groove_top + p.wall / 2))
    parts[f"wall_{side}_groove_floor"] = cq.Workplane("XY").box(
        p.grasp_groove_depth, p.grasp_groove_width, p.wall
    ).translate((groove_depth_center_x, 0, groove_bottom - p.wall / 2))
    for name, y_sign in (("left", -1.0), ("right", 1.0)):
        parts[f"wall_{side}_groove_end_{name}"] = cq.Workplane("XY").box(
            p.grasp_groove_depth, p.wall, p.grasp_groove_height
        ).translate(
            (
                groove_depth_center_x,
                y_sign * (p.grasp_groove_width + p.wall) / 2,
                (groove_bottom + groove_top) / 2,
            )
        )
    return parts

def tote_parts(p: ToteParameters) -> dict[str, cq.Workplane]:
    """构造中空箱体和短边抓取凹槽。"""
    parts: dict[str, cq.Workplane] = {
        "bottom": cq.Workplane("XY").box(p.length, p.width, p.bottom).translate((0, 0, p.bottom / 2)),
        "wall_pos_y": cq.Workplane("XY").box(p.length, p.wall, p.height).translate((0, (p.width - p.wall) / 2, p.height / 2)),
        "wall_neg_y": cq.Workplane("XY").box(p.length, p.wall, p.height).translate((0, -(p.width - p.wall) / 2, p.height / 2)),
    }
    parts.update(_short_wall_parts(p, "pos"))
    parts.update(_short_wall_parts(p, "neg"))
    rim_z = p.height - p.rim_height / 2
    rim_l = p.length + 2 * p.rim_overhang
    rim_w = p.width + 2 * p.rim_overhang
    parts.update(
        {
            "rim_pos_y": cq.Workplane("XY").box(rim_l, p.rim_height, p.rim_height).translate((0, (rim_w - p.rim_height) / 2, rim_z)),
            "rim_neg_y": cq.Workplane("XY").box(rim_l, p.rim_height, p.rim_height).translate((0, -(rim_w - p.rim_height) / 2, rim_z)),
            "rim_pos_x": cq.Workplane("XY").box(p.rim_height, rim_w - 2 * p.rim_height, p.rim_height).translate(((rim_l - p.rim_height) / 2, 0, rim_z)),
            "rim_neg_x": cq.Workplane("XY").box(p.rim_height, rim_w - 2 * p.rim_height, p.rim_height).translate((-(rim_l - p.rim_height) / 2, 0, rim_z)),
        }
    )
    # 中部让位给抓取凹槽，竖筋只保留在凹槽两侧。
    for side, x in (("pos", p.length / 2 + p.rib_depth / 2), ("neg", -p.length / 2 - p.rib_depth / 2)):
        for index, y in enumerate((-0.38 * p.width, -0.25 * p.width, 0.25 * p.width, 0.38 * p.width)):
            parts[f"rib_{side}_{index}"] = cq.Workplane("XY").box(p.rib_depth, p.rib_width, p.height * 0.78).translate((x, y, p.height * 0.43))
        parts[f"rib_horizontal_{side}"] = cq.Workplane("XY").box(p.rib_depth, p.width * 0.82, p.horizontal_rib_height).translate((x, 0, p.height * 0.24))
    return parts



def tote_collision_parts(
    p: ToteParameters, visual_parts: dict[str, cq.Workplane]
) -> dict[str, cq.Workplane]:
    """返回只由凸体组成的物理模型；加强筋只保留在视觉模型。"""
    structural_prefixes = (
        "bottom",
        "wall_",
        "rim_",
    )
    return {
        name: part
        for name, part in visual_parts.items()
        if name.startswith(structural_prefixes)
    }


def gripper_parts(raw: dict) -> dict[str, cq.Workplane]:
    """生成固定尺寸夹具：45° 下沉框体、长导向下钩与主动上压片。"""
    g = raw["gripper"]
    adapter_x, adapter_y, adapter_z = g["adapter_plate_mm"]
    connector_length = g["connector_horizontal_length_mm"]
    connector_drop = g["connector_vertical_drop_mm"]
    connector_width = g["connector_width_mm"]
    wrist_height = g["connector_wrist_height_mm"]
    frame_x, frame_y, frame_z = g["contact_frame_mm"]
    hook_width = g["lower_hook_width_mm"]
    hook_drop = g["lower_hook_drop_mm"]
    hook_insert = g["lower_hook_insert_mm"]
    hook_t = g["lower_hook_thickness_mm"]
    pad_x, pad_y, pad_z = g["upper_clamp_pad_mm"]
    carriage_x, carriage_y, carriage_z = g["upper_clamp_carriage_mm"]
    clamp_overlap = g["upper_clamp_inboard_overlap_mm"]
    tip_x, tip_y, tip_z = g["replaceable_tip_mm"]

    adapter = cq.Workplane("XY").box(adapter_x, adapter_y, adapter_z)
    # 腕部真实孔位尚未实测；这里只表达 X 法向安装面和中心让位孔。
    adapter = adapter.faces(">X").workplane().hole(18.0)
    x0 = adapter_x / 2
    x1 = x0 + connector_length
    contact_centre_z = -connector_drop

    connector_outer = (
        cq.Workplane("XZ")
        .polyline(
            (
                (x0, -wrist_height / 2),
                (x1, contact_centre_z - wrist_height / 2),
                (x1, contact_centre_z + wrist_height / 2),
                (x0, wrist_height / 2),
            )
        )
        .close()
        .extrude(connector_width / 2, both=True)
    )
    # 照片中的腕部支撑是带大窗口的轻量框体，而不是实心三角块。
    # 中心线在竖直方向下沉 45°；左右臂只镜像安装，不翻转倾角。
    window_margin = 9.0
    inner_half_height = max(wrist_height / 2 - window_margin, 5.0)
    inner_end_z = -connector_drop * (
        connector_length - window_margin
    ) / connector_length
    connector_window = (
        cq.Workplane("XZ")
        .polyline(
            (
                (x0 + window_margin, -inner_half_height),
                (x1 - window_margin, inner_end_z - inner_half_height),
                (x1 - window_margin, inner_end_z + inner_half_height),
                (x0 + window_margin, inner_half_height),
            )
        )
        .close()
        .extrude(connector_width, both=True)
    )
    angled_connector = connector_outer.cut(connector_window)

    frame_bar = max(hook_t + 2.0, 10.0)
    frame_x_centre = x1 + frame_x / 2
    # 黑色接触框下窄上宽，中央保留大窗口，接近照片中的紧凑 C 型壳体。
    outer = (
        cq.Workplane("YZ")
        .polyline(
            (
                (-frame_y / 2, frame_z / 2),
                (frame_y / 2, frame_z / 2),
                (frame_y * 0.42, -frame_z / 2),
                (-frame_y * 0.42, -frame_z / 2),
            )
        )
        .close()
        .extrude(frame_x / 2, both=True)
        .translate((frame_x_centre, 0, contact_centre_z))
    )
    window = (
        cq.Workplane("YZ")
        .polyline(
            (
                (-frame_y / 2 + frame_bar, frame_z / 2 - frame_bar),
                (frame_y / 2 - frame_bar, frame_z / 2 - frame_bar),
                (frame_y * 0.42 - frame_bar, -frame_z / 2 + frame_bar),
                (-frame_y * 0.42 + frame_bar, -frame_z / 2 + frame_bar),
            )
        )
        .close()
        .extrude(frame_x, both=True)
        .translate((frame_x_centre, 0, contact_centre_z))
    )
    contact_frame = outer.cut(window)

    frame_bottom = contact_centre_z - frame_z / 2
    hook_bearing_z = frame_bottom - hook_drop + hook_t
    groove_height = float(
        raw["tote_family"]["common"]["grasp_groove_height_mm"]["value"]
    )
    # 导轨需要随钩脚进入短边抓取凹槽，长度受真实槽高约束。旧外形把
    # 导轨一直延伸到接触框中部，水平插入时会被箱沿挡住；上下各留
    # 半个钩厚的装配间隙，承载面最终扣住凹槽上沿内侧。
    rail_length = groove_height - 2 * hook_t
    frame_contact_x = x1 + frame_x
    # 参考视频中每个末端是一个宽的 C 形钩，而不是两根并排小钩。竖直
    # 导轨位于箱壁外侧，短钩脚伸入水平凹槽；钩角越过槽口后扣住槽沿，
    # 内端仍不能碰到 26 mm 深凹槽的背板。
    rail_x = frame_contact_x - hook_t / 2
    hook_guide_rail = cq.Workplane("XY").box(
        3.0, 12.0, rail_length
    ).translate(
        (
            rail_x - 1.5,
            0,
            hook_bearing_z + rail_length / 2,
        )
    )
    hook_spine_length = rail_length * 0.56
    hook_spine = cq.Workplane("XY").box(
        hook_t, hook_width, hook_spine_length
    ).translate(
        (
            rail_x,
            0,
            hook_bearing_z + hook_spine_length / 2,
        )
    )
    foot = cq.Workplane("XY").box(
        hook_insert, hook_width, hook_t
    ).translate(
        (
            rail_x + hook_insert / 2,
            0,
            hook_bearing_z - hook_t / 2,
        )
    )
    retaining_lip = cq.Workplane("XY").box(
        hook_t, hook_width, hook_t * 2
    ).translate(
        (
            rail_x + hook_insert - hook_t / 2,
            0,
            hook_bearing_z + hook_t / 2,
        )
    )
    fixed_lower_hooks = hook_spine.union(foot).union(retaining_lip)

    # 两种箱体共享固定 86 mm 夹持跨度，不因箱型改变夹具尺寸。
    clamp_contact_z = hook_bearing_z + g["closed_contact_span_mm"]
    frame_outer_x = x1 + frame_x
    # 主动件由框内滑块和短压板组成。压板只跨入箱沿 14 mm，避免旧模型
    # 的大横板穿入箱壁；橡胶压片覆盖主要承压宽度并随滑块一起运动。
    tip_centre_x = frame_outer_x + clamp_overlap - tip_x / 2
    pad_centre_x = frame_outer_x + clamp_overlap - pad_x / 2
    carriage = cq.Workplane("XY").box(
        carriage_x, carriage_y, carriage_z
    ).translate(
        (
            frame_outer_x - carriage_x / 2,
            0,
            clamp_contact_z + carriage_z / 2,
        )
    )
    pressure_plate = cq.Workplane("XY").box(
        pad_x, pad_y, pad_z
    ).translate(
        (
            pad_centre_x,
            0,
            clamp_contact_z + tip_z + pad_z / 2,
        )
    )
    active_upper_clamp = carriage.union(pressure_plate)
    replaceable_tip = cq.Workplane("XY").box(
        tip_x, tip_y, tip_z
    ).translate(
        (
            tip_centre_x,
            0,
            clamp_contact_z + tip_z / 2,
        )
    )
    return {
        "wrist_adapter": adapter,
        "angled_connector": angled_connector,
        "hook_guide_rail": hook_guide_rail,
        "contact_frame": contact_frame,
        "fixed_lower_hooks": fixed_lower_hooks,
        "active_upper_clamp": active_upper_clamp,
        "replaceable_tip": replaceable_tip,
    }


def _compound(parts: dict[str, cq.Workplane]) -> cq.Compound:
    return cq.Compound.makeCompound([part.val() for part in parts.values()])


def _export_parts(
    parts: dict[str, cq.Workplane],
    directory: Path,
    color: tuple[int, int, int, int],
    prefix_colors: dict[str, tuple[int, int, int, int]] | None = None,
) -> None:
    """输出米制 STL/GLB，并让评审 GLB 保留关键分件颜色。"""
    directory.mkdir(parents=True, exist_ok=True)
    # 部件名称会随评审结论调整；必须先清掉同目录旧 STL，避免新旧几何被
    # MuJoCo 评审台架同时枚举。这里只删除本生成器自己的派生输出。
    for stale in directory.glob("*.stl"):
        stale.unlink()
    scene = trimesh.Scene()
    for name, part in parts.items():
        stl_path = directory / f"{name}.stl"
        cq.exporters.export(part, str(stl_path))
        mesh = trimesh.load_mesh(stl_path, file_type="stl")
        mesh.apply_scale(0.001)
        part_color = color
        for prefix, candidate in (prefix_colors or {}).items():
            if name.startswith(prefix):
                part_color = candidate
                break
        mesh.visual.face_colors = part_color
        mesh.export(stl_path)
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    (directory / "visual.glb").write_bytes(scene.export(file_type="glb"))


def generate() -> None:
    raw, variants = load_parameters()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    family_manifest = {
        "schema_version": 1,
        "authoring_length_unit": "millimetre",
        "mesh_length_unit": "metre",
        "coordinate_frame": "right-handed-z-up",
        "variants": [],
    }
    family_shapes: list[cq.Shape] = []
    family_offset = 0.0
    for p in variants:
        parts = tote_parts(p)
        target = OUTPUT / p.key
        target.mkdir(parents=True, exist_ok=True)
        cq.exporters.export(_compound(parts), str(target / f"{p.key}.step"))
        _export_parts(
            parts,
            target / "visual",
            tuple(round(c * 255) for c in p.color),
            {},
        )
        collision = tote_collision_parts(p, parts)
        _export_parts(collision, target / "collision", (80, 80, 80, 255))
        # 窄加强筋只参与视觉，不进入物理接触，避免窄面抖动。
        family_manifest["variants"].append({"id": p.key, "outer_size_mm": [p.length, p.width, p.height], "mass_kg": p.mass_kg})
        family_shapes.extend(part.translate((0, family_offset, 0)).val() for part in parts.values())
        family_offset += p.width + 180.0
    cq.exporters.export(cq.Compound.makeCompound(family_shapes), str(OUTPUT / "tote-family.step"))
    gripper = gripper_parts(raw)
    cq.exporters.export(_compound(gripper), str(OUTPUT / "tote-gripper.step"))
    cq.exporters.export(gripper["wrist_adapter"], str(OUTPUT / "wrist-adapter.step"))
    cq.exporters.export(gripper["replaceable_tip"], str(OUTPUT / "replaceable-tips.step"))
    _export_parts(
        gripper,
        OUTPUT / "gripper-visual",
        (35, 38, 44, 255),
        {
            "wrist_adapter": (31, 34, 40, 255),
            "hook_guide_rail": (158, 165, 174, 255),
            "fixed_lower_hooks": (18, 20, 24, 255),
            "replaceable_tip": (7, 7, 8, 255),
        },
    )
    (OUTPUT / "tote-family.json").write_text(json.dumps(family_manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"generated: {OUTPUT}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.parse_args()
    generate()
