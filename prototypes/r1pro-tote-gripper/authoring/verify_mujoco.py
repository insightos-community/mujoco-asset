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

"""在独立 MuJoCo 台架中验证周转箱和双臂专用夹具。

验证只使用接触、关节和执行器。脚本不会在步进过程中修改箱体 qpos，
也不会创建 weld/equality 等隐藏附着约束。
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from pathlib import Path

# smoke 默认在无桌面的 CI/开发机运行。必须在导入 mujoco 前选定 EGL，
# 否则部分环境会退回 GLFW，并在没有 DISPLAY 时错误退出。
os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v3 as iio
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / "generated"


@dataclass(frozen=True)
class Variant:
    key: str
    length: float
    width: float
    height: float
    mass: float
    groove_top: float
    groove_bottom: float
    groove_depth: float
    hook_length: float
    hook_thickness: float
    backing_clearance: float
    closed_contact_span: float

    @property
    def hook_center_insertion(self) -> float:
        return self.groove_depth - self.hook_length / 2 - self.backing_clearance

    @property
    def hook_center_z(self) -> float:
        # tote body初始离地3mm；钩脚上表面与凹槽上沿内侧相接。
        return 0.003 + self.groove_top - self.hook_thickness / 2


def _value(raw):
    return raw["value"] if isinstance(raw, dict) and "value" in raw else raw


def _variant(key: str) -> tuple[dict, Variant]:
    raw = yaml.safe_load((ROOT / "engineering-parameters.yaml").read_text(encoding="utf-8"))
    common = raw["tote_family"]["common"]
    source = raw["tote_family"]["variants"][key]
    length, width, height = (float(v) / 1000.0 for v in source["outer_size_mm"])
    groove_top = height - float(_value(common["grasp_groove_top_offset_mm"])) / 1000.0
    groove_height = float(_value(common["grasp_groove_height_mm"])) / 1000.0
    gripper = raw["gripper"]
    return raw, Variant(
        key=key,
        length=length,
        width=width,
        height=height,
        mass=float(_value(source["empty_mass_kg"])),
        groove_top=groove_top,
        groove_bottom=groove_top - groove_height,
        groove_depth=float(_value(common["grasp_groove_depth_mm"])) / 1000.0,
        hook_length=float(gripper["lower_hook_insert_mm"]) / 1000.0,
        hook_thickness=float(gripper["lower_hook_thickness_mm"]) / 1000.0,
        backing_clearance=float(gripper["groove_backing_clearance_mm"]) / 1000.0,
        closed_contact_span=float(gripper["closed_contact_span_mm"]) / 1000.0,
    )


def _mesh_assets(variant: str) -> tuple[str, list[str], list[str]]:
    collision_dir = GENERATED / variant / "collision"
    visual_dir = GENERATED / variant / "visual"
    if not collision_dir.is_dir() or not visual_dir.is_dir():
        raise SystemExit("请先运行 authoring/generate_models.py")
    declarations: list[str] = []
    collision_names: list[str] = []
    visual_names: list[str] = []
    for prefix, directory, target in (
        ("collision", collision_dir, collision_names),
        ("visual", visual_dir, visual_names),
    ):
        for path in sorted(directory.glob("*.stl")):
            name = f"{prefix}_{path.stem}"
            declarations.append(f'<mesh name="{name}" file="{path.as_posix()}"/>')
            target.append(name)
    return "\n".join(declarations), collision_names, visual_names


def _inertia(mass: float, length: float, width: float, height: float) -> tuple[float, float, float]:
    return (
        mass * (width * width + height * height) / 12.0,
        mass * (length * length + height * height) / 12.0,
        mass * (length * length + width * width) / 12.0,
    )


def _tote_body(v: Variant, collisions: list[str], visuals: list[str]) -> str:
    inertia = _inertia(v.mass, v.length, v.width, v.height)
    collision_geoms = "\n".join(
        f'<geom name="tote_{name}" type="mesh" mesh="{name}" rgba="0 0 0 0" group="3"/>'
        for name in collisions
    )
    visual_geoms = "\n".join(
        f'<geom type="mesh" mesh="{name}" material="tote_blue" contype="0" conaffinity="0" group="1"/>'
        for name in visuals
    )
    return f"""
    <body name="tote" pos="0 0 0.003">
      <freejoint name="tote_free"/>
      <inertial pos="0 0 {v.height / 2}" mass="{v.mass}"
                diaginertia="{inertia[0]} {inertia[1]} {inertia[2]}"/>
      {collision_geoms}
      {visual_geoms}
    </body>
    """


def _gripper_body(side: str, v: Variant) -> str:
    """建立与正式工具相同的C形接触接口，不复制整台Robot视觉外形。"""

    sign = 1.0 if side == "pos" else -1.0
    axis = -sign
    approach_margin = 0.060
    approach_travel = 0.040
    body_x = sign * (v.length / 2 + approach_margin)
    engagement_travel = 0.008
    # approach到位后，完整22mm钩脚进入26mm深凹槽，内端距背板2mm。
    final_body_x = sign * (v.length / 2 + approach_margin - approach_travel)
    hook_center_x_world = sign * (v.length / 2 - v.hook_center_insertion)
    hook_x = hook_center_x_world - final_body_x
    spine_x = sign * (v.length / 2 + 0.002) - final_body_x
    clamp_x = sign * v.length / 2 - final_body_x
    clamp_pad_half_z = 0.0035
    clamp_travel = 0.020
    clamp_open_z = (
        v.closed_contact_span
        + v.hook_thickness / 2
        + clamp_pad_half_z
        + clamp_travel
    )
    return f"""
    <body name="gripper_{side}" pos="{body_x} 0 {v.hook_center_z - engagement_travel}">
      <joint name="approach_{side}" type="slide" axis="{axis} 0 0"
             range="0 {approach_travel}" damping="25"/>
      <joint name="lift_{side}" type="slide" axis="0 0 1"
             range="0 0.148" damping="40"/>
      <inertial pos="0 0 0" mass="1.35" diaginertia="0.004 0.006 0.006"/>
      <geom name="hook_foot_{side}" type="box" pos="{hook_x} 0 0"
            size="{v.hook_length / 2} 0.017 {v.hook_thickness / 2}"
            material="tool_dark"/>
      <geom name="hook_spine_{side}" type="box"
            pos="{spine_x} 0 {v.groove_bottom - (v.hook_center_z - 0.003) + 0.012}"
            size="0.003 0.017 0.012" material="tool_dark"/>
      <body name="clamp_{side}" pos="{clamp_x} 0 {clamp_open_z}">
        <joint name="clamp_{side}" type="slide" axis="0 0 -1"
               range="0 0.035" damping="8"/>
        <inertial pos="0 0 0" mass="0.18" diaginertia="0.0002 0.0002 0.0002"/>
        <geom name="clamp_pad_{side}" type="box" size="0.008 0.030 {clamp_pad_half_z}"
              material="tip_black" friction="0.8 0.01 0.0001"/>
      </body>
    </body>
    """

def _model_xml(v: Variant) -> str:
    assets, collisions, visuals = _mesh_assets(v.key)
    return f"""<mujoco model="r1pro_tote_gripper_prototype">
  <compiler angle="radian" meshdir="."/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>
  <default>
    <geom friction="0.35 0.005 0.0001" solref="0.006 1" solimp="0.9 0.95 0.001"/>
    <joint limited="true"/>
  </default>
  <asset>
    {assets}
    <texture name="checker" type="2d" builtin="checker" rgb1="0.10 0.13 0.16"
             rgb2="0.22 0.27 0.33" width="512" height="512"/>
    <material name="floor" texture="checker" texrepeat="8 8" reflectance="0.1"/>
    <material name="tote_blue" rgba="0.02 0.22 0.82 1"/>
    <material name="tool_dark" rgba="0.16 0.18 0.22 1"/>
    <material name="tip_black" rgba="0.03 0.03 0.04 1"/>
  </asset>
  <worldbody>
    <light pos="0 -1 1.5" dir="0 0.5 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="ground" type="plane" size="2 2 0.05" material="floor"/>
    {_tote_body(v, collisions, visuals)}
    {_gripper_body("pos", v)}
    {_gripper_body("neg", v)}
    <body name="cavity_probe" pos="0 0 {v.height + 0.08}">
      <freejoint/>
      <geom name="probe" type="sphere" size="0.018" mass="0.05" rgba="1 0.55 0.05 1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="approach_pos" joint="approach_pos" kp="1800"
              ctrlrange="0 0.040" forcerange="-180 180"/>
    <position name="approach_neg" joint="approach_neg" kp="1800"
              ctrlrange="0 0.040" forcerange="-180 180"/>
    <position name="lift_pos" joint="lift_pos" kp="2400"
              ctrlrange="0 0.140" forcerange="-160 160"/>
    <position name="lift_neg" joint="lift_neg" kp="2400"
              ctrlrange="0 0.140" forcerange="-160 160"/>
    <position name="clamp_pos" joint="clamp_pos" kp="1600"
              ctrlrange="0 0.035" forcerange="-120 120"/>
    <position name="clamp_neg" joint="clamp_neg" kp="1600"
              ctrlrange="0 0.035" forcerange="-120 120"/>
  </actuator>
</mujoco>"""


def _step(model: mujoco.MjModel, data: mujoco.MjData, count: int) -> None:
    for _ in range(count):
        mujoco.mj_step(model, data)


def _joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return float(data.qpos[model.jnt_qposadr[joint_id]])


def _body_z(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    return float(data.xpos[body_id][2])


def run_verification(variant_key: str, output: Path) -> dict:
    raw, variant = _variant(variant_key)
    model = mujoco.MjModel.from_xml_string(_model_xml(variant))
    data = mujoco.MjData(model)

    # 先让空箱与内部探针落稳。探针落到箱底而不是穿过或停在实心凸包上，
    # 同时证明箱体碰撞是中空的。
    _step(model, data, 1200)
    tote_ground_z = _body_z(model, data, "tote")
    probe_z = _body_z(model, data, "cavity_probe")
    cavity_ok = 0.0 < probe_z < variant.height * 0.45

    # 与Robot Skill保持同一动作顺序：低位水平插入完整钩脚，再上移8mm
    # 贴合凹槽上沿内侧，最后闭合上夹片。不能在最终承载高度水平硬顶箱沿。
    data.ctrl[:] = [0.040, 0.040, 0.0, 0.0, 0.0, 0.0]
    _step(model, data, 700)
    data.ctrl[2:4] = 0.008
    _step(model, data, 350)
    data.ctrl[4:6] = 0.020
    _step(model, data, 550)
    approach = [_joint_qpos(model, data, name) for name in ("approach_pos", "approach_neg")]
    clamp = [_joint_qpos(model, data, name) for name in ("clamp_pos", "clamp_neg")]
    clamp_force = [
        abs(float(data.actuator_force[index])) for index in (4, 5)
    ]

    # 双侧同步抬升。唯一载荷路径是钩脚/上夹片与箱体凹槽、箱沿的真实接触。
    before_lift = _body_z(model, data, "tote")
    data.ctrl[2] = 0.128
    data.ctrl[3] = 0.128
    contact_pairs: set[tuple[str, str]] = set()
    for _ in range(1800):
        mujoco.mj_step(model, data)
        for contact in data.contact:
            first = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            second = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            if first and second:
                contact_pairs.add(tuple(sorted((first, second))))
    after_lift = _body_z(model, data, "tote")
    lifted = after_lift - before_lift > 0.060
    groove_hook_sides = {
        side
        for side in ("pos", "neg")
        if any(
            f"hook_foot_{side}" in pair
            and any("above_groove" in name for name in pair)
            for pair in contact_pairs
        )
    }
    clamp_contact_sides = {
        side
        for side in ("pos", "neg")
        if any(
            f"clamp_pad_{side}" in pair
            and any(
                marker in name
                for name in pair
                for marker in ("rim_", "above_groove")
            )
            for pair in contact_pairs
        )
    }
    bilateral_groove_contact = groove_hook_sides == {"pos", "neg"}
    bilateral_clamp_contact = clamp_contact_sides == {"pos", "neg"}

    # stop/hold 通过把执行器目标锁定在当前关节位置实现，不写箱体自由关节。
    hold_targets = [
        _joint_qpos(model, data, "lift_pos"),
        _joint_qpos(model, data, "lift_neg"),
    ]
    data.ctrl[2:4] = hold_targets
    _step(model, data, 500)
    hold_joint_start = [_joint_qpos(model, data, name) for name in ("lift_pos", "lift_neg")]
    hold_start = _body_z(model, data, "tote")
    _step(model, data, 250)
    hold_end = _body_z(model, data, "tote")
    hold_drift = abs(hold_end - hold_start)
    hold_joint_drift = max(abs(_joint_qpos(model, data, name) - start) for name, start in zip(("lift_pos", "lift_neg"), hold_joint_start))

    # 回落并释放压紧件。Hook 仍在槽内时只验证卸载，不把它等同于完整退钩。
    data.ctrl[2:4] = 0.008
    _step(model, data, 1300)
    data.ctrl[4:6] = 0.0
    _step(model, data, 500)

    renderer = mujoco.Renderer(model, height=720, width=1280)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, variant.height * 0.55]
    camera.distance = 1.35
    camera.azimuth = 135
    camera.elevation = -24
    renderer.update_scene(data, camera=camera)
    output.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(output, renderer.render())
    renderer.close()

    report = {
        "variant": variant.key,
        "no_hidden_attachment": True,
        "gross_payload_limit_kg": _value(raw["design_limits"]["gross_payload_kg"]),
        "tote_ground_z": tote_ground_z,
        "cavity_probe_z": probe_z,
        "cavity_open": cavity_ok,
        "approach_positions_m": approach,
        "clamp_positions_m": clamp,
        "clamp_forces_n": clamp_force,
        "clamp_force_within_peak": max(clamp_force) <= float(_value(raw["design_limits"]["peak_clamp_force_n"])) + 1e-6,
        "lift_delta_m": after_lift - before_lift,
        "lifted_by_contact": lifted,
        "hook_center_insertion_m": variant.hook_center_insertion,
        "bilateral_groove_contact": bilateral_groove_contact,
        "bilateral_clamp_contact": bilateral_clamp_contact,
        "contact_pairs": sorted([list(pair) for pair in contact_pairs]),
        "hold_drift_m": hold_drift,
        "hold_joint_drift_m": hold_joint_drift,
        "hold_stable": hold_joint_drift < 0.002 and hold_drift < 0.006,
        "evidence_image": str(output),
    }
    report_path = output.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not all(
        (
            report["cavity_open"],
            report["clamp_force_within_peak"],
            report["lifted_by_contact"],
            report["bilateral_groove_contact"],
            report["bilateral_clamp_contact"],
            report["hold_stable"],
        )
    ):
        raise SystemExit("MuJoCo 原型验证失败，详见报告")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--variant",
        choices=("tote-600x400x340", "tote-530x410x240"),
        default="tote-600x400x340",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or GENERATED / f"{args.variant}-verification.png"
    run_verification(args.variant, output)


if __name__ == "__main__":
    main()
