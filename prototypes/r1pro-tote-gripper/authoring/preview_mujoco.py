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

"""用 MuJoCo 官方 Viewer 快速评审两种周转箱和 R1 Pro 专用夹具外形。

这是独立视觉评审台架，不接入正式场景，也不声明夹具机构已经完成制造设计。
生成的 STL 使用米制；箱体短边凹槽和完整夹具均直接从 generated/ 加载。
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import imageio.v3 as iio
import mujoco
import mujoco.viewer
import yaml

ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / "generated"
TOTE_VARIANTS = (
    ("tote-600x400x340", "0 0 0", "tote_blue"),
    ("tote-530x410x240", "0 0.72 0", "tote_blue_light"),
)
GRIPPER_PARTS = (
    ("wrist_adapter", "tool_housing"),
    ("angled_connector", "tool_housing"),
    ("hook_guide_rail", "tool_guide"),
    ("contact_frame", "tool_frame"),
    ("fixed_lower_hooks", "tool_hook"),
    ("active_upper_clamp", "tool_clamp"),
    ("replaceable_tip", "tool_tip"),
)


def _require_generated() -> None:
    required = [GENERATED / variant / "visual" for variant, _, _ in TOTE_VARIANTS]
    required.append(GENERATED / "gripper-visual")
    missing = [path for path in required if not path.is_dir()]
    if missing:
        paths = "\n".join(f"- {path}" for path in missing)
        raise SystemExit("缺少生成模型，请先运行 authoring/generate_models.py：\n" + paths)


def _mesh_declarations() -> str:
    declarations: list[str] = []
    for variant, _, _ in TOTE_VARIANTS:
        for path in sorted((GENERATED / variant / "visual").glob("*.stl")):
            declarations.append(f'<mesh name="{variant}_{path.stem}" file="{path.as_posix()}"/>')
    for part, _ in GRIPPER_PARTS:
        path = GENERATED / "gripper-visual" / f"{part}.stl"
        declarations.append(f'<mesh name="gripper_{part}" file="{path.as_posix()}"/>')
    return "\n".join(declarations)


def _tote_body(variant: str, position: str, material: str) -> str:
    geoms = "\n".join(
        f'<geom type="mesh" mesh="{variant}_{path.stem}" '
        f'material="{material}" '
        'contype="0" conaffinity="0"/>'
        for path in sorted((GENERATED / variant / "visual").glob("*.stl"))
    )
    return f"""
    <body name="{variant}" pos="{position}">
      {geoms}
    </body>
    """


def _gripper_body(name: str, position: str, quaternion_wxyz: str) -> str:
    geoms = "\n".join(
        f'<geom type="mesh" mesh="gripper_{part}" material="{material}" contype="0" conaffinity="0"/>'
        for part, material in GRIPPER_PARTS
    )
    return f"""
    <body name="{name}" pos="{position}" quat="{quaternion_wxyz}">
      {geoms}
    </body>
    """


def _review_gripper_poses() -> tuple[str, str]:
    """按共同承力接口放置同一固定夹具，不针对箱型缩放模型。"""
    raw = yaml.safe_load(
        (ROOT / "engineering-parameters.yaml").read_text(encoding="utf-8")
    )
    common = raw["tote_family"]["common"]
    tote = raw["tote_family"]["variants"]["tote-600x400x340"]
    g = raw["gripper"]
    length, _, height = tote["outer_size_mm"]
    groove_top = height - common["grasp_groove_top_offset_mm"]["value"]
    # 下钩承载面扣住箱体凹槽上沿内侧；这里对齐的是钩脚承载面，不是腕部
    # 原点，也不是箱体内部任意构件的中心。
    hook_world_z = groove_top
    hook_local_z = -g["connector_vertical_drop_mm"] - g["contact_frame_mm"][2] / 2 - g["lower_hook_drop_mm"] + g["lower_hook_thickness_mm"]
    origin_z = (hook_world_z - hook_local_z) / 1000.0
    frame_outer = g["adapter_plate_mm"][0] / 2 + g["connector_horizontal_length_mm"] + g["contact_frame_mm"][0]
    hook_center_insertion = (
        common["grasp_groove_depth_mm"]["value"]
        - g["lower_hook_insert_mm"] / 2
        - g["groove_backing_clearance_mm"]
    )
    # 评审位姿必须把完整钩脚放进槽内：钩脚外端越过槽口，内端与背板
    # 保留装配间隙。只把接触框贴到箱壁会让钩脚中心仍停在槽口附近。
    origin_x = (
        length / 2 + frame_outer - hook_center_insertion
    ) / 1000.0
    return f"{-origin_x:.6f} 0 {origin_z:.6f}", f"{origin_x:.6f} 0 {origin_z:.6f}"


def _model_xml() -> str:
    # 夹具 STEP 的局部 +X 从腕部指向承力钩。左右夹具分别使用原方向和
    # 绕 Z 轴 180° 的安装；Z 偏移使左右钩脚分别扣入箱体短边凹槽。
    left_pose, right_pose = _review_gripper_poses()
    return f"""<mujoco model="r1pro_tote_gripper_visual_review">
  <compiler angle="radian" meshdir="."/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.75 0.75 0.75" specular="0.25 0.25 0.25"/>
  </visual>
  <asset>
    {_mesh_declarations()}
    <texture name="floor_checker" type="2d" builtin="checker" rgb1="0.08 0.11 0.15"
             rgb2="0.20 0.25 0.31" width="512" height="512"/>
    <material name="floor" texture="floor_checker" texrepeat="10 10" reflectance="0.08"/>
    <material name="tote_blue" rgba="0.02 0.22 0.82 1"/>
    <material name="tote_blue_light" rgba="0.04 0.34 0.95 1"/>
    <material name="tool_adapter" rgba="0.55 0.58 0.62 1"/>
    <material name="tool_housing" rgba="0.16 0.18 0.22 1"/>
    <material name="tool_frame" rgba="0.10 0.11 0.13 1"/>
    <material name="tool_hook" rgba="0.07 0.08 0.095 1"/>
    <material name="tool_guide" rgba="0.62 0.65 0.69 1"/>
    <material name="tool_clamp" rgba="0.08 0.09 0.11 1"/>
    <material name="tool_tip" rgba="0.02 0.02 0.025 1"/>
  </asset>
  <worldbody>
    <light pos="-0.8 -1.0 1.8" dir="0.35 0.35 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="1.0 0.8 1.2" dir="-0.5 -0.3 -1" diffuse="0.45 0.45 0.45"/>
    <geom name="ground" type="plane" size="2 2 0.05" material="floor"/>
    {_tote_body(*TOTE_VARIANTS[0])}
    {_tote_body(*TOTE_VARIANTS[1])}
    {_gripper_body("left_gripper", left_pose, "1 0 0 0")}
    {_gripper_body("right_gripper", right_pose, "0 0 0 1")}
    {_gripper_body("review_gripper", "0 -0.80 0.32", "1 0 0 0")}
  </worldbody>
</mujoco>"""


def _render_review_views(model: mujoco.MjModel, data: mujoco.MjData, output_dir: Path) -> None:
    """导出整体、短边接口和夹持关系的固定相机证据。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    views = (
        ("overall-front", (0.0, 0.0, 0.18), 1.35, 135.0, -18.0),
        ("overall-opposite", (0.0, 0.0, 0.18), 1.35, 315.0, -18.0),
        ("contact-a-front", (-0.30, 0.0, 0.27), 0.52, 0.0, -12.0),
        ("contact-a-quarter", (-0.30, 0.0, 0.27), 0.52, 45.0, -12.0),
        ("contact-a-side", (-0.30, 0.0, 0.27), 0.52, 90.0, -12.0),
        ("contact-a-back", (-0.30, 0.0, 0.27), 0.52, 180.0, -12.0),
        ("contact-a-top", (-0.30, 0.0, 0.30), 0.46, 45.0, -42.0),
        ("contact-b-front", (0.30, 0.0, 0.27), 0.52, 180.0, -12.0),
        ("contact-b-quarter", (0.30, 0.0, 0.27), 0.52, 225.0, -12.0),
        ("contact-b-side", (0.30, 0.0, 0.27), 0.52, 270.0, -12.0),
        ("contact-b-top", (0.30, 0.0, 0.30), 0.46, 225.0, -42.0),
        ("gripper-front", (0.06, -0.80, 0.25), 0.32, 0.0, -3.0),
        ("gripper-quarter", (0.06, -0.80, 0.25), 0.32, 42.0, -7.0),
        ("gripper-side", (0.06, -0.80, 0.25), 0.32, 90.0, -3.0),
    )
    renderer = mujoco.Renderer(model, height=720, width=1280)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    for name, lookat, distance, azimuth, elevation in views:
        camera.lookat[:] = lookat
        camera.distance = distance
        camera.azimuth = azimuth
        camera.elevation = elevation
        renderer.update_scene(data, camera=camera)
        iio.imwrite(output_dir / f"{name}.png", renderer.render())
    renderer.close()
    print(f"已导出模型评审图：{output_dir}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--screenshots", type=Path)
    args = parser.parse_args()
    _require_generated()
    model = mujoco.MjModel.from_xml_string(_model_xml())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if args.screenshots:
        _render_review_views(model, data, args.screenshots)
        return

    print("MuJoCo 模型评审窗口已打开：")
    print("- 左键拖动：旋转")
    print("- Shift + 右键拖动：平移")
    print("- 滚轮：缩放")
    print("- 大箱两侧是左右专用夹具；后方是第二种箱体")

    with mujoco.viewer.launch_passive(model, data) as handle:
        handle.cam.lookat[:] = [0.0, 0.28, 0.18]
        handle.cam.distance = 1.75
        handle.cam.azimuth = 135
        handle.cam.elevation = -24
        while handle.is_running():
            handle.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
