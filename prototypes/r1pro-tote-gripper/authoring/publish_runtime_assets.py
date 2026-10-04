#!/usr/bin/env python3
"""把已评审原型发布为正式 MuJoCo 资产。

脚本只做确定性的资产装配：旧 R1 Pro 与旧场景保持不变，新夹具作为独立
Robot 变体发布。原型长度为毫米，STL 已由生成器转换为米；公共坐标约定为
右手系、Z-up、xyzw，MuJoCo XML 边界保持 wxyz。
"""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PROTOTYPE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
GENERATED_ROOT = PROTOTYPE_ROOT / "generated"
PARAMETERS_PATH = PROTOTYPE_ROOT / "engineering-parameters.yaml"
ROBOT_MODEL = "r1_pro_tote_gripper"
ROBOT_HARDWARE_MODEL = "r1_pro_chassis"
SCENE_ID = "palletizing_depalletizing_tote_v1"

# R1 Pro link7 网格的法兰远端平面位于本体坐标 z=-91.5 mm；新夹具腕部
# 适配器的安装面位于夹具坐标 z=+29 mm。因此固定连接原点应为
# -91.5-29=-120.5 mm。旧值 -160.65 mm 是旧夹爪 Mesh 自身的几何偏移，
# 不能继续作为新夹具 Body 的安装位姿，否则法兰与适配器之间会悬空约 40 mm。
# 新夹具生成时已经使用 R1 Pro 法兰坐标方向，固定连接不再额外旋转；任意
# 90° 旋转都会让 C 型下钩横装。
TOTE_TOOL_MOUNT_XYZ = "-0.0295 0 -0.1205"

# 原始 R1 Pro 的左右 RealSense 都沿用旧平行夹爪附近的安装位姿。专用 C 型
# 夹具接入后，该位置会进入腕部适配器和主动上压片的视觉空间。这里保持原
# 光轴，只把两侧相机分别向各自手臂外侧移动 35 mm，并沿法兰轴向近端退让
# 45 mm。左右相机文件实际复用了同一个非对称 Mesh；右侧不能只反向平移，
# 还必须绕 Mesh 自身光轴旋转 180°，否则安装端和外壳朝向会与真机镜像关系
# 相反。这里同时保存 MJCF(wxyz) 和 URDF(xyz RPY) 姿态，防止两套模型漂移。
WRIST_CAMERA_MOUNT_XYZ = {
    "left": "0.02101 0.0378934 -0.110518",
    "right": "0.02101 -0.0321066 -0.110518",
}
WRIST_CAMERA_MOUNT_QUAT = {
    "left": "0.21643 -0.000394204 -0.976296 0.00176584",
    "right": "-0.0017658408 -0.97629644 0.00039420418 0.2164301",
}
WRIST_CAMERA_MOUNT_RPY = {
    "left": "-3.1376 -0.43631 3.1399",
    "right": "3.1376 0.436310481 -0.00169265629",
}


def _require(path: Path) -> Path:
    if not path.exists():
        raise FileNotFoundError(f"缺少已评审的原型输出：{path}")
    return path


def _indent_and_write(root: ET.Element, path: Path) -> None:
    ET.indent(root, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def _copy_tree_files(source: Path, destination: Path, pattern: str) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    # 这些目录只保存本发布器生成的派生Mesh。先清理同类旧文件，避免资产
    # 改名后旧矩形障碍与新凹槽被Scene同时枚举，形成肉眼和碰撞都难以解释的
    # 双重几何；不删除目录中的其他类型文件。
    for stale in destination.glob(pattern):
        stale.unlink()
    for item in sorted(_require(source).glob(pattern)):
        shutil.copy2(item, destination / item.name)


def _load_parameters() -> dict:
    return yaml.safe_load(_require(PARAMETERS_PATH).read_text(encoding="utf-8"))


def _box_inertia(mass: float, size_m: list[float]) -> list[float]:
    x, y, z = size_m
    return [
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    ]


def _publish_tote_model(variant: str, variant_parameters: dict, common: dict) -> None:
    """发布中空箱体，碰撞模型不允许退化为填死凹腔的单一凸包。"""

    runtime_name = variant.replace("-", "_")
    source_root = _require(GENERATED_ROOT / variant)
    mesh_root = REPOSITORY_ROOT / "assets" / "objects" / "meshes" / runtime_name
    _copy_tree_files(source_root / "visual", mesh_root / "visual", "*.stl")
    _copy_tree_files(source_root / "collision", mesh_root / "collision", "*.stl")

    root = ET.Element("mujoco", {"model": runtime_name})
    asset = ET.SubElement(root, "asset")
    material_name = f"{runtime_name}_blue"
    ET.SubElement(asset, "material", {
        "name": material_name,
        "rgba": " ".join(str(value) for value in variant_parameters["color_rgba"]),
        "specular": "0.22",
        "shininess": "0.35",
    })
    visual_meshes: list[str] = []
    collision_meshes: list[str] = []
    for kind, target in (("visual", visual_meshes), ("collision", collision_meshes)):
        for path in sorted((mesh_root / kind).glob("*.stl")):
            mesh_name = f"{runtime_name}_{kind}_{path.stem}"
            target.append(mesh_name)
            ET.SubElement(asset, "mesh", {
                "name": mesh_name,
                "file": f"meshes/{runtime_name}/{kind}/{path.name}",
            })

    worldbody = ET.SubElement(root, "worldbody")
    body = ET.SubElement(worldbody, "body", {"name": runtime_name})
    ET.SubElement(body, "freejoint", {"name": f"{runtime_name}_freejoint"})
    mass = float(variant_parameters["empty_mass_kg"]["value"])
    size_m = [float(value) / 1000.0 for value in variant_parameters["outer_size_mm"]]
    inertia = _box_inertia(mass, size_m)
    ET.SubElement(body, "inertial", {
        "pos": f"0 0 {size_m[2] * 0.48:.6f}",
        "mass": f"{mass:.6f}",
        "diaginertia": " ".join(f"{value:.8f}" for value in inertia),
    })
    for mesh_name in visual_meshes:
        ET.SubElement(body, "geom", {
            "name": mesh_name,
            "type": "mesh",
            "mesh": mesh_name,
            "material": material_name,
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
            "mass": "0",
        })
    friction = "{} {} {}".format(
        common["sliding_friction"]["value"],
        common["torsional_friction"]["value"],
        common["rolling_friction"]["value"],
    )
    for mesh_name in collision_meshes:
        ET.SubElement(body, "geom", {
            "name": mesh_name,
            "type": "mesh",
            "mesh": mesh_name,
            "rgba": "0 0 0 0",
            # 详细薄壁和抓取凹槽只负责Robot/夹具接触。若它们也参与箱体
            # 堆叠，几十个共面凸网格会在同一支撑面重复施加摩擦约束，静止
            # 整垛会持续横向爬行。堆叠支撑由下面与真实底板/箱沿同尺寸的
            # primitive 表达；这仍是普通接触，不固定物体也不修改 Pose。
            "contype": "2",
            "conaffinity": "1",
            "friction": friction,
            # 多层自由箱体由大量薄壁网格共同接触。过硬的接触会把微小的
            # 网格离散误差放大为高频冲击，数秒后导致自由关节 QACC 发散。
            # 这里只降低箱体接触的刚度；夹具钩爪与压片仍保留更硬的独立
            # 参数，因此不会用柔软接触掩盖抓取失败。
            "solref": "0.020 1",
            "solimp": "0.90 0.95 0.001",
            "group": "3",
            "mass": "0",
        })
    support_common = {
        "type": "box",
        "rgba": "0 0 0 0",
        "contype": "4",
        "conaffinity": "12",
        "friction": friction,
        "solref": "0.020 1",
        "solimp": "0.90 0.95 0.001",
        "group": "3",
        "mass": "0",
    }
    half_x, half_y, height = size_m[0] * 0.5, size_m[1] * 0.5, size_m[2]
    # 这些尺寸直接由同一份箱体外形参数和已评审箱沿截面派生，因此大小箱
    # 共用一套生成逻辑；不能把 600 mm 箱的支撑代理复制给另一种箱型。
    support_geometries = (
        ("stack_support_bottom", "0 0 0.0025", f"{half_x:.6f} {half_y:.6f} 0.0025"),
        (
            "stack_support_rim_neg_x",
            f"{-half_x + 0.002:.6f} 0 {height - 0.009:.6f}",
            f"0.009 {half_y - 0.011:.6f} 0.009",
        ),
        (
            "stack_support_rim_pos_x",
            f"{half_x - 0.002:.6f} 0 {height - 0.009:.6f}",
            f"0.009 {half_y - 0.011:.6f} 0.009",
        ),
        (
            "stack_support_rim_neg_y",
            f"0 {-half_y + 0.002:.6f} {height - 0.009:.6f}",
            f"{half_x + 0.007:.6f} 0.009 0.009",
        ),
        (
            "stack_support_rim_pos_y",
            f"0 {half_y - 0.002:.6f} {height - 0.009:.6f}",
            f"{half_x + 0.007:.6f} 0.009 0.009",
        ),
    )
    for suffix, position, size in support_geometries:
        ET.SubElement(body, "geom", {
            **support_common,
            "name": f"{runtime_name}_{suffix}",
            "pos": position,
            "size": size,
        })
    _indent_and_write(root, REPOSITORY_ROOT / "assets" / "objects" / f"{runtime_name}.xml")


def _remove_matching(parent: ET.Element, predicate) -> None:
    for child in list(parent):
        if predicate(child):
            parent.remove(child)


def _add_tool_visual(parent: ET.Element, side: str, mesh: str) -> None:
    ET.SubElement(parent, "geom", {
        "name": f"{side}_tote_tool_visual_{mesh}",
        "type": "mesh",
        "mesh": f"tote_tool_{mesh}",
        "contype": "0",
        "conaffinity": "0",
        "group": "1",
        "mass": "0",
    })



def _add_mjcf_tool(arm_link: ET.Element, side: str) -> None:
    """在腕部添加固定下钩和单自由度主动上压片。

    q=0 是夹紧位置，正方向向上打开。Runtime 的 position 直接表示夹具开度，
    不在物理层求解末端目标或执行抓取策略。
    """

    gripper = _load_parameters()["gripper"]
    preload_m = float(gripper["upper_clamp_preload_travel_mm"]) / 1000.0
    travel_m = float(gripper["upper_clamp_vertical_travel_mm"]) / 1000.0
    joint_upper_m = preload_m + travel_m

    common = _load_parameters()["tote_family"]["common"]
    adapter_x = float(gripper["adapter_plate_mm"][0]) / 1000.0
    connector_length = float(gripper["connector_horizontal_length_mm"]) / 1000.0
    connector_drop = float(gripper["connector_vertical_drop_mm"]) / 1000.0
    frame_x = float(gripper["contact_frame_mm"][0]) / 1000.0
    frame_z = float(gripper["contact_frame_mm"][2]) / 1000.0
    hook_width = float(gripper["lower_hook_width_mm"]) / 1000.0
    hook_drop = float(gripper["lower_hook_drop_mm"]) / 1000.0
    hook_insert = float(gripper["lower_hook_insert_mm"]) / 1000.0
    hook_t = float(gripper["lower_hook_thickness_mm"]) / 1000.0
    groove_height = float(common["grasp_groove_height_mm"]["value"]) / 1000.0

    frame_bottom = -connector_drop - frame_z / 2.0
    hook_bearing_z = frame_bottom - hook_drop + hook_t
    rail_length = groove_height - 2.0 * hook_t
    frame_contact_x = adapter_x / 2.0 + connector_length + frame_x
    rail_x = frame_contact_x - hook_t / 2.0
    hook_spine_length = rail_length * 0.56

    tool = ET.SubElement(arm_link, "body", {
        "name": f"{side}_tote_tool",
        "pos": TOTE_TOOL_MOUNT_XYZ,
    })
    ET.SubElement(tool, "inertial", {
        "pos": "0.030 0 -0.045",
        "mass": "0.82",
        "diaginertia": "0.0014 0.0011 0.0009",
    })
    for mesh in ("wrist_adapter", "angled_connector", "contact_frame"):
        _add_tool_visual(tool, side, mesh)

    hook = ET.SubElement(tool, "body", {"name": f"{side}_tote_hook"})
    ET.SubElement(hook, "inertial", {
        "pos": "0.074 0 -0.094",
        "mass": "0.15",
        "diaginertia": "0.00018 0.00015 0.00006",
    })
    for mesh in ("hook_guide_rail", "fixed_lower_hooks"):
        _add_tool_visual(hook, side, mesh)

    # 下钩碰撞代理由同一份夹具与凹槽尺寸生成，不能继续沿用旧矩形障碍模型
    # 下的56 mm导轨。单一凸包会填死C形钩口，使“进入凹槽”在物理上永远
    # 不可能，因此仍按导轨、钩脊、钩脚和钩角四个凸Box表达。
    def box_pose(position: tuple[float, float, float]) -> str:
        return " ".join(f"{value:.6f}" for value in position)

    def box_size(size: tuple[float, float, float]) -> str:
        return " ".join(f"{value:.6f}" for value in size)

    hook_collision = (
        (
            "guide",
            box_pose((rail_x - 0.0015, 0.0, hook_bearing_z + rail_length / 2.0)),
            box_size((0.0015, 0.006, rail_length / 2.0)),
        ),
        (
            "spine",
            box_pose((rail_x, 0.0, hook_bearing_z + hook_spine_length / 2.0)),
            box_size((hook_t / 2.0, hook_width / 2.0, hook_spine_length / 2.0)),
        ),
        (
            "foot",
            box_pose((rail_x + hook_insert / 2.0, 0.0, hook_bearing_z - hook_t / 2.0)),
            box_size((hook_insert / 2.0, hook_width / 2.0, hook_t / 2.0)),
        ),
        (
            "lip",
            box_pose((rail_x + hook_insert - hook_t / 2.0, 0.0, hook_bearing_z + hook_t / 2.0)),
            box_size((hook_t / 2.0, hook_width / 2.0, hook_t)),
        ),
    )
    for suffix, position, size in hook_collision:
        ET.SubElement(hook, "geom", {
            "name": f"{side}_tote_hook_{suffix}_collision",
            "type": "box",
            "pos": position,
            "size": size,
            "rgba": "0 0 0 0",
            "contype": "1",
            "conaffinity": "15",
            "friction": "0.75 0.01 0.001",
            "solref": "0.004 1",
            "solimp": "0.97 0.995 0.001",
            "group": "3",
            "mass": "0",
        })

    # q=preload_m 是箱沿与下钩名义跨度相等的闭合参考位；继续向 q=0
    # 运动的 4 mm 表达压片/箱沿弹性预压。若没有这段机械行程，控制器
    # 会在刚接触时同时到达关节下限，无法形成可测夹紧力。这里移动的是
    # 夹具活动件，不修改箱体 Pose，也不降低 stable_load 条件。
    clamp = ET.SubElement(tool, "body", {
        "name": f"{side}_tote_clamp", "pos": f"0 0 {-preload_m:.6f}",
    })
    ET.SubElement(clamp, "inertial", {
        "pos": "0.074 0 -0.034",
        "mass": "0.08",
        "diaginertia": "0.00004 0.00003 0.00002",
    })
    ET.SubElement(clamp, "joint", {
        "name": f"{side}_tote_clamp_joint",
        "type": "slide",
        "axis": "0 0 1",
        "range": f"0 {joint_upper_m:.6f}",
        "limited": "true",
        "damping": "18",
        "frictionloss": "2",
        "armature": "0.004",
    })
    for mesh in ("active_upper_clamp", "replaceable_tip"):
        _add_tool_visual(clamp, side, mesh)
    ET.SubElement(clamp, "geom", {
        "name": f"{side}_tote_clamp_pad_collision",
        "type": "box",
        "pos": "0.075 0 -0.036",
        "size": "0.014 0.026 0.002",
        "rgba": "0 0 0 0",
        "contype": "1",
        "conaffinity": "15",
        "friction": "0.85 0.012 0.001",
        "solref": "0.004 1",
        "solimp": "0.98 0.995 0.001",
        "group": "3",
        "mass": "0",
    })

    ET.SubElement(tool, "site", {
        "name": f"{side}_ee_site",
        "pos": "0.083 0 -0.078",
        "size": "0.008",
    })
    ET.SubElement(tool, "site", {
        "name": f"{side}_tote_approach",
        "pos": "0.120 0 -0.078",
        "size": "0.006",
    })
    ET.SubElement(tool, "site", {
        "name": f"{side}_tote_load_frame",
        # load frame 是规划与验证共同使用的真实承载点，必须落在固定下钩脚
        # 中心，而不是沿用腕部外观参考点。否则 SDK 虽能把命名 frame 精确
        # 移到 receiver，实体钩脚仍会在其下方 49 mm，最终只能空行程夹紧。
        "pos": "0.083 0 -0.127",
        "size": "0.006",
    })
    ET.SubElement(hook, "site", {
        "name": f"{side}_tote_hook_tip",
        "pos": "0.091 0 -0.121",
        "size": "0.004",
    })
    ET.SubElement(clamp, "site", {
        "name": f"{side}_tote_clamp_pad",
        "pos": "0.075 0 -0.036",
        "size": "0.004",
    })


def _publish_robot_mjcf() -> None:
    source = _require(
        REPOSITORY_ROOT / "robot" / "r1_pro_chassis" / "config" / "r1_pro_chassis.xml"
    )
    root = ET.parse(source).getroot()
    root.set("model", ROBOT_MODEL)

    # 专用Robot由这份MJCF执行、由同包URDF做IK。历史chassis MJCF把双臂
    # joint4下限写成-100°，而URDF为-120°；规划出的高位姿因此会被Runtime
    # 卡在MJCF限位并持续打满25 Nm。发布时从规划模型复制这两个真实限位，
    # 让同一Robot Package内的规划与执行一致，而不是放宽执行到位容差。
    urdf_source = _require(
        REPOSITORY_ROOT
        / "robot"
        / "r1_pro_chassis"
        / "meshes"
        / "r1_pro_with_gripper.urdf"
    )
    urdf_root = ET.parse(urdf_source).getroot()
    for side in ("left", "right"):
        joint_name = f"{side}_arm_joint4"
        mjcf_joint = root.find(f".//joint[@name='{joint_name}']")
        urdf_limit = urdf_root.find(f".//joint[@name='{joint_name}']/limit")
        if mjcf_joint is None or urdf_limit is None:
            raise ValueError(f"R1 Pro规划或执行模型缺少{joint_name}限位")
        lower = urdf_limit.get("lower")
        upper = urdf_limit.get("upper")
        if lower is None or upper is None:
            raise ValueError(f"R1 Pro URDF的{joint_name}限位不完整")
        mjcf_joint.set("range", f"{lower} {upper}")

    # 原始 chassis 的临时支撑板底面穿入 ground 10 mm。多箱场景中该初始
    # 约束会持续向系统注入能量，最终把自由箱体 QACC 推到非有限值；专用
    # Robot 包发布时把支撑板改为恰好贴地，而不改变公共 chassis 资产。
    support_plate = root.find(".//geom[@name='root_support_plate']")
    if support_plate is None:
        raise ValueError("R1 Pro MJCF 缺少 root_support_plate")
    support_plate.set("pos", "0 0 0")

    # yaw joint 本身没有角度限位，底盘路线也会用连续角度表达跨越 +/-pi 的
    # 最短转向。给执行器设置任何固定 ctrlrange 都只能推迟问题：连续执行多次
    # 转向后仍会撞上边界并永久卡在 99.9%。因此角度目标不设限，仅保留下面的
    # forcerange 约束实际驱动力矩。
    yaw_actuator = root.find(".//position[@name='root_z_rotate_motor']")
    if yaw_actuator is None:
        raise ValueError("R1 Pro MJCF 缺少 root_z_rotate_motor")
    yaw_actuator.set("ctrllimited", "false")
    yaw_actuator.attrib.pop("ctrlrange", None)

    asset = root.find("asset")
    if asset is None:
        raise ValueError("R1 Pro MJCF 缺少 asset 节点")
    for mesh in list(asset.findall("mesh")):
        file_path = mesh.get("file", "")
        mesh_name = mesh.get("name", "")
        if "gripper" in mesh_name:
            asset.remove(mesh)
            continue
        mesh.set("file", f"../../r1_pro_chassis/meshes/{Path(file_path).name}")
    for mesh in (
        "wrist_adapter",
        "angled_connector",
        "contact_frame",
        "hook_guide_rail",
        "fixed_lower_hooks",
        "active_upper_clamp",
        "replaceable_tip",
    ):
        ET.SubElement(asset, "mesh", {
            "name": f"tote_tool_{mesh}",
            "file": f"../meshes/tool/{mesh}.stl",
        })

    for side in ("left", "right"):
        arm_link = root.find(f".//body[@name='{side}_arm_link7']")
        if arm_link is None:
            raise ValueError(f"R1 Pro MJCF 缺少 {side}_arm_link7")
        for geom in arm_link.findall("geom"):
            if geom.get("mesh") == f"{side}_realsense_link":
                geom.set("pos", WRIST_CAMERA_MOUNT_XYZ[side])
                geom.set("quat", WRIST_CAMERA_MOUNT_QUAT[side])
        _remove_matching(
            arm_link,
            lambda element: (
                element.tag == "body"
                and element.get("name", "").startswith(f"{side}_gripper_")
            )
            or (element.tag == "geom" and "gripper" in element.get("mesh", ""))
            or (element.tag == "site" and element.get("name") == f"{side}_ee_site"),
        )
        _add_mjcf_tool(arm_link, side)

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    _remove_matching(
        actuator,
        lambda element: "gripper_finger" in element.get("name", ""),
    )
    for side in ("left", "right"):
        ET.SubElement(actuator, "motor", {
            "name": f"{side}_tote_clamp_joint",
            "joint": f"{side}_tote_clamp_joint",
            "ctrllimited": "true",
            "ctrlrange": "-120 120",
            "gear": "1",
        })

    sensor = root.find("sensor")
    if sensor is None:
        sensor = ET.SubElement(root, "sensor")
    _remove_matching(sensor, lambda element: "gripper" in element.get("name", ""))
    for side in ("left", "right"):
        joint = f"{side}_tote_clamp_joint"
        ET.SubElement(sensor, "jointpos", {
            "name": f"{joint}_position",
            "joint": joint,
        })
        ET.SubElement(sensor, "jointvel", {
            "name": f"{joint}_velocity",
            "joint": joint,
        })
        ET.SubElement(sensor, "actuatorfrc", {
            "name": f"{joint}_force",
            "actuator": joint,
        })

    # Robot 自碰撞、详细箱体几何和托盘仍按正常方式接触，但不与仅用于
    # 箱体堆叠的支撑代理重复碰撞。位掩码只属于该 MuJoCo 资产包，不会
    # 进入 Runtime、SDK 或 Ability 的公共契约。
    for default_geom in root.findall(".//default//geom"):
        if default_geom.get("contype", "1") != "0":
            default_geom.set("conaffinity", "11")
    for collision_geom in root.iter("geom"):
        if collision_geom.get("contype") not in {None, "0"}:
            collision_geom.set("conaffinity", "11")

    contact = root.find("contact")
    if contact is not None:
        _remove_matching(
            contact,
            lambda element: "gripper" in " ".join(element.attrib.values()),
        )
        if not list(contact):
            root.remove(contact)

    destination = (
        REPOSITORY_ROOT / "robot" / ROBOT_MODEL / "config" / f"{ROBOT_MODEL}.xml"
    )
    _indent_and_write(root, destination)



def _urdf_origin(parent: ET.Element, xyz: str) -> None:
    ET.SubElement(parent, "origin", {"xyz": xyz, "rpy": "0 0 0"})


def _urdf_inertial(link: ET.Element, mass: float, inertia: float) -> None:
    inertial = ET.SubElement(link, "inertial")
    _urdf_origin(inertial, "0 0 0")
    ET.SubElement(inertial, "mass", {"value": str(mass)})
    ET.SubElement(inertial, "inertia", {
        "ixx": str(inertia),
        "ixy": "0",
        "ixz": "0",
        "iyy": str(inertia),
        "iyz": "0",
        "izz": str(inertia),
    })


def _urdf_mesh_visual(link: ET.Element, mesh: str) -> None:
    visual = ET.SubElement(link, "visual")
    _urdf_origin(visual, "0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", {
        "filename": f"package://{ROBOT_MODEL}/meshes/tool/{mesh}.stl",
    })
    material = ET.SubElement(visual, "material", {"name": "tote_tool_black"})
    ET.SubElement(material, "color", {"rgba": "0.025 0.03 0.035 1"})


def _urdf_box_collision(link: ET.Element, xyz: str, size: str) -> None:
    collision = ET.SubElement(link, "collision")
    _urdf_origin(collision, xyz)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": size})


def _fixed_joint(
    root: ET.Element,
    name: str,
    parent: str,
    child: str,
    xyz: str,
) -> None:
    joint = ET.SubElement(root, "joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    _urdf_origin(joint, xyz)


def _add_urdf_tool(root: ET.Element, side: str) -> None:
    gripper = _load_parameters()["gripper"]
    preload_m = float(gripper["upper_clamp_preload_travel_mm"]) / 1000.0
    travel_m = float(gripper["upper_clamp_vertical_travel_mm"]) / 1000.0
    joint_upper_m = preload_m + travel_m

    tool_name = f"{side}_tote_tool"
    tool = ET.SubElement(root, "link", {"name": tool_name})
    _urdf_inertial(tool, 0.82, 0.0011)
    for mesh in ("wrist_adapter", "angled_connector", "contact_frame", "hook_guide_rail"):
        _urdf_mesh_visual(tool, mesh)
    _fixed_joint(
        root,
        f"{side}_tote_tool_mount",
        f"{side}_arm_link7",
        tool_name,
        TOTE_TOOL_MOUNT_XYZ,
    )

    hook_name = f"{side}_tote_hook"
    hook = ET.SubElement(root, "link", {"name": hook_name})
    _urdf_inertial(hook, 0.15, 0.00015)
    _urdf_mesh_visual(hook, "fixed_lower_hooks")
    _urdf_box_collision(hook, "0.072 0 -0.11168", "0.006 0.034 0.02464")
    _urdf_box_collision(hook, "0.083 0 -0.127", "0.022 0.034 0.006")
    _urdf_box_collision(hook, "0.091 0 -0.121", "0.006 0.034 0.012")
    _fixed_joint(root, f"{side}_tote_hook_fixed", tool_name, hook_name, "0 0 0")

    clamp_name = f"{side}_tote_clamp"
    clamp = ET.SubElement(root, "link", {"name": clamp_name})
    _urdf_inertial(clamp, 0.08, 0.00003)
    _urdf_mesh_visual(clamp, "active_upper_clamp")
    _urdf_mesh_visual(clamp, "replaceable_tip")
    _urdf_box_collision(clamp, "0.075 0 -0.036", "0.028 0.052 0.004")
    joint = ET.SubElement(
        root,
        "joint",
        {"name": f"{side}_tote_clamp_joint", "type": "prismatic"},
    )
    ET.SubElement(joint, "parent", {"link": tool_name})
    ET.SubElement(joint, "child", {"link": clamp_name})
    _urdf_origin(joint, f"0 0 {-preload_m:.6f}")
    ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
    ET.SubElement(joint, "limit", {
        "lower": "0",
        "upper": f"{joint_upper_m:.6f}",
        "effort": "120",
        "velocity": "0.04",
    })

    frames = {
        # URDF 与 MJCF 必须对同一个 load frame 给出同一物理位置；IK 的
        # 目标因此就是钩脚承载中心，不需要 Ability 再维护资产补偿常量。
        f"{side}_tote_load_frame": "0.083 0 -0.127",
        f"{side}_tote_approach": "0.120 0 -0.078",
        f"{side}_tote_hook_tip": "0.091 0 -0.121",
        f"{side}_tote_clamp_pad": "0.075 0 -0.036",
    }
    for frame_name, xyz in frames.items():
        ET.SubElement(root, "link", {"name": frame_name})
        _fixed_joint(root, f"{frame_name}_fixed", tool_name, frame_name, xyz)


def _publish_robot_urdf() -> None:
    source = _require(
        REPOSITORY_ROOT
        / "robot"
        / "r1_pro_chassis"
        / "meshes"
        / "r1_pro_with_gripper.urdf"
    )
    root = ET.parse(source).getroot()
    root.set("name", ROBOT_MODEL)

    removed_links = {
        f"{side}_{name}"
        for side in ("left", "right")
        for name in ("gripper_link", "gripper_finger_link1", "gripper_finger_link2")
    }
    for child in list(root):
        if child.tag == "link" and child.get("name") in removed_links:
            root.remove(child)
        elif child.tag == "joint":
            parent = child.find("parent")
            target = child.find("child")
            parent_link = parent.get("link", "") if parent is not None else ""
            child_link = target.get("link", "") if target is not None else ""
            if parent_link in removed_links or child_link in removed_links:
                if child_link not in {"left_realsense_link", "right_realsense_link"}:
                    root.remove(child)
                    continue
                side = child_link.split("_", 1)[0]
                parent.set("link", f"{side}_arm_link7")
                origin = child.find("origin")
                if origin is not None:
                    origin.set("xyz", WRIST_CAMERA_MOUNT_XYZ[side])
                    origin.set("rpy", WRIST_CAMERA_MOUNT_RPY[side])

    for side in ("left", "right"):
        camera_joint = root.find(f".//joint[@name='{side}_realsense_joint']")
        if camera_joint is None:
            raise ValueError(f"R1 Pro URDF 缺少 {side}_realsense_joint")
        camera_origin = camera_joint.find("origin")
        if camera_origin is None:
            raise ValueError(f"R1 Pro URDF 缺少 {side}_realsense_joint/origin")
        camera_origin.set("xyz", WRIST_CAMERA_MOUNT_XYZ[side])
        camera_origin.set("rpy", WRIST_CAMERA_MOUNT_RPY[side])

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://r1_pro_chassis/"):
            continue
        if filename:
            mesh.set(
                "filename",
                f"package://r1_pro_chassis/meshes/{Path(filename).name}",
            )

    for side in ("left", "right"):
        _add_urdf_tool(root, side)

    destination = (
        REPOSITORY_ROOT
        / "robot"
        / ROBOT_MODEL
        / "meshes"
        / f"{ROBOT_MODEL}.urdf"
    )
    _indent_and_write(root, destination)


def _write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.strip() + "\n", encoding="utf-8")

def _publish_robot_profile() -> None:
    """写入 Runtime/Robot SDK 共用的唯一名称、行程和力反馈配置。"""

    gripper = _load_parameters()["gripper"]
    preload_m = float(gripper["upper_clamp_preload_travel_mm"]) / 1000.0
    travel_m = float(gripper["upper_clamp_vertical_travel_mm"]) / 1000.0
    joint_upper_m = preload_m + travel_m
    clamp_kp = float(gripper["upper_clamp_position_kp_n_per_m"])
    clamp_kd = float(gripper["upper_clamp_position_kd_ns_per_m"])
    minimum_hook_force_n = float(gripper["minimum_stable_hook_force_n"])
    minimum_clamp_force_n = float(gripper["minimum_stable_clamp_force_n"])

    profile = {
        "schema_version": 1,
        # Robot 资产目录允许按末端工具形成变体，但对外型号必须保持为
        # R1 Pro 硬件型号。Runtime 用资产目录加载 MJCF，再用该字段向
        # Framework/Robot SDK 报告稳定型号，二者不能再被混为一个名称。
        "model": ROBOT_HARDWARE_MODEL,
        "kind": "mobile_manipulator",
        "coordinate_frame": "world",
        "sdk_package": "semantic-robot-sdk-r1pro",
        "backend": "mujoco",
        "backend_profile": "r1pro-tote-mujoco-v1",
        "root_body": "root",
        "source_quaternion_order": "wxyz",
        "kinematics": {
            "root_frame": "base_link",
            "urdf": f"meshes/{ROBOT_MODEL}.urdf",
            "package_root": "..",
            "end_effector_frames": {
                "left": "left_tote_load_frame",
                "right": "right_tote_load_frame",
            },
            "disabled_collision_pairs": [
                ["left_tote_hook", "left_tote_clamp"],
                ["right_tote_hook", "right_tote_clamp"],
            ],
        },
        "base": {
            "x_joint": "root_x_translate",
            "y_joint": "root_y_translate",
            "yaw_joint": "root_z_rotate",
        },
        "joint_groups": {
            "torso": [f"torso_joint{index}" for index in range(1, 5)],
            "left_arm": [f"left_arm_joint{index}" for index in range(1, 8)],
            "right_arm": [f"right_arm_joint{index}" for index in range(1, 8)],
        },
        # 躯干长期承受双臂和专用夹具的重力矩，需要比手臂更高的稳态刚度；
        # 手臂保持较低增益以避免进入公开力矩限幅后的往复振荡。该标定属于
        # Robot 资产/Profile，不应散落在 Runtime、SDK 或抓取 Skill 中。
        # 两侧夹具需要克服滑轨的静摩擦后才能到达接触位置，因此单独使用较高
        # 的位置增益；这仍是执行机构标定，不改变 Runtime 的通用控制语义。
        "joint_control": {
            "torso": {"kp": 2400.0, "kd": 80.0},
            # 手臂需要在公开力矩限制内平稳跟随双侧轨迹。增益过高会让伸臂
            # 姿态持续撞击限幅并在目标附近振荡，反而无法满足到位条件；
            # 300/8 按当前关节质量矩阵的临界阻尼量级标定，并由代表性双臂
            # 接近和带载插入测试共同验证。增益过高会在力矩限幅两端切换，
            # 不能靠放宽完成容差掩盖。真机仍由 vendor Controller 负责。
            "left_arm": {"kp": 300.0, "kd": 8.0},
            "right_arm": {"kp": 300.0, "kd": 8.0},
            # 4 mm机械预压行程使接触发生在关节目标之前。下钩扣住凹槽
            # 上沿后会让负载坐标系产生毫米级沉降；滑轨还存在静摩擦。
            # 夹具增益和最低稳定力由工程参数一起标定，使接触持续越过门槛，
            # 而不是只在瞬态达标；它仍受 60 N 命令力和 120 N 过载上限约束。
            # stable_load 仍只由 Runtime 的真实接触力、滑移和连续帧判定。
            # 这属于资产执行器标定，不改变 SDK 或 Skill 的通用控制语义。
            "left": {"kp": clamp_kp, "kd": clamp_kd},
            "right": {"kp": clamp_kp, "kd": clamp_kd},
        },
        "end_effectors": {"left": "left_tote_load_frame", "right": "right_tote_load_frame"},
        "grippers": {
            "left": ["left_tote_clamp_joint"],
            "right": ["right_tote_clamp_joint"],
        },
        "gripper_tools": {
            side: {
                "kind": "tote_clamp",
                "tool_ref": f"component://tool/{side}",
                "frame": f"{side}_tote_load_frame",
                "joint": f"{side}_tote_clamp_joint",
                "software_range_m": [0.0, joint_upper_m],
                "normal_force_n": 60.0,
                "peak_force_n": 120.0,
                "control_force_scale_n_per_unit": 1.0,
                "hook_bodies": [f"{side}_tote_hook"],
                "clamp_bodies": [f"{side}_tote_clamp"],
                "minimum_clamp_force_n": minimum_clamp_force_n,
                "minimum_hook_force_n": minimum_hook_force_n,
                "maximum_slip_speed_m_s": 0.025,
            }
            for side in ("left", "right")
        },
        "sensors": {"camera": "camera"},
    }
    destination = REPOSITORY_ROOT / "robot" / ROBOT_MODEL / "config"
    _write_text(
        destination / "semantic_robot_profile.yaml",
        yaml.safe_dump(profile, allow_unicode=True, sort_keys=False),
    )
    _write_text(
        REPOSITORY_ROOT / "robot" / ROBOT_MODEL / "README.md",
        """
# R1 Pro 周转箱夹具变体

该目录是独立 Robot 资产，不会替换 r1_pro_chassis。左右腕各安装一个固定尺寸
的 C 型周转箱夹具；同一套夹具通过 0–35 mm 主动压片行程适配两种箱体。

- 固定下钩进入箱体短边26 mm抓取凹槽并扣住槽沿。
- 主动上压片只执行低层开合与力限制。
- q=0 表示夹紧，正方向向上打开。
- Runtime 只报告位置、力、Hook/Clamp 接触、滑移与稳定承载。
- 双臂对位、IK、抓取顺序和失败恢复由后续 Robot SDK/Skill 实现。

机械参数仍需在真机制造前按腕部法兰、负载和箱体实测数据标定。
""",
    )


def _scene_base_xml() -> ET.Element:
    root = ET.Element("mujoco", {"model": SCENE_ID})
    ET.SubElement(root, "option", {
        "iterations": "100",
        "timestep": "0.001",
        "solver": "Newton",
        "integrator": "implicitfast",
        "gravity": "0 0 -9.81",
    })
    ET.SubElement(root, "compiler", {
        "angle": "radian",
        "eulerseq": "zyx",
        "autolimits": "true",
    })
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {
        "type": "skybox",
        "builtin": "gradient",
        "rgb1": "0.3 0.5 0.7",
        "rgb2": "0 0 0",
        "width": "512",
        "height": "512",
    })
    ET.SubElement(asset, "texture", {
        "name": "texplane",
        "type": "2d",
        "builtin": "checker",
        "rgb1": ".2 .3 .4",
        "rgb2": ".1 .15 .2",
        "width": "512",
        "height": "512",
        "mark": "cross",
        "markrgb": ".8 .8 .8",
    })
    ET.SubElement(asset, "material", {
        "name": "matplane",
        "reflectance": "0",
        "texture": "texplane",
        "texrepeat": "1 1",
        "texuniform": "true",
    })
    ET.SubElement(asset, "material", {
        "name": "visualgeom",
        "rgba": "0.5 0.9 0.2 1",
    })
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", {
        "directional": "true",
        "diffuse": ".45 .45 .45",
        "specular": ".12 .12 .12",
        "pos": "0 0 5",
        "dir": "0 0 -1",
    })
    ET.SubElement(worldbody, "light", {
        "directional": "true",
        "diffuse": ".6 .6 .6",
        "specular": ".2 .2 .2",
        "pos": "2 -2 4",
        "dir": "-.3 .3 -1",
    })
    ET.SubElement(worldbody, "geom", {
        "name": "ground",
        "type": "plane",
        "pos": "0 0 0",
        "size": "100 100 .001",
        "material": "matplane",
        "condim": "3",
        "conaffinity": "15",
    })
    return root


def _asset(
    name: str,
    model: str,
    position: list[float],
    *,
    rotation: list[float] | None = None,
    size: list[float] | None = None,
    rgba: list[float] | None = None,
    static: bool = False,
    interactive: bool = True,
    category: str | None = None,
    properties: dict | None = None,
    pose_reference: str | None = None,
    collision: dict | None = None,
) -> dict:
    value = {
        "asset_name": name,
        "asset_model": model,
        "position": position,
        "rotation": rotation or [0, 0, 0],
        "static": static,
        "interactive": interactive,
    }
    if category is not None:
        value["category"] = category
    if properties is not None:
        value["properties"] = properties
    if pose_reference is not None:
        value["pose_reference"] = pose_reference
    if size is not None:
        value["size"] = list(size)
    if rgba is not None:
        value["rgba"] = rgba
    if collision is not None:
        value["collision"] = dict(collision)
    return value


def _layout(name: str, totes: list[dict], *, accepted_models: list[str]) -> dict:
    assets = [
        _asset(
            "pallet-a", "box", [0.0, 1.5, 0.075],
            size=[1.2, 1.0, 0.15], rgba=[0.0, 0.0, 0.3, 1.0],
            static=True, interactive=False, category="pallet",
            collision={"contype": 8, "conaffinity": 5},
        ),
        _asset(
            "pallet-b", "box", [1.5, 1.5, 0.075],
            size=[1.2, 1.0, 0.15], rgba=[0.0, 0.0, 0.3, 1.0],
            static=True, interactive=False, category="pallet",
            collision={"contype": 8, "conaffinity": 5},
        ),
        # Scene只声明托盘与堆叠列，不能把特定Robot的approach固化到Runtime。
        # 基座工作位姿由Agent根据当前Robot、对象几何和通行空间推导。
        *[
            _asset(
                f"pallet-b-slot-r{row}-c{column}", "target", [x, y, 0.16],
                size=[0.56, 0.36, 0.02], rgba=[0.25, 0.75, 0.55, 0.28],
                static=True, interactive=False, category="region",
                properties={
                    "row": row,
                    "column": column,
                    "support_surface_ref": "pallet-b",
                    "support_z_m": 0.15,
                    "accepted_models": list(accepted_models),
                    "max_layers": 3,
                },
            )
            for row, y in enumerate((1.29, 1.71), start=1)
            for column, x in enumerate((1.19, 1.81), start=1)
        ],
        *totes,
    ]
    return {"layout_name": name, "assets": assets}


def _publish_scene() -> None:
    scene_root = REPOSITORY_ROOT / "scene" / SCENE_ID
    _indent_and_write(_scene_base_xml(), scene_root / f"{SCENE_ID}.xml")
    scene_info = {
        "basic_info": {
            "scene_id": SCENE_ID,
            "scene_name": "R1 Pro 真实周转箱拆码垛场景",
            "base_model_path": f"./{SCENE_ID}.xml",
        },
        "robots": [{
            "robot_name": ROBOT_MODEL,
            "robot_id": 1,
            "position": [0, 0, 0.01],
            "rotation": [0, 0, 0],
            "sensor_name": "camera",
        }],
        "sensors": [{
            "sensor_name": "camera",
            "sensor_type": "camera",
            "width": 640,
            "height": 480,
            "camera_intrinsics": {
                "sensorsize": [0.004, 0.003],
                "focalpixel": [385.57, 385.01],
                "principalpixel": [327.58, 238.69],
            },
            "fps": 15,
            "flip_horizontal": False,
            "flip_vertical": False,
        }],
        "simulation_params": {"timestep": 0.001, "gravity": [0, 0, -9.81]},
        "viewer": {
            "fixed_camera_name": "",
            "azimuth": 120,
            "elevation": -25,
            "distance": 6.0,
            "lookat": [0.0, 0.0, 0.8],
        },
    }
    _write_text(
        scene_root / "scene_info.yaml",
        yaml.safe_dump(scene_info, allow_unicode=True, sort_keys=False),
    )
    # layout001/002/003 是面向产品验收的业务布局，不能因为物理 Smoke
    # 暂时只验证单箱就删掉完整垛。独立的 layout_smoke 只承担“模型加载、
    # 动态箱体稳定、低层真实接触”的快速回归，二者的通过条件必须分开。
    def stacked_totes(
        prefix: str,
        model: str,
        *,
        pallet_x: float,
        pallet_y: float,
        column_x: float,
        row_offset: float,
        bottom_z: float,
        layer_pitch: float,
        extent: list[float],
    ) -> list[dict]:
        return [
            _asset(
                f"{prefix}-l{layer}-r{row}-c{column}", model,
                [x, y, bottom_z + (layer - 1) * layer_pitch],
                size=extent, category="tote", pose_reference="bottom_center",
            )
            for layer in range(1, 4)
            for row, y in enumerate(
                (pallet_y - row_offset, pallet_y + row_offset), start=1
            )
            for column, x in enumerate(
                (pallet_x - column_x, pallet_x + column_x), start=1
            )
        ]

    # 中心距计入 7 mm 箱沿外伸和少量初始接触余量。标称 600×400 或
    # 530×410 的箱体如果仍按标称尺寸零间隙排布，同行箱沿会在 t=0 产生
    # 14 mm 穿透，随后自由箱体 QACC 发散；这属于场景初态错误，不是控制器。
    large_stack = stacked_totes(
        "tote-large", "tote_600x400x340", pallet_x=0.0, pallet_y=1.5,
        column_x=0.31, row_offset=0.21, bottom_z=0.152, layer_pitch=0.34,
        extent=[0.60, 0.40, 0.34],
    )
    small_stack_a = stacked_totes(
        "tote-small", "tote_530x410x240", pallet_x=0.0, pallet_y=1.5,
        column_x=0.275, row_offset=0.215, bottom_z=0.152, layer_pitch=0.24,
        extent=[0.53, 0.41, 0.24],
    )
    small_stack_b = stacked_totes(
        "tote-small", "tote_530x410x240", pallet_x=1.5, pallet_y=1.5,
        column_x=0.275, row_offset=0.215, bottom_z=0.152, layer_pitch=0.24,
        extent=[0.53, 0.41, 0.24],
    )
    smoke_tote = _asset(
        "tote-large-smoke", "tote_600x400x340", [0.0, 1.5, 0.152],
        size=[0.60, 0.40, 0.34], category="tote", pose_reference="bottom_center",
    )
    layouts = {
        "layout001": _layout(
            "layout001", large_stack, accepted_models=["tote_600x400x340"]
        ),
        "layout002": _layout(
            "layout002", small_stack_a, accepted_models=["tote_530x410x240"]
        ),
        "layout003": _layout(
            "layout003", [*large_stack, *small_stack_b],
            accepted_models=["tote_530x410x240"],
        ),
        "layout_smoke": _layout(
            "layout_smoke", [smoke_tote], accepted_models=["tote_600x400x340"]
        ),
    }
    for layout_name, layout in layouts.items():
        _write_text(
            scene_root / f"{layout_name}.yaml",
            yaml.safe_dump(layout, allow_unicode=True, sort_keys=False),
        )
    manifest = {
        "schema_version": 1,
        "scene_key": SCENE_ID,
        "catalog": {
            "scene_id": "native-r1pro-tote-depalletizing",
            "name": "R1 Pro 周转箱拆码垛",
            "description": "R1 Pro 双臂专用夹具与真实周转箱拆码垛场景",
            "version": "0.5.0",
            "source": "mujoco-asset",
            "tags": ["native-mujoco", "depalletizing", "r1-pro", "tote"],
            "capabilities": ["viewer", "rgb", "depth", "contact", "holding"],
            "authoring": {"mode": "fixed_layout"},
        },
        "layouts": {
            layout_name: {
                "name": layout_name,
                "expected_objects": [
                    item["asset_name"] for item in layout["assets"]
                ],
            }
            for layout_name, layout in layouts.items()
        },
        "robots": [{
            "model": ROBOT_HARDWARE_MODEL,
            "profile": f"../../robot/{ROBOT_MODEL}/config/semantic_robot_profile.yaml",
        }],
    }
    _write_text(
        scene_root / "asset-manifest.yaml",
        yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False),
    )
    _write_text(
        scene_root / "README.md",
        """
# R1 Pro 周转箱拆码垛场景

该场景与原有 palletizing_depalletizing_001 并存，使用独立的
r1_pro_tote_gripper Robot 资产和两种参数化中空周转箱。

- layout001：托盘 A 上 2×2×3 个 600×400×340 mm 周转箱；保留
  tote-large-l3-r1-c1/c2 等验收语义引用。
- layout002：托盘 A 上 2×2×3 个 530×410×240 mm 周转箱。
- layout003：两种规格分别位于两个托盘，用于兼容性与放置回归。
- layout_smoke：单个大箱的物理稳定性快速回归，不替代完整垛验收。
- 箱体碰撞由底、壁、箱沿、加强筋和短边抓取凹槽组成，不使用实心方块。
- 夹具抓取必须依靠固定下钩与主动上压片的真实接触，不允许隐藏附着。

v0.5.0 的物理验收只允许通过 Runtime 低层轨迹与夹具命令完成接近、接触、
稳定持有、抬升和释放；禁止 weld、teleport 或直接改写箱体 pose。IK、导航和
抓取候选仍由 R1 Pro Robot SDK/Ability 实现，不进入 Runtime。

此前完整堆垛因相邻箱沿和 Robot 支撑板的初始穿透出现 QACC 发散；当前发布参数
已消除初始穿透，并由 Runtime 对 layout001/002/003 执行有限值回归。后续若回归，
必须报告为物理参数阻塞，不允许自动退化到 layout_smoke 或静默减少箱体数量。
""",
    )


def main() -> None:
    parameters = _load_parameters()
    common = parameters["tote_family"]["common"]
    for variant, variant_parameters in parameters["tote_family"]["variants"].items():
        _publish_tote_model(variant, variant_parameters, common)

    tool_mesh_root = REPOSITORY_ROOT / "robot" / ROBOT_MODEL / "meshes" / "tool"
    _copy_tree_files(GENERATED_ROOT / "gripper-visual", tool_mesh_root, "*.stl")
    _publish_robot_mjcf()
    _publish_robot_urdf()
    _publish_robot_profile()
    _publish_scene()
    print(f"已发布 Robot={ROBOT_MODEL}、Scene={SCENE_ID} 与两种周转箱资产")


if __name__ == "__main__":
    main()
