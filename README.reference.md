> Historical technical reference / 历史技术参考。For current build and usage instructions, see [English](README.md) / [中文](README.zh-CN.md). Version-specific examples below are not a current release manifest.

# MuJoCo Assets

本仓库保存 Semantic 仿真平台使用的版本化场景、Robot 模型、Mesh、材质和资产目录。代码仓与资产仓保持分离；`plugin-mujoco` 通过资产根目录加载内容，不复制资产，也不依赖宿主机绝对路径。

## Git LFS

Mesh、图片、GLB 和其他大文件由 Git LFS 管理。首次克隆前安装 Git LFS：

```bash
git lfs install
git clone https://github.com/insightos-community/mujoco-asset.git
cd mujoco-asset
git lfs pull
```

提交前必须确认新增的大文件已成为 LFS 对象：

```bash
git lfs status
git lfs ls-files
git lfs fsck
```

禁止直接提交构建目录、缓存、虚拟环境、Runtime 日志或来源不明的临时二进制。

## 目录

```text
assets/                    通用对象资产
robot/                     Robot 模型、Mesh 与语义映射
scene/                     Scene Package、Layout 与 authoring 模板
asset-catalog.v1.json      Framework 使用的版本化资产目录
```

拆码垛场景位于 `scene/palletizing_depalletizing_001`，包含三套官方 Layout、authoring 模板和资产清单。Franka Panda 的版本化运动学模型包位于 `robot/franka_panda/model_bundle`。

## 使用

开发模式下，将本仓库根目录传给 Runtime：

```bash
# 在 mujoco-runtime 源码目录、已准备的环境中运行：
MUJOCO_ASSET_ROOT=/absolute/path/to/mujoco-asset uv run plugin-mujoco
```

正式环境由 RuntimeInstallation 的 `content_refs` 登记资产位置，不要求用户长期设置环境变量。

## 资产治理

- 每个可发布资产必须具有稳定 ID、来源、许可和分发结论。
- 坐标使用右手系，长度单位为米，角度单位为弧度，公共四元数顺序为 `xyzw`。
- 场景引用相对路径，禁止写入开发主机路径。
- 发布版本不可覆盖；变更通过新版本或新 Layout 交付。
- `source: unknown`、`license: pending` 或 `distribution_status: internal-only` 的资产只能用于内部开发与验收，不能作为公开制品分发。

## 提交检查

```bash
git diff --check
git lfs fsck
python -m json.tool asset-catalog.v1.json >/dev/null
```

XML、URDF、JSON、YAML 和场景 Layout 还应由 `plugin-mujoco` 的 native 集成测试完成实际加载验证。
