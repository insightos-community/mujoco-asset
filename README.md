# MuJoCo Assets

本仓库保存 Semantic 仿真平台使用的版本化场景、Robot 模型、Mesh、材质和资产目录。代码仓与资产仓保持分离；`plugin-mujoco` 通过资产根目录加载内容，不复制资产，也不依赖宿主机绝对路径。

## Git LFS

Mesh、图片、GLB 和其他大文件由 Git LFS 管理。首次克隆前安装 Git LFS：

```bash
git lfs install
git clone <mujoco_asset 仓库地址>
cd mujoco_asset
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

## Scene Package 扩展

每个公共 Native 场景是 `scene/<scene_key>/` 下的独立 Scene Package：

- `asset-manifest.yaml` 是目录索引的唯一公共清单，声明稳定 ID、版本、Layout、能力和 authoring 约束；
- `scene_info.yaml` 只保存 Runtime 加载参数、Robot、Sensor 和物理设置；
- `layout*.yaml` 保存官方只读 Layout；
- `authoring/` 保存可派生 Layout 的模板和兼容素材集合。

新增 Scene Package 不需要修改 Runtime 源码。安装或升级内容后重新生成冻结目录：

```bash
mujoco-runtime catalog index \
  --profile native-mujoco \
  --content-root /absolute/path/to/mujoco_asset \
  --output /installation/content/catalog.yaml
```

Runtime 启动后只读取该冻结目录，不扫描资产仓。旧 `scene/scene_list.json` 已删除，避免与 `asset-manifest.yaml`、`scene_info.yaml` 和素材目录形成重复事实来源。

## 使用

开发模式下，将本仓库根目录传给 Runtime：

```bash
MUJOCO_ASSET_ROOT=/absolute/path/to/mujoco_asset \
MUJOCO_SCENE_CATALOG=/absolute/path/to/catalog.yaml \
uv run mujoco-runtime
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
