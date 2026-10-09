# MuJoCo Assets

[English](README.md) | [简体中文](README.zh-CN.md)

> 🌐 Scene, robot, and object assets used by the Semantic MuJoCo Runtime. This repository is not an executable program, nor a Python environment.
> The approved legacy R1 Pro models are distributed via Git LFS. See [model downloads and provenance](EXTERNAL_MODELS.md). Third-party model rights are not covered by this repository's Apache-2.0 license.

This repository holds the versioned scenes, Robot models, meshes, materials, and asset catalog used by the Semantic simulation platform. The code repo and the asset repo stay separate; `plugin-mujoco` loads content through the asset root directory — it does not copy assets and does not depend on host absolute paths.

## Git LFS

Meshes, images, GLBs, and other large files are managed by Git LFS. Install Git LFS before the first clone:

```bash
git lfs install
git clone https://github.com/insightos-community/mujoco-asset.git
cd mujoco-asset
git lfs pull
```

Before committing, confirm that newly added large files have become LFS objects:

```bash
git lfs status
git lfs ls-files
git lfs fsck
```

Do not commit build directories, caches, virtual environments, Runtime logs, or temporary binaries of unknown origin directly.

## Directory layout

```text
assets/                    generic object assets
robot/                     Robot models, meshes, and semantic mappings
scene/                     Scene Packages, Layouts, and authoring templates
asset-catalog.v1.json      versioned asset catalog used by the Framework
```

The depalletizing scene is at `scene/palletizing_depalletizing_001` and includes three official Layouts, authoring templates, and an asset manifest. The versioned kinematic model bundle for Franka Panda is at `robot/franka_panda/model_bundle`.

## Extending Scene Packages

Each public native scene is an independent Scene Package under `scene/<scene_key>/`:

- `asset-manifest.yaml` is the sole public manifest of the directory index, declaring stable IDs, versions, Layouts, capabilities, and authoring constraints;
- `scene_info.yaml` holds only Runtime loading parameters, Robots, Sensors, and physics settings;
- `layout*.yaml` holds the official read-only Layouts;
- `authoring/` holds the templates from which Layouts can be derived and the compatible material set.

Adding a Scene Package requires no Runtime source changes. After installing or upgrading content, regenerate the frozen catalog:

```bash
mujoco-runtime catalog index \
  --profile native-mujoco \
  --content-root /absolute/path/to/mujoco_asset \
  --output /installation/content/catalog.yaml
```

Once started, the Runtime reads only this frozen catalog and does not scan the asset repo. The old `scene/scene_list.json` has been removed to avoid duplicate sources of truth alongside `asset-manifest.yaml`, `scene_info.yaml`, and the material directories.

## Usage

In development mode, pass this repository's root directory to the Runtime:

```bash
MUJOCO_ASSET_ROOT=/absolute/path/to/mujoco_asset \
MUJOCO_SCENE_CATALOG=/absolute/path/to/catalog.yaml \
uv run mujoco-runtime
```

In production, the RuntimeInstallation's `content_refs` register the asset locations; users are not required to set environment variables long-term.

## Asset governance

- Every publishable asset must have a stable ID, provenance, license, and distribution conclusion.
- Coordinates are right-handed, lengths are in meters, angles in radians, and public quaternion order is `xyzw`.
- Scenes reference relative paths; writing developer-host paths is forbidden.
- Published versions cannot be overwritten; changes are delivered as new versions or new Layouts.
- Assets marked `source: unknown`, `license: pending`, or `distribution_status: internal-only` may only be used for internal development and acceptance, and cannot be distributed as public artifacts.

## Pre-commit checks

```bash
git diff --check
git lfs fsck
python -m json.tool asset-catalog.v1.json >/dev/null
```

XML, URDF, JSON, YAML, and scene Layouts should also be verified by actual loading through `plugin-mujoco`'s native integration tests.

## Project layout

- `assets/`: reusable object assets.
- `robot/`: robot models and meshes.
- `scene/`: scene packages and layouts.
- `asset-catalog.v1.json`: asset catalog.
- `prototypes/`: experimental assets.

## Setup and validation

Install Git LFS first, then pull the assets from this repository:

```bash
git lfs install
git lfs pull
git lfs fsck
python3 check_external_models.py
python3 -m json.tool asset-catalog.v1.json > /dev/null
```

No executables need to be compiled. These commands validate LFS objects and JSON syntax but not full model semantics; selected scenes also need to be load-verified by the MuJoCo Runtime.

## Usage notes

When starting the standalone mujoco-runtime project, set `MUJOCO_ASSET_ROOT` to this repository's absolute path. quick-start registers the asset root into native MuJoCo. Scene / package manifests, referenced meshes, and layouts should be kept complete.

If a mesh file actually contains only a few lines of text, it is usually an LFS pointer. Finish the download first before troubleshooting rendering or model loading problems.

## ⚠️ Distribution boundary

Every distributable asset should have provenance, a license, a stable ID, and a distribution conclusion. Assets with unknown provenance, pending licenses, or marked internal-only **must not be published as public artifacts**.

The Apache license of first-party code does not change the licensing of third-party models or meshes. Keep the original notices and licenses, including the relevant files inside the Franka model bundle.

[Detailed asset reference](README.reference.md) · [License scope](LICENSE_SCOPE.md)

## License

Copyright 2026 InsightOS. First-party code is under [Apache-2.0](LICENSE); see [NOTICE](NOTICE) and [license scope](LICENSE_SCOPE.md) for third-party components and assets.

## Reproducing builds on three platforms

See the [glibc, musl, and macOS build guide](README.build.md): pinned source versions, actual script entries, tool requirements, local and CI commands, artifact locations, and per-platform verification scope.
