"""Static checks of the locomotion layout; need neither Isaac Sim nor a GPU.

Run from the repository root:  uv run --extra dev pytest scripts/isaaclab/test_layout.py -q
"""

import py_compile
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = sorted((ROOT / "scripts" / "isaaclab").rglob("*.py"))
SOURCES = sorted((ROOT / "source").rglob("*.py"))
DEPLOY_CONFIGS = sorted((ROOT / "policies").glob("*/policy.yaml")) + sorted((ROOT / "configs").glob("*.yaml"))


def _value(path, key):
    match = re.search(rf"^{key}:\s*(\S+)", path.read_text(), re.MULTILINE)
    return match.group(1) if match else None


def test_scripts_and_sources_compile(tmp_path):
    for i, path in enumerate(SCRIPTS + SOURCES):
        py_compile.compile(str(path), cfile=str(tmp_path / f"{i}.pyc"), doraise=True)


def test_mjcf_paths_exist():
    assert DEPLOY_CONFIGS
    for path in DEPLOY_CONFIGS:
        mjcf = _value(path, "mjcf_path")
        if mjcf:
            assert (ROOT / mjcf).is_file(), f"{path.relative_to(ROOT)}: {mjcf}"


def test_policy_files_exist():
    for config in (ROOT / "policies").glob("*/policy.yaml"):
        onnx = _value(config, "policy_checkpoint_path")
        assert onnx and (ROOT / onnx).is_file(), f"{config.relative_to(ROOT)}: {onnx}"
        assert list(config.parent.glob("model_*.pt")), f"no checkpoint in {config.parent}"
        assert (config.parent / "params" / "env.yaml").is_file()


def test_no_stale_script_paths():
    old = re.compile(r"scripts/(rsl_rl|sim2sim|push_test|s2r|scenario|sim2real)/")
    for path in SCRIPTS:
        for number, line in enumerate(path.read_text().splitlines(), 1):
            assert not old.search(line), f"{path.relative_to(ROOT)}:{number}: {line.strip()}"


def test_robot_model_has_single_definition():
    assert not (ROOT / "source/berkeley_humanoid_lite_assets/berkeley_humanoid_lite_assets/robots/asmr2026_humanoid.py").exists()
    for path in SCRIPTS + SOURCES:
        if path == Path(__file__):
            continue
        text = path.read_text()
        assert "berkeley_humanoid_lite_assets.robots.asmr2026_humanoid" not in text, path.relative_to(ROOT)
