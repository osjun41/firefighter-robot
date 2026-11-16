"""Static checks of the locomotion layout; need neither Isaac Sim nor a GPU.

Run from the repository root:  uv run --extra dev pytest sim/locomotion/tests/test_layout.py -q
"""

import py_compile
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LOCO = ROOT / "sim" / "locomotion"
SCRIPTS = sorted((LOCO / "scripts").rglob("*.py"))
SOURCES = sorted((LOCO / "packages").rglob("*.py")) + sorted((ROOT / "source").rglob("*.py"))
DEPLOY_CONFIGS = sorted((ROOT / "policies").glob("*/policy.yaml")) + sorted((LOCO / "configs").glob("*.yaml"))


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
    old = re.compile(r"(?<![\w/])scripts/(isaaclab|rsl_rl|sim2sim|push_test|s2r|scenario|sim2real)/")
    for path in SCRIPTS:
        for number, line in enumerate(path.read_text().splitlines(), 1):
            assert not old.search(line), f"{path.relative_to(ROOT)}:{number}: {line.strip()}"


def test_robot_model_has_single_definition():
    assert not (LOCO / "packages/berkeley_humanoid_lite_assets").exists(), "the BHL robot is not part of this repository"
    for path in SCRIPTS + SOURCES:
        if path == Path(__file__):
            continue
        text = path.read_text()
        assert "berkeley_humanoid_lite_assets" not in text, path.relative_to(ROOT)


def _module_exists(name):
    """True if `name` (dotted) is a module or package file below one of the repository's source roots."""
    roots = [ROOT] + [p / sub for p in (LOCO / "packages").iterdir() for sub in ("",)]
    parts = name.split(".")
    for root in roots:
        base = root.joinpath(*parts)
        if base.with_suffix(".py").is_file() or (base / "__init__.py").is_file():
            return True
        # src layout: packages/<project>/<package>/...
        for project in (LOCO / "packages").iterdir():
            base = project.joinpath(*parts)
            if base.with_suffix(".py").is_file() or (base / "__init__.py").is_file():
                return True
    return False


def test_internal_imports_resolve():
    """Every import of our own packages (berkeley_humanoid_lite*, source.robot_models) points at a file that exists."""
    import ast

    own = ("berkeley_humanoid_lite", "source.", "policies.")
    for path in SCRIPTS + SOURCES + sorted(LOCO.joinpath("tests").glob("*.py")):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                names = [node.module]
            elif isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            for name in names:
                if name.startswith(own):
                    assert _module_exists(name), f"{path.relative_to(ROOT)}: cannot resolve `{name}`"


def test_policy_select_by_name():
    """Policies are chosen by folder name; no name means baseline; a wrong name lists the available ones."""
    import sys

    import pytest

    sys.path.insert(0, str(ROOT))
    from policies import policy_select

    assert policy_select.onnx() == ROOT / "policies" / "baseline" / "policy.onnx"
    assert policy_select.config() == ROOT / "policies" / "baseline" / "policy.yaml"
    assert policy_select.chosen(None) == ["baseline"]
    names = policy_select.names()
    assert "baseline" in names and len(names) == len(list((ROOT / "policies").glob("*/policy.yaml")))
    for name in names:
        assert policy_select.onnx(name).is_file() and policy_select.config(name).is_file()
        assert policy_select.checkpoint(name).name.startswith("model_")
    with pytest.raises(FileNotFoundError, match="baseline"):
        policy_select.folder("no_such_policy")
    for bad in ("policies/baseline", "baseline/policy.onnx", "policy.onnx", ".."):
        with pytest.raises(ValueError):
            policy_select.folder(bad)


def test_scripts_select_policies_by_name():
    """No script asks for a policy path (--config) or a NAME=CHECKPOINT pair; robot/config.toml names a policy folder."""
    for path in SCRIPTS:
        text = path.read_text()
        assert '"--config"' not in text.replace("markers.config", ""), f"{path.relative_to(ROOT)}: use --policy NAME"
        assert "NAME=CHECKPOINT" not in text, path.relative_to(ROOT)
    toml = (ROOT / "robot" / "config.toml").read_text()
    match = re.search(r'^\[legs\][^\[]*?^policy\s*=\s*"([^"]*)"', toml, re.MULTILINE | re.DOTALL)
    assert match and "/" not in match.group(1) and not match.group(1).endswith(".onnx"), "[legs] policy is a NAME"


def test_old_task_names_are_not_registered():
    registered = "\n".join(p.read_text() for p in (LOCO / "packages").rglob("__init__.py"))
    assert not re.search(r'id\s*=\s*["\']Velocity-Berkeley-Humanoid-Lite-Biped', registered)
    ids = re.findall(r'\bid\s*=\s*f?["\'](Velocity-[^"\']*)', registered)
    assert ids and all(i.startswith("Velocity-ASMR-Biped") for i in ids), ids
