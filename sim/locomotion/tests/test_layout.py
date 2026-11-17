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
    with_yaml = {p.parent.name for p in (ROOT / "policies").glob("*/policy.yaml")}
    assert "baseline" in names and with_yaml <= set(names)
    for name in names:
        assert policy_select.onnx(name).is_file()
        if name in with_yaml:  # ASMR policies: deployment config and training checkpoint
            assert policy_select.config(name).is_file() and policy_select.checkpoint(name).name.startswith("model_")
        else:  # BHL-model policies (f1_v1, f3_v1) have only the network
            assert policy_select.info(name)["trained_on"] == "bhl", name
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


def test_policy_folders_say_which_robot_they_are_for():
    """Every policy folder is an ASMR policy (policy.yaml) or has an info.json; the ASMR robot code warns about the others."""
    import sys

    import pytest

    sys.path.insert(0, str(ROOT))
    from policies import policy_select

    for name in policy_select.names():
        assert policy_select.info(name)["trained_on"] in ("asmr2026", "bhl"), name
    assert policy_select.info("baseline")["trained_on"] == "asmr2026"
    for name in ("f1_v1", "f3_v1"):
        assert policy_select.info(name)["trained_on"] == "bhl"
        assert policy_select.onnx(name).is_file()
        with pytest.warns(UserWarning, match="trained for 'bhl'"):
            assert policy_select.onnx(name, robot="asmr2026").is_file()
        with pytest.raises(ValueError, match="trained for 'bhl'"):
            policy_select.onnx(name, robot="asmr2026", strict=True)
    assert policy_select.onnx("baseline", robot="asmr2026").is_file()


def test_no_code_reads_the_archive_or_sim_rl():
    """archive/ is kept for reference only: no script opens a path inside it, and nothing points to the removed sim/rl."""
    pattern = re.compile(r'["\']archive["\']|["\']archive/|sim/rl/|"rl"')
    for path in ROOT.rglob("*.py"):
        rel = path.relative_to(ROOT).as_posix()
        if rel.startswith(("archive/", ".venv/", "third_party/")) or "/.venv/" in rel or rel == "sim/locomotion/tests/test_layout.py":
            continue
        for number, line in enumerate(path.read_text().splitlines(), 1):
            code = line.split("#")[0]
            if pattern.search(code) and not code.strip().startswith(('"', "'")):
                raise AssertionError(f"{rel}:{number}: {line.strip()}")


def _payloads():
    import sys

    sys.path.insert(0, str(ROOT))
    from source.robot_models.robots import asmr2026_payload

    return asmr2026_payload


def test_payload_options():
    """none, box_1kg (the task's box) and f3 (the CAD parts) are defined; the parts add up and the F3 deck lies on the plate."""
    import json

    import pytest

    payloads = _payloads()
    assert payloads.NAMES == ("none", "box_1kg", "f3") and payloads.DEFAULT == "box_1kg"
    assert payloads.parts("none") == [] and payloads.combined("none")[0] == 0.0
    box = payloads.parts("box_1kg")
    assert len(box) == 1 and box[0]["mass"] == 1.0 and abs(box[0]["position"][2] - 0.8838) < 1e-9
    task = (ROOT / "sim/locomotion/packages/berkeley_humanoid_lite/berkeley_humanoid_lite/tasks/locomotion/velocity/"
            "config/asmr_biped/env_cfg.py").read_text()
    assert "TORSO_TOP_Z_M = 0.8338" in task and "PAYLOAD_MASS_KG = 1.0" in task  # the box above is the task's
    layout = json.loads((ROOT / "cad/final/out_f3/layout.json").read_text())
    f3 = payloads.parts("f3")
    assert len(f3) == len(layout["items"]) > 40
    mass, centre, inertia = payloads.combined("f3")
    assert abs(mass - sum(i["mass_kg"] for i in layout["items"])) < 1e-9
    assert 3.5 < mass < 5.0 and 0.6 < centre[2] < 0.8 and abs(centre[1]) < 0.02
    assert abs(next(p for p in f3 if p["name"] == "payload_deck_f1")["position"][2] - 0.8338) < 2e-3  # on the top plate
    assert min(p["position"][2] - p["size"][2] / 2 for p in f3) > 0.5  # nothing hangs below the torso
    assert abs(inertia - inertia.T).max() < 1e-12 and (inertia.diagonal() > 0).all()
    assert payloads.sim_payload("none") is None and len(payloads.sim_payload("f3")["parts"]) == len(f3)
    assert set(payloads.sim_payload("box_1kg")) == {"mass", "position", "size"}
    with pytest.raises(ValueError, match="Available"):
        payloads.parts("f4")


def test_payload_combination_is_the_parallel_axis_sum():
    """Two 1 kg cubes of 0.2 m edge at x = +-0.5 m: centre 0, I_xx = 2 * 1/12 * 0.08, I_yy = I_zz = I_xx + 2 * 0.25."""
    import numpy as np

    payloads = _payloads()
    cube = dict(mass=1.0, size=(0.2, 0.2, 0.2))
    mass, centre, inertia = payloads.combine([dict(cube, position=(0.5, 0, 1)), dict(cube, position=(-0.5, 0, 1))])
    assert mass == 2.0 and np.allclose(centre, [0, 0, 1])
    assert np.allclose(np.diag(inertia), [2 * 0.08 / 12, 2 * 0.08 / 12 + 0.5, 2 * 0.08 / 12 + 0.5])
    assert np.allclose(inertia - np.diag(np.diag(inertia)), 0)


def test_sim2sim_payload_masses():
    """The ASMR MuJoCo model gets the parts of the chosen payload: 12.208 kg + 0, 1 and about 4.14 kg."""
    import importlib.util

    payloads = _payloads()
    spec = importlib.util.spec_from_file_location("asmr_sim2sim_run", ROOT / "sim/sim2sim/run.py")
    run = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(run)
    base = run.load_model(payload="none").body_subtreemass[1]
    assert abs(base - 12.208) < 0.01
    for name in ("box_1kg", "f3"):
        assert abs(run.load_model(payload=name).body_subtreemass[1] - base - payloads.combined(name)[0]) < 1e-6, name


def test_scripts_select_the_payload():
    """train.py, play.py and play_onnx.py take --payload; export_onnx.py records it; the env variable passes it to sim2sim workers."""
    import ast

    scripts = ROOT / "sim/locomotion/scripts/rsl_rl"
    for name in ("train.py", "play.py", "play_onnx.py"):
        text = (scripts / name).read_text()
        assert "asmr2026_payload.add_argument(parser)" in text and "select_payload(env_cfg, args_cli.payload)" in text, name
    assert "asmr2026_payload.add_argument(parser)" in (scripts / "export_onnx.py").read_text()
    task = ROOT / "sim/locomotion/packages/berkeley_humanoid_lite/berkeley_humanoid_lite"
    for path in (task / "tasks/locomotion/velocity/config/asmr_biped/payload.py", task / "environments/mujoco.py",
                 task / "tasks/locomotion/velocity/mdp/dynamics.py", ROOT / "sim/locomotion/scripts/s2r/eval_dynamics_gap.py",
                 ROOT / "sim/locomotion/scripts/sim2sim/eval_tracking_isaaclab.py"):
        ast.parse(path.read_text())
    assert "SIM2SIM_PAYLOAD" in (ROOT / "sim/sim2sim/run.py").read_text()


def test_payload_merge_matches_mujoco_fusion():
    """Merging the F3 parts into the base link (what Isaac Lab does, by the parallel-axis sum of
    mdp.dynamics.combine_with_payload) gives the mass, centre and inertia that MuJoCo computes when it fuses the parts."""
    import importlib.util

    import mujoco
    import numpy as np

    payloads = _payloads()
    spec = importlib.util.spec_from_file_location("asmr_sim2sim_run_fuse", ROOT / "sim/sim2sim/run.py")
    run = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(run)

    def base(payload):
        mjspec = mujoco.MjSpec.from_file(str(run.MODEL_PATH))
        run.add_payload(mjspec, payload)
        mjspec.compiler.fusestatic = True  # welded bodies (IMU, parts) become part of the base body
        model = mjspec.compile()
        i = model.body("base").id
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, model.body_iquat[i])
        rot = rot.reshape(3, 3)
        return model.body_mass[i], model.body_ipos[i].copy(), rot @ np.diag(model.body_inertia[i]) @ rot.T

    mass0, com0, inertia0 = base("none")
    mass1, com1, inertia1 = base("f3")
    pmass, pcom, pinertia = payloads.combined("f3")

    def shift(m, d):
        return m * (d @ d * np.eye(3) - np.outer(d, d))

    mass = mass0 + pmass
    com = (mass0 * com0 + pmass * pcom) / mass
    inertia = inertia0 + shift(mass0, com0 - com) + pinertia + shift(pmass, pcom - com)
    assert abs(mass - mass1) < 1e-9 and np.abs(com - com1).max() < 1e-9 and np.abs(inertia - inertia1).max() < 1e-9
