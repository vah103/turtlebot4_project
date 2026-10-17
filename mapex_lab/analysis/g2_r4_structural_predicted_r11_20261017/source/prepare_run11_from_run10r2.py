#!/usr/bin/env python3
"""Stage isolated R11 native-2D source from frozen Run10-R2; NEVER starts a run.

Run on DELL/COM1 after checking out this branch. Use explicit --base and
--patch-dir, preserving source provenance and the original Run10-R2 output.
"""
from __future__ import annotations
import argparse
import hashlib
import re
import shutil
from pathlib import Path

FILES = (
    "explore_g2_r4_lama3_smallroi_run06.py",
    "g2_native_integration.py",
    "g2_r4_policy_sidecar.py",
    "g2_predicted_goals.py",
    "g2_local_backstop.py",
    "room_native_integration.py",
    "fast_native_lama_3model.py",
    "sidecar.py",
    "native_g2_r4_3model_worker_run10r2.py",
    "guard_g2_r4_run10r2.py",
)
PATCHED = (
    "g2_structural_arbitration.py",
    "g2_predicted_goals.py",
    "g2_r4_policy_sidecar.py",
    "test_r11_structural_arbitration.py",
)
OLD_RUN = "native_g2_r4_frontier_exit_run10r2"
NEW_RUN = "native_g2_r4_structural_predicted_run11"
OLD_LABEL = "G2_R4_FRONTIER_EXIT_RUN10R2"
NEW_LABEL = "G2_R4_STRUCTURAL_PREDICTED_RUN11"
OLD_GUARD = "guard_run10r2_frontier_exit"
NEW_GUARD = "guard_run11_structural_predicted"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage(base: Path, patch: Path, target: Path):
    if target.exists():
        raise FileExistsError("Refusing overwrite of output or any existing directory: "+str(target))
    if target == base or base in target.parents or target in base.parents:
        raise ValueError("base/target must be separate workspaces")
    for name in FILES:
        if not (base/name).is_file():
            raise FileNotFoundError(base/name)
    for name in PATCHED:
        if not (patch/name).is_file():
            raise FileNotFoundError(patch/name)
    if not (base/"g2src").is_dir() or not (base/"v41src").is_dir():
        raise FileNotFoundError("missing g2src/v41src from Run10-R2 native source")
    # Check original guard source pins, not previous run's produced output.
    guardtext = (base/"guard_g2_r4_run10r2.py").read_text()
    old_expected = dict(re.findall(r"'([^']+)':'([0-9a-f]{64})'", guardtext))
    for name, old_hash in old_expected.items():
        if name.startswith("../"):
            continue
        if not (base/name).is_file() or sha(base/name) != old_hash:
            raise ValueError("Run10-R2 frozen source pin mismatch: "+name)
    # Ensure clean new workspace: never copy prior output or evidence.
    target.mkdir(parents=True)
    for name in FILES:
        shutil.copy2(base/name, target/name)
    shutil.copytree(base/"g2src",target/"g2src",ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    shutil.copytree(base/"v41src",target/"v41src",ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    for name in PATCHED:
        shutil.copy2(patch/name,target/name)
    (target/"evidence").mkdir()
    (target/"reports").mkdir()
    original_root=str(base.resolve())
    new_root=str(target.resolve())
    if original_root == new_root:raise ValueError("identical run roots")
    substitutions={
        original_root:new_root, OLD_RUN:NEW_RUN, OLD_LABEL:NEW_LABEL,
        OLD_GUARD:NEW_GUARD,"worker_run10r2.py":"worker_run11.py",
    }
    for original,renamed in (
        ("native_g2_r4_3model_worker_run10r2.py","native_g2_r4_3model_worker_run11.py"),
        ("guard_g2_r4_run10r2.py","guard_g2_r4_run11.py"),
    ):
        p=target/original
        s=p.read_text()
        for old,new in substitutions.items():
            s=s.replace(old,new)
        if original_root in s or OLD_RUN in s or OLD_LABEL in s:
            raise ValueError("old root/run label survived in "+original)
        new_path=target/renamed
        new_path.write_text(s)
        p.unlink()
    guard=target/"guard_g2_r4_run11.py"
    g=guard.read_text()
    # Rehash every internally pinned file after the source overlays.
    expected=dict(re.findall(r"'([^']+)':'([0-9a-f]{64})'",g))
    for name,old_hash in expected.items():
        if name.startswith("../"):
            continue
        digest=sha(target/name)
        before=f"'{name}':'{old_hash}'"
        after=f"'{name}':'{digest}'"
        if g.count(before)!=1:
            raise ValueError("ambiguous SHA pin "+name)
        g=g.replace(before,after)
    if "'g2_structural_arbitration.py':" in g:
        raise ValueError("duplicate arbitration pin")
    g=g.replace("    expected={","    expected={\n      'g2_structural_arbitration.py':'"+sha(target/"g2_structural_arbitration.py")+"',",1)
    if "    expected={" not in g:
        raise ValueError("guard does not pin expected sources")
    # Reflect what changed in the run metadata.
    g=g.replace('"old_goal_chain_PLUS_PREDICTED_GOAL_WITH_WFD_EXIT_ONLY"',
        '"R11_STRUCTURAL_ARBITRATION_PLUS_PREDICTED_WITH_WFD_EXIT_ONLY"')
    guard.write_text(g)
    import py_compile
    for name in ("g2_structural_arbitration.py","g2_predicted_goals.py",
                 "g2_r4_policy_sidecar.py","native_g2_r4_3model_worker_run11.py",
                 "guard_g2_r4_run11.py","test_r11_structural_arbitration.py"):
        py_compile.compile(str(target/name),doraise=True)
    # Internal source pins must be correct after all scripts are written.
    pins=dict(re.findall(r"'([^']+)':'([0-9a-f]{64})'",guard.read_text()))
    for name,pin in pins.items():
        if name.startswith("../"):continue
        if sha(target/name)!=pin:raise AssertionError("bad final source pin "+name)
    if (target/NEW_RUN).exists():
        raise AssertionError("unexpected pre-existing run output")
    print("STAGED",target)
    print("PATCHED",list(PATCHED))
    print("PINNED_SOURCES",len(pins),"OUTPUT_ABSENT",True)
    print("RUN_NOT_STARTED",True)


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--base",type=Path,required=True,
                        help="Existing frozen Run10-R2 source root, NOT the run output")
    parser.add_argument("--patch-dir",type=Path,required=True,
                        help="Checked-out R11 source/ directory from GitHub branch")
    parser.add_argument("--target",type=Path,required=True,
                        help="New, initially nonexistent isolated R11 workspace")
    args=parser.parse_args()
    stage(args.base.resolve(),args.patch_dir.resolve(),args.target.resolve())
