"""ROS-free regression checks for R003 paper500 constructor ordering."""

import ast
from pathlib import Path
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def _self_attr_assignment_line(init: ast.FunctionDef, attr_name: str) -> int:
    lines = []
    for node in ast.walk(init):
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        for target in targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
                and target.attr == attr_name
            ):
                lines.append(node.lineno)
    if not lines:
        raise AssertionError(f"self.{attr_name} assignment not found")
    return min(lines)


def _self_method_call_line(init: ast.FunctionDef, method_name: str) -> int:
    lines = []
    for node in ast.walk(init):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (
            isinstance(func, ast.Attribute)
            and isinstance(func.value, ast.Name)
            and func.value.id == "self"
            and func.attr == method_name
        ):
            lines.append(node.lineno)
    if not lines:
        raise AssertionError(f"self.{method_name}(...) call not found")
    return min(lines)


class Paper500InitializationOrderTest(unittest.TestCase):
    def _assert_budget_before_provenance(self, filename: str, class_name: str):
        tree = ast.parse((SCRIPTS / filename).read_text(encoding="utf-8"))
        cls = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == class_name
        )
        init = next(
            node
            for node in cls.body
            if isinstance(node, ast.FunctionDef) and node.name == "__init__"
        )
        budget_line = _self_attr_assignment_line(init, "paper500_budget")
        provenance_line = _self_method_call_line(init, "_write_initial_provenance")
        self.assertLess(
            budget_line,
            provenance_line,
            f"{filename}: paper500_budget must exist before initial provenance",
        )

    def test_nf_budget_exists_before_initial_provenance(self):
        self._assert_budget_before_provenance("_nf_run_core.py", "Stage2Run")

    def test_mapex_budget_exists_before_initial_provenance(self):
        self._assert_budget_before_provenance("mapex_run.py", "MapExRun")


if __name__ == "__main__":
    unittest.main()
