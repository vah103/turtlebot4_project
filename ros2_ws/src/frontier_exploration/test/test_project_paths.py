from pathlib import Path

import pytest

from frontier_exploration.project_paths import (
    PROJECT_ROOT_ENV,
    find_project_root,
    resolve_project_path,
)


def _make_project(tmp_path: Path) -> Path:
    project = tmp_path / 'turtlebot4_project'
    package = project / 'ros2_ws' / 'src' / 'frontier_exploration'
    package.mkdir(parents=True)
    (package / 'package.xml').write_text('<package/>', encoding='utf-8')
    return project


def test_environment_project_root_is_preferred(tmp_path, monkeypatch):
    project = _make_project(tmp_path)
    monkeypatch.setenv(PROJECT_ROOT_ENV, str(project))

    assert find_project_root(tmp_path) == project.resolve()


def test_searches_parent_directories(tmp_path, monkeypatch):
    project = _make_project(tmp_path)
    nested = project / 'ros2_ws' / 'build' / 'nested'
    nested.mkdir(parents=True)
    monkeypatch.delenv(PROJECT_ROOT_ENV, raising=False)

    assert find_project_root(nested) == project.resolve()


def test_relative_output_is_inside_project(tmp_path, monkeypatch):
    project = _make_project(tmp_path)
    monkeypatch.setenv(PROJECT_ROOT_ENV, str(project))

    assert resolve_project_path('data/lama_runs') == (
        project / 'data' / 'lama_runs'
    ).resolve()


def test_absolute_override_remains_supported(tmp_path, monkeypatch):
    project = _make_project(tmp_path)
    output = tmp_path / 'external-output'
    monkeypatch.setenv(PROJECT_ROOT_ENV, str(project))

    assert resolve_project_path(output) == output.resolve()


def test_invalid_environment_root_fails_loudly(tmp_path, monkeypatch):
    monkeypatch.setenv(PROJECT_ROOT_ENV, str(tmp_path))

    with pytest.raises(RuntimeError, match='not the TurtleBot4 project root'):
        find_project_root()


def test_empty_path_is_rejected():
    with pytest.raises(ValueError, match='must not be empty'):
        resolve_project_path('')
