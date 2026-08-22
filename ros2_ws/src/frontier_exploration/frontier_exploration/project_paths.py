"""Resolve project-owned data paths without machine-specific home paths."""

from __future__ import annotations

import os
from pathlib import Path


PROJECT_ROOT_ENV = 'TURTLEBOT4_PROJECT_ROOT'
DEFAULT_LAMA_RUNS_DIR = 'data/lama_runs'


def _project_marker(root: Path) -> Path:
    return root / 'ros2_ws' / 'src' / 'frontier_exploration' / 'package.xml'


def _validated_project_root(value: str | Path) -> Path:
    root = Path(
        os.path.expandvars(os.path.expanduser(str(value)))
    ).resolve()
    if not _project_marker(root).is_file():
        raise RuntimeError(
            f'{root} is not the TurtleBot4 project root: '
            f'missing {_project_marker(root)}'
        )
    return root


def find_project_root(start: str | Path | None = None) -> Path:
    """Return the repository root, preferring an explicit environment value."""
    configured = os.environ.get(PROJECT_ROOT_ENV, '').strip()
    if configured:
        return _validated_project_root(configured)

    starts = [Path(start).resolve()] if start is not None else []
    starts.extend((Path.cwd().resolve(), Path(__file__).resolve()))

    checked: set[Path] = set()
    for candidate in starts:
        if candidate.is_file():
            candidate = candidate.parent
        for root in (candidate, *candidate.parents):
            if root in checked:
                continue
            checked.add(root)
            if _project_marker(root).is_file():
                return root

    raise RuntimeError(
        'Cannot locate the TurtleBot4 project root. Run from inside the '
        f'repository or export {PROJECT_ROOT_ENV}='
        '/absolute/path/to/repository.'
    )


def resolve_project_path(
    value: str | Path,
    start: str | Path | None = None,
) -> Path:
    """Expand a path; anchor relative values at the TurtleBot4 project root."""
    raw_value = os.path.expandvars(os.path.expanduser(str(value).strip()))
    if not raw_value:
        raise ValueError('Path value must not be empty')

    path = Path(raw_value)
    if path.is_absolute():
        return path.resolve()
    return (find_project_root(start) / path).resolve()
