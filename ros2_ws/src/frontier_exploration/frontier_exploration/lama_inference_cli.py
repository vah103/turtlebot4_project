"""CLI shim that selects the working Python environment for LaMa inference."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from frontier_exploration.lama_inference_runner import main as runner_main


def _has_torch(python_executable: Path) -> bool:
    if not python_executable.is_file():
        return False
    result = subprocess.run(
        [str(python_executable), '-c', 'import torch'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def resolve_default_lama_python() -> str:
    """Prefer the known ``lama_cpu`` conda environment, then current Python."""
    candidates: list[Path] = []

    explicit = os.environ.get('LAMA_PYTHON', '').strip()
    if explicit:
        candidates.append(Path(explicit).expanduser())

    conda_prefix = os.environ.get('CONDA_PREFIX', '').strip()
    if conda_prefix:
        prefix = Path(conda_prefix).expanduser()
        if prefix.name == 'lama_cpu':
            candidates.append(prefix / 'bin' / 'python')
        candidates.append(prefix / 'envs' / 'lama_cpu' / 'bin' / 'python')
        if prefix.parent.name == 'envs':
            candidates.append(prefix.parent / 'lama_cpu' / 'bin' / 'python')

    conda_exe = os.environ.get('CONDA_EXE', '').strip()
    if conda_exe:
        conda_root = Path(conda_exe).expanduser().resolve().parent.parent
        candidates.append(conda_root / 'envs' / 'lama_cpu' / 'bin' / 'python')

    candidates.extend(
        [
            Path.home() / 'miniconda3' / 'envs' / 'lama_cpu' / 'bin' / 'python',
            Path.home() / 'anaconda3' / 'envs' / 'lama_cpu' / 'bin' / 'python',
            Path.home() / '.conda' / 'envs' / 'lama_cpu' / 'bin' / 'python',
            Path(sys.executable),
        ]
    )

    seen: set[Path] = set()
    for candidate in candidates:
        candidate = candidate.expanduser().resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        if _has_torch(candidate):
            return str(candidate)

    raise RuntimeError(
        'Cannot find a Python interpreter with PyTorch for LaMa. '
        'Expected the existing conda environment named lama_cpu. '
        'Activate it or pass --python /path/to/python, or set LAMA_PYTHON.'
    )


def main(args: list[str] | None = None) -> None:
    forwarded = list(sys.argv[1:] if args is None else args)
    if '--python' not in forwarded:
        forwarded.extend(['--python', resolve_default_lama_python()])
    runner_main(forwarded)


if __name__ == '__main__':
    main()
