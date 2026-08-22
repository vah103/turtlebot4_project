from pathlib import Path

from frontier_exploration import lama_inference_cli


def test_resolve_default_lama_python_prefers_named_conda_env(monkeypatch, tmp_path: Path) -> None:
    conda_root = tmp_path / 'miniconda3'
    conda_exe = conda_root / 'bin' / 'conda'
    expected = (conda_root / 'envs' / 'lama_cpu' / 'bin' / 'python').resolve()

    monkeypatch.setenv('CONDA_EXE', str(conda_exe))
    monkeypatch.delenv('CONDA_PREFIX', raising=False)
    monkeypatch.delenv('LAMA_PYTHON', raising=False)
    monkeypatch.setattr(
        lama_inference_cli,
        '_has_torch',
        lambda candidate: candidate == expected,
    )

    assert lama_inference_cli.resolve_default_lama_python() == str(expected)


def test_main_injects_resolved_python_when_not_explicit(monkeypatch) -> None:
    captured = []
    monkeypatch.setattr(
        lama_inference_cli,
        'resolve_default_lama_python',
        lambda: '/opt/conda/envs/lama_cpu/bin/python',
    )
    monkeypatch.setattr(
        lama_inference_cli,
        'runner_main',
        lambda args: captured.extend(args),
    )

    lama_inference_cli.main(['eval_dir', '--dry-run'])

    assert captured == [
        'eval_dir',
        '--dry-run',
        '--python',
        '/opt/conda/envs/lama_cpu/bin/python',
    ]


def test_main_preserves_explicit_python(monkeypatch) -> None:
    captured = []
    monkeypatch.setattr(
        lama_inference_cli,
        'resolve_default_lama_python',
        lambda: (_ for _ in ()).throw(AssertionError('should not resolve')),
    )
    monkeypatch.setattr(
        lama_inference_cli,
        'runner_main',
        lambda args: captured.extend(args),
    )

    lama_inference_cli.main(['eval_dir', '--python', '/custom/python'])

    assert captured == ['eval_dir', '--python', '/custom/python']
