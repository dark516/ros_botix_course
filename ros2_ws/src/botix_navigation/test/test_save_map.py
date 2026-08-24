import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from botix_navigation.save_map import cleanup_outputs, map_prefix, save_occupancy_map


def test_map_prefix_creates_parent(tmp_path):
    prefix = map_prefix(str(tmp_path / "maps" / "lab"))

    assert prefix == (tmp_path / "maps" / "lab").resolve()
    assert prefix.parent.is_dir()


def test_map_prefix_rejects_extension(tmp_path):
    with pytest.raises(Exception, match="without an extension"):
        map_prefix(str(tmp_path / "lab.yaml"))


def test_map_prefix_refuses_existing_output(tmp_path):
    (tmp_path / "lab.yaml").touch()

    with pytest.raises(Exception, match="already exists"):
        map_prefix(str(tmp_path / "lab"))


def test_save_occupancy_map_uses_nav2_cli(tmp_path):
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0)

    prefix = tmp_path / "lab"

    assert save_occupancy_map(prefix, run=run)
    assert calls == [
        (
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                str(prefix),
            ],
            {"check": False, "timeout": 30.0},
        )
    ]


@pytest.mark.parametrize("returncode", [1, 2])
def test_save_occupancy_map_propagates_cli_failure(tmp_path, returncode):
    def run(*_args, **_kwargs):
        return SimpleNamespace(returncode=returncode)

    assert not save_occupancy_map(tmp_path / "lab", run=run)


def test_save_occupancy_map_handles_timeout(tmp_path):
    def run(*_args, **_kwargs):
        raise subprocess.TimeoutExpired("map_saver_cli", 30.0)

    assert not save_occupancy_map(tmp_path / "lab", run=run)


def test_cleanup_outputs_removes_only_map_artifacts(tmp_path):
    prefix = tmp_path / "lab"
    outputs = [prefix.with_suffix(suffix) for suffix in (".yaml", ".pgm", ".posegraph", ".data")]
    unrelated = prefix.with_suffix(".txt")
    for path in (*outputs, unrelated):
        path.touch()

    cleanup_outputs(prefix)

    assert not any(path.exists() for path in outputs)
    assert unrelated.exists()
