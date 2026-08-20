from pathlib import Path

import pytest

from botix_navigation.save_map import map_prefix


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
