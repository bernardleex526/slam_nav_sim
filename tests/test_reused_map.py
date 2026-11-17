from pathlib import Path
import numpy as np
import pytest

from robot_validate.cli import main
from robot_validate.model import sha256
from robot_validate.ros_backend import reuse_generated_map, same_map
from robot_validate.slam import Grid


def test_reused_map_retains_pixels_geometry_and_immutable_source(tmp_path):
    source = tmp_path / "source"
    grid = Grid(np.array([[0, -1], [100, 0]], np.int8), 0.04, (-2.03, -2.11, 0.1))
    yaml = grid.save(source)
    hashes = {p.name: sha256(p) for p in source.iterdir()}
    output = tmp_path / "new-run"
    provenance = reuse_generated_map(yaml, output)
    assert same_map(Grid.load(output / "map.yaml"), grid)
    assert hashes == {p.name: sha256(p) for p in source.iterdir()}
    assert provenance["source_image_sha256"] == provenance["copied_image_sha256"]
    assert provenance["mapping_executed_this_run"] is False
    with pytest.raises(ValueError, match="separate"):
        reuse_generated_map(yaml, source)


def test_cli_saved_map_requires_navigation_only(tmp_path):
    with pytest.raises(SystemExit) as error:
        main(["sim", "--generated-map", str(tmp_path / "map.yaml")])
    assert error.value.code == 2


def test_cli_dispatches_saved_map_as_navigation_only(monkeypatch, tmp_path):
    captured = {}

    def run(request, output, timeout):
        captured.update(request)
        return 0, Path(output) / "report.html"

    monkeypatch.setattr("robot_validate.cli.supervised_run", run)
    source = tmp_path / "source/map.yaml"
    assert (
        main(
            [
                "sim",
                "--navigation-only",
                "--generated-map",
                str(source),
                "--output",
                str(tmp_path / "out"),
            ]
        )
        == 0
    )
    assert captured["navigation_only"] is True
    assert captured["generated_map"] == str(source.resolve())


@pytest.mark.parametrize("mode", ["raw", "scale"])
def test_reuse_rejects_unsupported_occupancy_semantics(tmp_path, mode):
    import yaml

    source = tmp_path / "source"
    path = Grid(np.array([[0, 100]], np.int8), 0.04, (0, 0, 0)).save(source)
    metadata = yaml.safe_load(path.read_text())
    metadata["mode"] = mode
    path.write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match="trinary"):
        reuse_generated_map(path, tmp_path / "output")
    assert not (tmp_path / "output/map.yaml").exists()


def test_reuse_rejects_alpha_image_instead_of_losing_unknown_semantics(tmp_path):
    from PIL import Image
    import yaml

    source = tmp_path / "source"
    path = Grid(np.array([[0, 100]], np.int8), 0.04, (0, 0, 0)).save(source)
    Image.new("RGBA", (2, 1), (0, 0, 0, 0)).save(source / "map.png")
    metadata = yaml.safe_load(path.read_text())
    metadata["image"] = "map.png"
    path.write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match="grayscale PGM"):
        reuse_generated_map(path, tmp_path / "output")
