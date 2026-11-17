from dataclasses import dataclass
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

from .data import write_tum
from .model import dump_json
from .scan_matcher import ScanMatcher, rotate

FREE_THRESH = 0.196
OCCUPIED_THRESH = 0.65


def grayscale_to_cells(pixels, negate=False, free=FREE_THRESH, occupied=OCCUPIED_THRESH):
    probability = pixels.astype(float) / 255 if negate else (255 - pixels.astype(float)) / 255
    cells = np.full(pixels.shape, -1, np.int8)
    cells[probability < free] = 0
    cells[probability > occupied] = 100
    return cells


def native_ranges(ranges, max_range):
    # CoreSLAM: zero clears space; a tiny positive distance is skipped by its hole-width guard.
    output = np.ones(len(ranges), dtype=int)
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges <= max_range)
    output[valid] = np.rint(ranges[valid] * 1000).astype(int)
    output[np.isposinf(ranges)] = 0
    return output.tolist()


@dataclass
class Grid:
    cells: (
        np.ndarray
    )  # ROS occupancy semantics: -1 unknown, 0 free, 100 occupied; y increases upward
    resolution: float
    origin: tuple

    def save(self, directory, name="map"):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        pixels = np.full(self.cells.shape, 205, np.uint8)
        pixels[self.cells == 0] = 254
        pixels[self.cells == 100] = 0
        Image.fromarray(np.flipud(pixels)).save(directory / f"{name}.pgm")
        meta = dict(
            image=f"{name}.pgm",
            mode="trinary",
            resolution=self.resolution,
            origin=[float(value) for value in self.origin],
            negate=0,
            occupied_thresh=OCCUPIED_THRESH,
            free_thresh=FREE_THRESH,
        )
        path = directory / f"{name}.yaml"
        path.write_text(yaml.safe_dump(meta))
        return path

    @classmethod
    def load(cls, path):
        path = Path(path)
        meta = yaml.safe_load(path.read_text())
        pixels = np.flipud(np.asarray(Image.open(path.parent / meta["image"]).convert("L")))
        cells = grayscale_to_cells(
            pixels, meta.get("negate", 0), meta["free_thresh"], meta["occupied_thresh"]
        )
        resolution, origin = float(meta["resolution"]), tuple(map(float, meta["origin"]))
        if resolution <= 0 or len(origin) != 3 or not np.isfinite([resolution, *origin]).all():
            raise ValueError("Invalid map geometry")
        return cls(cells, resolution, origin)


def run_breezy(frames, spec, seed, output, initial=(0.0, 0.0, 0.0), matcher="point_to_line"):
    import pybreezyslam
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser

    if not frames:
        raise ValueError("No sensor frames")
    if matcher not in ("point_to_line", "rmhc"):
        raise ValueError(f"Unknown local scan matcher: {matcher}")
    if matcher == "point_to_line" and any(frame.beam_dt_s != 0 for frame in frames):
        raise ValueError(
            "Local point_to_line requires instantaneous or pre-deskewed scans; use the ROS backend for rolling scans"
        )
    laser = Laser(
        spec.size,
        spec.rate,
        spec.fov,
        round(spec.max_range * 1000),
        spec.margin,
        offset_mm=spec.offset * 1000,
    )
    slam = RMHC_SLAM(laser, 800, 32, random_seed=seed) if matcher == "rmhc" else None
    grid_map = slam.map if slam else pybreezyslam.Map(800, 32)
    scan = pybreezyslam.Scan(laser, 1)
    registration = ScanMatcher(spec)
    poses = []
    diagnostics = []
    for frame in frames:
        ranges = native_ranges(frame.ranges, spec.max_range)
        if slam:
            slam.update(ranges, (frame.distance * 1000, np.rad2deg(frame.yaw), frame.dt))
            x, y, yaw = slam.getpos()
            pose = np.array([x / 1000 - 16, y / 1000 - 16, np.deg2rad(yaw)])
        else:
            pose, diagnostic = registration.update(frame)
            diagnostics.append(diagnostic)
            # Exactly one ray per measured beam. A span of 3 invents free rays across depth edges.
            scan.update(
                scans_mm=ranges,
                hole_width_mm=600,
                velocities=(0.0, 0.0),
                scan_angles_degrees=None,
            )
            laser_xy = pose[:2] + rotate(np.array([[spec.offset, 0]]), pose[2])[0]
            grid_map.update(
                scan,
                pybreezyslam.Position(
                    (laser_xy[0] + 16) * 1000, (laser_xy[1] + 16) * 1000, np.rad2deg(pose[2])
                ),
                50,
                600,
            )
        world_xy = rotate(pose[None, :2], initial[2])[0] + initial[:2]
        poses.append(
            [
                frame.stamp,
                *world_xy,
                pose[2] + initial[2],
            ]
        )
    raw = bytearray(800 * 800)
    grid_map.get(raw)
    pixels = np.frombuffer(raw, np.uint8).reshape(800, 800)
    # The native map uses increasing row = increasing world Y (PGM export flips rows).
    cells = grayscale_to_cells(pixels)
    # Native pixel i is centered at i*resolution. ROS origin is the lower-left cell corner.
    corner = rotate(np.array([[-16.02, -16.02]]), initial[2])[0] + initial[:2]
    grid = Grid(cells, 0.04, (*corner, initial[2]))
    dump_json(
        Path(output) / "mapping-diagnostics.json",
        dict(
            matcher=matcher,
            truth_input=False,
            mapping_ray_span=1 if matcher == "point_to_line" else 3,
            motion_compensation="none: instantaneous/pre-deskewed scans"
            if matcher == "point_to_line"
            else "legacy native velocity model",
            hole_width_mm=600,
            map_quality=50,
            free_probability=FREE_THRESH,
            occupied_probability=OCCUPIED_THRESH,
            frames=diagnostics,
        ),
    )
    poses = np.asarray(poses)
    write_tum(Path(output) / "estimated.tum", poses)
    return poses, grid
