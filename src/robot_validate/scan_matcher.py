"""Deterministic, sensor-only point-to-line registration for the local map adapter.

The submap contains measured laser endpoints transformed by previous estimates.
No evaluator, reference trajectory, reference map, or world geometry is an input.
"""

import numpy as np
from scipy.spatial import cKDTree


def rotate(points, yaw):
    cosine, sine = np.cos(yaw), np.sin(yaw)
    return points @ np.array([[cosine, sine], [-sine, cosine]])


class ScanMatcher:
    def __init__(self, spec):
        self.spec = spec
        angles = np.linspace(-np.deg2rad(spec.fov) / 2, np.deg2rad(spec.fov) / 2, spec.size)
        self.rays = np.column_stack((np.cos(angles), np.sin(angles)))
        self.pose = np.zeros(3)
        self.points = np.empty((0, 2))
        self.normals = np.empty((0, 2))
        self.tree = None
        self.last_keyframe = None

    def cloud(self, frame):
        finite_ranges = np.nan_to_num(frame.ranges, nan=0.0, posinf=0.0, neginf=0.0)
        points = finite_ranges[:, None] * self.rays + [self.spec.offset, 0]
        tangent = points[2:] - points[:-2]
        lengths = np.linalg.norm(tangent, axis=1)
        normals = np.column_stack((-tangent[:, 1], tangent[:, 0])) / np.maximum(
            lengths[:, None], 1e-8
        )
        # Reject normals that span a depth discontinuity or invalid/no-return beam.
        valid_ranges = (
            np.isfinite(frame.ranges)
            & (frame.ranges > 0.1)
            & (frame.ranges < self.spec.max_range - 0.02)
        )
        # Match CoreSLAM's beam window, including every neighbor used to estimate a normal.
        valid_ranges[: self.spec.margin + 1] = False
        valid_ranges[self.spec.size - self.spec.margin :] = False
        valid = valid_ranges[:-2] & valid_ranges[1:-1] & valid_ranges[2:]
        valid &= lengths > 1e-6
        valid &= np.linalg.norm(points[2:] - points[1:-1], axis=1) < 0.3
        valid &= np.linalg.norm(points[1:-1] - points[:-2], axis=1) < 0.3
        return points[1:-1][valid], normals[valid]

    def update(self, frame):
        prior = self.pose.copy()
        prior[:2] += frame.distance * np.array([np.cos(prior[2]), np.sin(prior[2])])
        prior[2] += frame.yaw
        pose = prior.copy()
        points, normals = self.cloud(frame)
        state, matches, residual_rmse, iterations = "initializing", 0, None, 0
        if self.tree is not None:
            state = "insufficient_geometry"
            for iterations in range(1, 21):
                turned = rotate(points, pose[2])
                world = turned + pose[:2]
                distance, nearest = self.tree.query(world)
                target, normal = self.points[nearest], self.normals[nearest]
                valid = (distance < 0.3) & (
                    abs(np.sum(rotate(normals, pose[2]) * normal, axis=1)) > 0.8
                )
                matches = int(valid.sum())
                if matches < 30:
                    pose = prior
                    state = "insufficient_geometry"
                    residual_rmse = None
                    break
                residual = np.sum((world - target) * normal, axis=1)[valid]
                jacobian = np.column_stack(
                    (
                        normal[:, 0],
                        normal[:, 1],
                        -normal[:, 0] * turned[:, 1] + normal[:, 1] * turned[:, 0],
                    )
                )[valid]
                weights = np.sqrt(np.minimum(1.0, 0.03 / np.maximum(abs(residual), 1e-8)))
                delta, _, rank, singular = np.linalg.lstsq(
                    jacobian * weights[:, None], -residual * weights, rcond=None
                )
                if rank < 3 or singular[-1] / singular[0] < 1e-4 or not np.isfinite(delta).all():
                    pose = prior
                    state = "degenerate_geometry"
                    residual_rmse = None
                    break
                pose += delta
                residual_rmse = float(np.sqrt(np.mean(residual**2)))
                state = "matched"
                if np.linalg.norm(delta) < 1e-6:
                    break
        self.pose = pose
        if len(points) >= 30 and (
            self.last_keyframe is None
            or (
                state == "matched"
                and (
                    np.linalg.norm(pose[:2] - self.last_keyframe[:2]) > 0.3
                    or abs(pose[2] - self.last_keyframe[2]) > 0.15
                )
            )
        ):
            world = rotate(points, pose[2]) + pose[:2]
            all_points = np.concatenate((self.points, world))
            all_normals = np.concatenate((self.normals, rotate(normals, pose[2])))
            _, indices = np.unique(
                np.floor(all_points / 0.03).astype(int), axis=0, return_index=True
            )
            self.points, self.normals = all_points[indices], all_normals[indices]
            # Bound long bag memory without importing any known-world coordinates.
            if len(self.points) > 50000:
                indices = np.argsort(np.linalg.norm(self.points - pose[:2], axis=1), kind="stable")[
                    :50000
                ]
                self.points, self.normals = self.points[indices], self.normals[indices]
            self.tree = cKDTree(self.points)
            self.last_keyframe = pose.copy()
        return pose.copy(), dict(
            timestamp_s=frame.stamp,
            state=state,
            matches=matches,
            iterations=iterations,
            residual_rmse_m=residual_rmse,
            correction_translation_m=float(np.linalg.norm(pose[:2] - prior[:2])),
            correction_yaw_rad=float(pose[2] - prior[2]),
            submap_points=len(self.points),
        )
