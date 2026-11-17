from copy import deepcopy
import math

import numpy as np
from scipy.spatial import cKDTree

from .model import Check


def trajectory_checks(estimated, reference, thresholds, independent=True, alignment="se3"):
    from evo.core import metrics, sync, trajectory
    from evo.core.units import Unit

    checks, details = [], {"alignment": alignment, "scale_correction": False}
    if reference is None or not independent:
        reason = "缺少独立参考轨迹" if reference is None else "参考来源与算法输入不独立"
        return [Check("ATE", "mapping", "insufficient", reason=reason)], details, None

    def make(data):
        if len(data) < 2 or not np.isfinite(data).all() or np.any(np.diff(data[:, 0]) <= 0):
            raise ValueError("Trajectory must have finite poses and strictly increasing times")
        xyz = np.column_stack((data[:, 1:3], np.zeros(len(data))))
        quat = np.column_stack(
            (np.cos(data[:, 3] / 2), np.zeros((len(data), 2)), np.sin(data[:, 3] / 2))
        )
        return trajectory.PoseTrajectory3D(xyz, quat, data[:, 0])

    ref, est = make(reference), make(estimated)
    try:
        paired_ref, paired_est = sync.associate_trajectories(
            ref, est, max_diff=thresholds["max_pair_dt_s"]
        )
    except sync.SyncException:
        return (
            [Check("ATE", "mapping", "insufficient", reason="轨迹没有可配对时间戳")],
            details,
            None,
        )
    ref_coverage, est_coverage = (
        paired_ref.num_poses / len(reference),
        paired_est.num_poses / len(estimated),
    )
    details.update(
        reference_coverage=ref_coverage,
        estimate_coverage=est_coverage,
        matched_poses=paired_ref.num_poses,
        max_pair_dt_s=thresholds["max_pair_dt_s"],
    )
    if (
        min(ref_coverage, est_coverage) < thresholds["min_pair_coverage"]
        or paired_ref.num_poses < 3
    ):
        return (
            [
                Check(
                    "ATE",
                    "mapping",
                    "insufficient",
                    min(ref_coverage, est_coverage),
                    thresholds["min_pair_coverage"],
                    "配对覆盖不足",
                )
            ],
            details,
            None,
        )
    transform = np.eye(4)
    aligned = deepcopy(est)
    if alignment == "se3":
        try:
            rotation, translation, _ = paired_est.align(paired_ref, correct_scale=False)
        except Exception as error:
            return (
                [Check("ATE", "mapping", "insufficient", reason=f"无法求解刚体对齐: {error}")],
                details,
                None,
            )
        transform[:3, :3], transform[:3, 3] = rotation, translation
        aligned.transform(transform)
    elif alignment != "none":
        raise ValueError("Only se3 or none alignment is allowed; metric sensors retain scale error")
    details["transform_estimate_to_reference"] = transform.tolist()
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((paired_ref, paired_est))
    rmse = float(ape.get_statistic(metrics.StatisticsType.rmse))
    worst = int(np.argmax(ape.error))
    checks.append(
        Check(
            "ATE",
            "mapping",
            "pass" if rmse <= thresholds["ate_rmse_m"] else "fail",
            rmse,
            thresholds["ate_rmse_m"],
            "evo APE translation RMSE，刚体对齐，不修正尺度",
            [float(paired_ref.timestamps[worst])],
            ["estimated.tum", "reference.tum"],
        )
    )
    rpe = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=Unit.frames)
    rpe.process_data((paired_ref, paired_est))
    details.update(
        ate_rmse_m=rmse,
        ate_max_m=float(np.max(ape.error)),
        rpe_rmse_m=float(rpe.get_statistic(metrics.StatisticsType.rmse)),
        error_times=paired_ref.timestamps.tolist(),
        errors_m=ape.error.tolist(),
    )
    return checks, details, aligned.positions_xyz[:, :2]


def health_checks(poses, grid, thresholds):
    if len(poses) < 2 or not np.isfinite(poses).all():
        return [Check("TRAJECTORY", "mapping", "fail", reason="轨迹不足或存在非有限值")]
    gap = np.diff(poses[:, 0])
    i = int(np.argmax(gap))
    known = int(np.count_nonzero(grid.cells >= 0))
    return [
        Check(
            "SCAN_GAP",
            "mapping",
            "pass" if 0 < gap.min() and gap[i] <= thresholds["max_scan_gap_s"] else "fail",
            float(gap[i]),
            thresholds["max_scan_gap_s"],
            "相邻估计位姿／扫描时间间隔",
            poses[i : i + 2, 0].tolist(),
            ["estimated.tum"],
        ),
        Check(
            "MAP_OUTPUT",
            "mapping",
            "pass" if known >= thresholds["min_known_cells"] else "fail",
            known,
            thresholds["min_known_cells"],
            "可加载地图的已知栅格数",
            evidence=["map.yaml", "map.pgm"],
        ),
    ]


def world_coordinates(grid):
    yy, xx = np.indices(grid.cells.shape)
    local = np.stack(
        ((xx.ravel() + 0.5) * grid.resolution, (yy.ravel() + 0.5) * grid.resolution), axis=1
    )
    x, y, yaw = grid.origin
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    return local @ rotation.T + [x, y]


def sample_grid(grid, points):
    x, y, yaw = grid.origin
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    local = (points - [x, y]) @ rotation
    indices = np.floor(local / grid.resolution).astype(int)
    valid = (indices[:, 0] >= 0) & (indices[:, 1] >= 0)
    valid &= (indices[:, 0] < grid.cells.shape[1]) & (indices[:, 1] < grid.cells.shape[0])
    result = np.full(len(points), -1, np.int8)
    result[valid] = grid.cells[indices[valid, 1], indices[valid, 0]]
    return result


def map_checks(generated, reference, thresholds):
    coords = world_coordinates(reference)
    truth = reference.cells.ravel()
    observed = sample_grid(generated, coords)
    valid = truth >= 0
    occupied = truth == 100
    if not valid.any() or not occupied.any():
        return (
            [
                Check(
                    "MAP_REFERENCE",
                    "mapping",
                    "insufficient",
                    reason="参考地图没有可评价区域／墙体",
                )
            ],
            {},
            observed,
        )
    coverage = float(np.mean(observed[valid] >= 0))
    false_free = float(np.mean(observed[occupied] == 0))
    a, b = coords[occupied], coords[(observed == 100) & valid]
    wall = (
        None
        if len(b) == 0
        else float(np.percentile(np.r_[cKDTree(a).query(b)[0], cKDTree(b).query(a)[0]], 95))
    )
    scores = {
        "coverage": coverage,
        "false_free_rate": false_free,
        "wall_p95_m": wall,
        "alignment": "declared map origins; no fitted map alignment",
        "sample_resolution_m": reference.resolution,
        "observable_cells": int(valid.sum()),
        "reference_origin": reference.origin,
        "generated_origin": generated.origin,
    }
    checks = [
        Check(
            "MAP_COVERAGE",
            "mapping",
            "pass" if coverage >= thresholds["map_coverage"] else "fail",
            coverage,
            thresholds["map_coverage"],
            evidence=["map.yaml", "reference-map.yaml"],
        ),
        Check(
            "FALSE_FREE",
            "mapping",
            "pass" if false_free <= thresholds["false_free_rate"] else "fail",
            false_free,
            thresholds["false_free_rate"],
            "未知区域不计作空闲区域",
        ),
    ]
    checks.append(
        Check(
            "WALL_P95",
            "mapping",
            "insufficient"
            if wall is None
            else ("pass" if wall <= thresholds["wall_p95_m"] else "fail"),
            wall,
            thresholds["wall_p95_m"],
            "双向墙体距离 95 分位；固定参考可观测范围",
        )
    )
    return checks, scores, observed


def navigation_checks(record, thresholds, section):
    """Fail closed: action status alone, missing contact stream, stale pose never pass."""
    prefix = record["name"]
    required = (
        "action_status",
        "final_pose",
        "goal",
        "contacts",
        "contact_samples",
        "duration_s",
        "timeout_s",
        "final_speed",
        "pose_age_s",
        "map_sha256",
        "contact_coverage",
        "contact_max_gap_s",
        "contact_age_s",
        "collision_geometries_observed",
    )
    missing = [key for key in required if record.get(key) is None]
    if missing:
        return [Check(prefix, section, "insufficient", reason=f"缺失证据: {', '.join(missing)}")]
    if (
        record["contact_samples"] <= 0
        or record["pose_age_s"] > 0.5
        or record["contact_coverage"] < 0.95
        or record["contact_max_gap_s"] > 0.5
        or record["contact_age_s"] > 0.5
        or record["collision_geometries_observed"] != 4
    ):
        return [
            Check(
                prefix,
                section,
                "insufficient",
                reason="碰撞观测未覆盖全程／全部碰撞体，或终点真值过期",
            )
        ]
    pose, goal = record["final_pose"], record["goal"]
    distance = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    yaw = abs(math.atan2(math.sin(pose[2] - goal[2]), math.cos(pose[2] - goal[2])))
    expected_failure = record.get("expected", "succeeded") == "controlled_failure"
    success = (
        record["action_status"] == "succeeded"
        and distance <= thresholds["position_m"]
        and yaw <= thresholds["yaw_rad"]
    )
    if expected_failure:
        success = record["action_status"] in ("aborted", "canceled")
        success &= record["final_speed"] <= thresholds["stop_speed_mps"]
    success &= record["duration_s"] <= record["timeout_s"]
    success &= record["contacts"] == 0
    if record.get("temporary_block"):
        success &= record.get("block_observed", False) and record.get("block_removed", False)
        success &= record.get("resumed_after_removal", False)
    value = dict(
        position_m=distance,
        yaw_rad=yaw,
        contacts=record["contacts"],
        action_status=record["action_status"],
        duration_s=record["duration_s"],
        final_speed=record["final_speed"],
    )
    limits = {k: thresholds[k] for k in ("position_m", "yaw_rad", "stop_speed_mps")}
    limits.update(contacts=0, timeout_s=record["timeout_s"])
    return [
        Check(
            prefix,
            section,
            "pass" if success else "fail",
            value,
            limits,
            "动作结果＋独立终点位姿＋接触采样",
            record.get("window_s", []),
            [record.get("evidence", "navigation.json")],
        )
    ]
