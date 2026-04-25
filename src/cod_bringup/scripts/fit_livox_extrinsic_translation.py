#!/usr/bin/env python3

import argparse
import math
import pathlib
import re
import sys
from dataclasses import dataclass

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass
class PoseSample:
    stamp: float
    position: np.ndarray
    rotation: np.ndarray


def quaternion_to_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("zero-length quaternion")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def matrix_from_euler(roll, pitch, yaw):
    return quaternion_to_matrix(*quaternion_from_euler(roll, pitch, yaw))


def yaw_from_matrix(rotation):
    return math.atan2(rotation[1, 0], rotation[0, 0])


def detect_storage_id(uri):
    metadata_path = uri / "metadata.yaml" if uri.is_dir() else uri.parent / "metadata.yaml"
    if metadata_path.exists():
        text = metadata_path.read_text(errors="ignore")
        match = re.search(r"storage_identifier:\s*([A-Za-z0-9_]+)", text)
        if match:
            return match.group(1)
        match = re.search(r"storage_id:\s*([A-Za-z0-9_]+)", text)
        if match:
            return match.group(1)
    if uri.is_file():
        if uri.suffix == ".mcap":
            return "mcap"
        if uri.suffix == ".db3":
            return "sqlite3"
    if any(uri.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def normalize_uri(path):
    uri = pathlib.Path(path).expanduser().resolve()
    if uri.is_file() and (uri.parent / "metadata.yaml").exists():
        return uri.parent
    return uri


def read_odom_samples(bag_path, topic):
    uri = normalize_uri(bag_path)
    if not uri.exists():
        raise FileNotFoundError(str(uri))
    storage_id = detect_storage_id(uri)
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = "\n".join(sorted(topic_types))
        raise RuntimeError(f"topic {topic} not found. Available topics:\n{available}")
    msg_type = get_message(topic_types[topic])
    samples = []
    while reader.has_next():
        topic_name, data, bag_timestamp = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        pose = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0:
            stamp = bag_timestamp * 1e-9
        position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        rotation = quaternion_to_matrix(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        samples.append(PoseSample(stamp=stamp, position=position, rotation=rotation))
    return samples, uri, storage_id


def filter_samples(samples, start_sec, end_sec, max_samples):
    if not samples:
        return []
    t0 = samples[0].stamp
    filtered = []
    for sample in samples:
        elapsed = sample.stamp - t0
        if start_sec is not None and elapsed < start_sec:
            continue
        if end_sec is not None and elapsed > end_sec:
            continue
        filtered.append(sample)
    if max_samples > 0 and len(filtered) > max_samples:
        indices = np.linspace(0, len(filtered) - 1, max_samples).astype(int)
        filtered = [filtered[index] for index in indices]
    return filtered


def pose_model_terms(sample, rotation_base_lidar, model):
    if model == "physical":
        return sample.position, -sample.rotation @ rotation_base_lidar.T
    if model == "cpp":
        rotation_base_odom_base = rotation_base_lidar @ sample.rotation @ rotation_base_lidar.T
        return rotation_base_lidar @ sample.position, np.eye(3) - rotation_base_odom_base
    raise ValueError(f"unsupported model: {model}")


def base_positions(samples, rotation_base_lidar, translation_base_lidar, model):
    positions = []
    for sample in samples:
        offset, translation_matrix = pose_model_terms(sample, rotation_base_lidar, model)
        positions.append(offset + translation_matrix @ translation_base_lidar)
    return np.array(positions, dtype=float)


def metrics(samples, rotation_base_lidar, translation_base_lidar, model):
    positions = base_positions(samples, rotation_base_lidar, translation_base_lidar, model)
    center = np.mean(positions, axis=0)
    delta = positions - center
    xy_error = np.linalg.norm(delta[:, :2], axis=1)
    ranges = np.ptp(positions, axis=0)
    return {
        "rms_xy": float(math.sqrt(np.mean(xy_error * xy_error))),
        "max_xy": float(np.max(xy_error)),
        "range_x": float(ranges[0]),
        "range_y": float(ranges[1]),
        "range_z": float(ranges[2]),
        "center": center,
    }


def fit_translation(samples, rotation_base_lidar, current_translation, fit_mode, free_z, model):
    dimensions = [0, 1] if fit_mode == "xy" else [0, 1, 2]
    variable_indices = [0, 1, 2] if free_z else [0, 1]
    fixed_indices = [] if free_z else [2]
    row_count = len(samples) * len(dimensions)
    column_count = len(variable_indices) + len(dimensions)
    a = np.zeros((row_count, column_count), dtype=float)
    b = np.zeros(row_count, dtype=float)
    row = 0
    for sample in samples:
        offset, translation_matrix = pose_model_terms(sample, rotation_base_lidar, model)
        for center_column, dimension in enumerate(dimensions):
            for variable_column, variable_index in enumerate(variable_indices):
                a[row, variable_column] = translation_matrix[dimension, variable_index]
            a[row, len(variable_indices) + center_column] = -1.0
            fixed_value = 0.0
            for fixed_index in fixed_indices:
                fixed_value += translation_matrix[dimension, fixed_index] * current_translation[fixed_index]
            b[row] = -offset[dimension] - fixed_value
            row += 1
    solution, _, rank, singular_values = np.linalg.lstsq(a, b, rcond=None)
    fitted = current_translation.copy()
    for variable_column, variable_index in enumerate(variable_indices):
        fitted[variable_index] = solution[variable_column]
    condition = float("inf")
    if len(singular_values) > 0 and singular_values[-1] > 0.0:
        condition = float(singular_values[0] / singular_values[-1])
    return fitted, int(rank), condition


def trim_samples(samples, rotation_base_lidar, translation_base_lidar, fit_mode, trim_ratio, model):
    if trim_ratio <= 0.0:
        return samples
    keep_count = int(round(len(samples) * (1.0 - trim_ratio)))
    keep_count = max(3, min(len(samples), keep_count))
    dimensions = [0, 1] if fit_mode == "xy" else [0, 1, 2]
    positions = base_positions(samples, rotation_base_lidar, translation_base_lidar, model)
    center = np.mean(positions, axis=0)
    errors = np.linalg.norm((positions - center)[:, dimensions], axis=1)
    keep_indices = np.argsort(errors)[:keep_count]
    keep_indices.sort()
    return [samples[index] for index in keep_indices]


def print_metric_block(title, value):
    print(title)
    print(f"  xy RMS: {value['rms_xy']:.4f} m")
    print(f"  xy max: {value['max_xy']:.4f} m")
    print(f"  range:  x={value['range_x']:.4f} m, y={value['range_y']:.4f} m, z={value['range_z']:.4f} m")


def parse_args():
    parser = argparse.ArgumentParser(description="Fit effective base_link -> livox_frame translation from an in-place rotation bag.")
    parser.add_argument("bag", help="ROS 2 bag directory, .mcap file, or .db3 file")
    parser.add_argument("--topic", default="/lidar_odom_raw")
    parser.add_argument("--x", type=float, default=0.22679)
    parser.add_argument("--y", type=float, default=0.06741)
    parser.add_argument("--z", type=float, default=0.41959)
    parser.add_argument("--roll", type=float, default=-1.071025)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=1.789491)
    parser.add_argument("--fit-mode", choices=["xy", "xyz"], default="xy")
    parser.add_argument("--model", choices=["cpp", "physical"], default="cpp")
    parser.add_argument("--free-z", action="store_true")
    parser.add_argument("--start-sec", type=float, default=None)
    parser.add_argument("--end-sec", type=float, default=None)
    parser.add_argument("--max-samples", type=int, default=5000)
    parser.add_argument("--trim-ratio", type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.trim_ratio < 0.0 or args.trim_ratio >= 0.8:
        print("--trim-ratio must be in [0.0, 0.8)", file=sys.stderr)
        return 2
    samples, uri, storage_id = read_odom_samples(args.bag, args.topic)
    samples = filter_samples(samples, args.start_sec, args.end_sec, args.max_samples)
    if len(samples) < 3:
        print("not enough odometry samples after filtering", file=sys.stderr)
        return 2
    rotation_base_lidar = matrix_from_euler(args.roll, args.pitch, args.yaw)
    current_translation = np.array([args.x, args.y, args.z], dtype=float)
    fitted_translation, rank, condition = fit_translation(
        samples,
        rotation_base_lidar,
        current_translation,
        args.fit_mode,
        args.free_z,
        args.model,
    )
    trimmed_count = 0
    if args.trim_ratio > 0.0:
        trimmed_samples = trim_samples(samples, rotation_base_lidar, fitted_translation, args.fit_mode, args.trim_ratio, args.model)
        trimmed_count = len(samples) - len(trimmed_samples)
        samples = trimmed_samples
        fitted_translation, rank, condition = fit_translation(
            samples,
            rotation_base_lidar,
            current_translation,
            args.fit_mode,
            args.free_z,
            args.model,
        )
    current_metrics = metrics(samples, rotation_base_lidar, current_translation, args.model)
    fitted_metrics = metrics(samples, rotation_base_lidar, fitted_translation, args.model)
    yaws = np.unwrap(np.array([yaw_from_matrix(sample.rotation) for sample in samples], dtype=float))
    duration = samples[-1].stamp - samples[0].stamp
    print(f"bag: {uri}")
    print(f"storage: {storage_id}")
    print(f"topic: {args.topic}")
    print(f"samples: {len(samples)}")
    if trimmed_count > 0:
        print(f"trimmed: {trimmed_count}")
    print(f"duration: {duration:.3f} s")
    print(f"raw yaw span: {math.degrees(float(np.max(yaws) - np.min(yaws))):.2f} deg")
    print(f"model: {args.model}")
    print(f"fit mode: {args.fit_mode}, z: {'free' if args.free_z else 'locked'}")
    print(f"least-squares rank: {rank}, condition: {condition:.2e}")
    print()
    print_metric_block("current translation", current_metrics)
    print()
    print_metric_block("fitted translation", fitted_metrics)
    print()
    print("suggested base_link -> livox_frame parameters")
    print(f"  x: {fitted_translation[0]:.6f}")
    print(f"  y: {fitted_translation[1]:.6f}")
    print(f"  z: {fitted_translation[2]:.6f}")
    print(f"  roll: {args.roll:.6f}")
    print(f"  pitch: {args.pitch:.6f}")
    print(f"  yaw: {args.yaw:.6f}")
    print()
    print("launch snippet")
    print(f'  "x": {fitted_translation[0]:.6f},')
    print(f'  "y": {fitted_translation[1]:.6f},')
    print(f'  "z": {fitted_translation[2]:.6f},')
    print(f'  "roll": {args.roll:.6f},')
    print(f'  "pitch": {args.pitch:.6f},')
    print(f'  "yaw": {args.yaw:.6f},')
    if math.degrees(float(np.max(yaws) - np.min(yaws))) < 30.0:
        print()
        print("warning: rotation span is small; record a wider left/right in-place rotation for a more stable fit")
    if condition > 1e5:
        print()
        print("warning: fit is ill-conditioned; prefer locked z, longer rotation, or a cleaner in-place segment")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
