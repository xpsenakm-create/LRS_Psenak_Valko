"""Shared helpers for the A2 bag tools (a2_plot.py, a2_characterize.py): read a ROS 2 bag into
NumPy arrays.  Offline only -- ground truth is read here, in a separate process, never by the
estimator or the controller."""

import math
import sys

import numpy as np


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def read_bag(path, topics):
    """Return {topic: [(receive_time_s, msg), ...]} for the requested topics present in the bag."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        sys.exit(f'Chyba: treba source /opt/ros/<distro>/setup.bash ({e})')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    out = {t: [] for t in wanted}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(data, classes[topic])))
    return out


def odometry_arrays(items):
    """nav_msgs/Odometry list -> dict of arrays (stamp, position, yaw, twist)."""
    if not items:
        return None
    m = [x[1] for x in items]
    return {
        't': np.array([stamp(o.header) for o in m]),
        'recv': np.array([x[0] for x in items]),
        'p': np.array([[o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z] for o in m]),
        'yaw': np.array([yaw_of(o.pose.pose.orientation) for o in m]),
        'v': np.array([[o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z] for o in m]),
        'wz': np.array([o.twist.twist.angular.z for o in m]),
        'frame': m[0].header.frame_id,
        'child': m[0].child_frame_id,
    }


def truth_arrays(items):
    """Ground truth with velocity and yaw rate DIFFERENTIATED from the pose (frame-agnostic)."""
    tr = odometry_arrays(items)
    if tr is None:
        return None
    keep = np.concatenate([[True], np.diff(tr['t']) > 0])
    for k in ('t', 'recv', 'p', 'yaw'):
        tr[k] = tr[k][keep]
    tr['yaw_u'] = np.unwrap(tr['yaw'])
    tr['v_enu'] = np.gradient(tr['p'], tr['t'], axis=0)
    tr['wz'] = np.gradient(tr['yaw_u'], tr['t'])
    return tr


def interp_truth(truth, t):
    """Truth position, unwrapped yaw, ENU velocity and yaw rate at times t."""
    p = np.stack([np.interp(t, truth['t'], truth['p'][:, i]) for i in range(3)], axis=1)
    yaw = np.interp(t, truth['t'], truth['yaw_u'])
    v = np.stack([np.interp(t, truth['t'], truth['v_enu'][:, i]) for i in range(3)], axis=1)
    wz = np.interp(t, truth['t'], truth['wz'])
    return p, yaw, v, wz


def wall_to_sim(odom):
    """Map bag receive time -> simulation time (for unstamped topics like /mission/state)."""
    return lambda t_wall: np.interp(t_wall, odom['recv'], odom['t'])


def state_spans(items, t_end):
    """/mission/state messages (receive time) -> [(name, t0, t1)]."""
    spans = []
    for i, (t, m) in enumerate(items):
        t1 = items[i + 1][0] if i + 1 < len(items) else t_end
        spans.append((m.data, t, t1))
    return spans
