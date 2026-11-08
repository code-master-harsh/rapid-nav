"""
Check the RAPID-NAV simulation in one command and write a bundle.

Start the simulation first (ros2 launch rapid_nav_bringup sim.launch.py), then run:

    ros2 run rapid_nav_tools sim_check [--out DIR] [--duration S] [--profile native|docker]
                                       [--odom-tf ekf|wheel] [--no-drive]

It measures the topic rates, checks the content of the camera, depth, IMU and lidar, checks the
TF tree and, unless --no-drive is given, drives the robot 1.2 m forward and then turns it 180
degrees in place, comparing the odometry with the simulator's ground truth. The bundle in DIR
holds environment.txt, results.json, summary.md, tf_frames.yaml, rgb.png, depth.png and scan.png.
The summary table is printed as well. The exit status is 0 when no check fails, else 1.
"""

import argparse
import datetime
import json
import math
import os
import platform
import shutil
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from PIL import Image
from rapid_nav_tools.checks import (
    angle_diff,
    colourise_depth,
    expected_fx,
    planar_delta,
    quaternion_from_rpy,
    rate_verdict,
    scan_image,
    yaw_from_quaternion,
)
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Imu, JointState, LaserScan
from sensor_msgs.msg import Image as ImageMsg
import tf2_ros

EXPECTED_RATES = {
    '/odom': 50.0,
    '/odometry/filtered': 50.0,
    '/joint_states': 30.0,
    '/imu': 200.0,
    '/scan': 10.0,
    '/camera/image': 30.0,
    '/camera/depth_image': 30.0,
    '/camera/camera_info': 30.0,
    '/ground_truth/odom': 50.0,
}
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_HFOV = 1.211
CLOCK_WAIT_S = 60.0
TF_SETTLE_S = 2.0
CMD_PERIOD_S = 0.05
DRIVE_SPEED = 0.3
DRIVE_TIME = 4.0
STOP_TIME = 1.0
TURN_RATE = 0.6
TURN_TIME = 5.236
SCAN_IMAGE_PX = 500
SCAN_RANGE_M = 6.0
DEPTH_NEAR_M = 0.1
DEPTH_FAR_M = 10.0
TF_TOLERANCE = 1e-3

# (target frame, source frame, expected translation, expected quaternion)
TF_EXPECTED = [
    ('base_footprint', 'base_link', (0.0, 0.0, 0.05), (0.0, 0.0, 0.0, 1.0)),
    ('base_link', 'camera_link', (0.175, 0.0, 0.175), (0.0, 0.0, 0.0, 1.0)),
    ('camera_link', 'camera_optical_frame', (0.0, 0.0, 0.0),
     quaternion_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)),
    ('base_link', 'imu_link', (0.0, 0.0, 0.10), (0.0, 0.0, 0.0, 1.0)),
    ('base_link', 'lidar_link', (0.0, 0.0, 0.225), (0.0, 0.0, 0.0, 1.0)),
    ('base_link', 'left_wheel_link', (0.0, 0.225, 0.0), (0.0, 0.0, 0.0, 1.0)),
]


def stamp_seconds(stamp):
    """Return a builtin_interfaces stamp in seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def verdict_of(ok):
    """Return PASS when ok, else FAIL."""
    return 'PASS' if ok else 'FAIL'


def run_text(command):
    """Run a command and return its first output line, or a short note when it cannot run."""
    if shutil.which(command[0]) is None:
        return 'not available'
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=20, check=False)
    except (OSError, subprocess.SubprocessError) as error:
        return 'error: {}'.format(error)
    lines = (result.stdout or result.stderr).strip().splitlines()
    if result.returncode != 0 or not lines:
        return 'not available'
    return lines[0]


def ram_text():
    """Return the total RAM from /proc/meminfo, or a note when it is not available."""
    try:
        with open('/proc/meminfo', encoding='utf-8') as handle:
            for line in handle:
                if line.startswith('MemTotal:'):
                    gibibytes = float(line.split()[1]) / 1024.0 / 1024.0
                    return '{:.1f} GiB'.format(gibibytes)
    except OSError:
        pass
    return 'not available'


def git_text():
    """Return the short commit and the branch when the current folder is in the repository."""
    commit = run_text(['git', 'rev-parse', '--short', 'HEAD'])
    if commit == 'not available':
        return 'not in a git repository'
    return '{} on {}'.format(commit, run_text(['git', 'branch', '--show-current']))


def environment_lines(argv):
    """Return the lines of environment.txt."""
    return [
        'date: ' + datetime.datetime.now().isoformat(timespec='seconds'),
        'ROS_DISTRO: ' + os.environ.get('ROS_DISTRO', 'unset'),
        'gz sim: ' + run_text(['gz', 'sim', '--versions']),
        'platform: ' + platform.platform(),
        'CPUs: {}'.format(os.cpu_count()),
        'RAM: ' + ram_text(),
        'GPU: ' + run_text(['nvidia-smi', '--query-gpu=name,memory.total',
                            '--format=csv,noheader']),
        'git: ' + git_text(),
        'arguments: ' + ' '.join(argv),
    ]


def format_number(value, digits=3):
    """Format a number for the table, or a dash when it is missing."""
    if value is None:
        return '-'
    return '{:.{}f}'.format(value, digits)


class SimCheck(Node):
    """Subscribe to the simulation topics, measure them, read the TF tree and drive the robot."""

    def __init__(self, odom_tf, profile, out_dir):
        """Create the subscriptions, the command publisher and the TF listener."""
        super().__init__(
            'sim_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.odom_tf = odom_tf
        self.profile = profile
        self.out_dir = out_dir
        self.values = {}
        self.topics = {
            '/clock': Clock,
            '/odom': Odometry,
            '/joint_states': JointState,
            '/imu': Imu,
            '/scan': LaserScan,
            '/camera/image': ImageMsg,
            '/camera/depth_image': ImageMsg,
            '/camera/camera_info': CameraInfo,
            '/ground_truth/odom': Odometry,
        }
        if odom_tf == 'ekf':
            self.topics['/odometry/filtered'] = Odometry
        self.latest = {}
        self._reset()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscriptions_kept = [
            self.create_subscription(msg_type, topic, self._recorder(topic), qos)
            for topic, msg_type in self.topics.items()
        ]
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _reset(self):
        """Forget the measurements of the previous window; the latest messages are kept."""
        self.stamps = {topic: [] for topic in self.topics}
        self.imu = []
        self.clock_pairs = []

    def _recorder(self, topic):
        """Return the callback that stores the stamp and the latest message of a topic."""
        def record(msg):
            if topic == '/clock':
                stamp = stamp_seconds(msg.clock)
                self.clock_pairs.append((stamp, time.monotonic()))
            else:
                stamp = stamp_seconds(msg.header.stamp)
            self.stamps[topic].append(stamp)
            self.latest[topic] = msg
            if topic == '/imu':
                gyro = msg.angular_velocity
                self.imu.append((stamp, gyro.x, gyro.y, gyro.z, msg.linear_acceleration.z))
        return record

    def spin_for(self, seconds):
        """Process callbacks for the given wall-clock time."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_now(self):
        """Return the simulated time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_for_clock(self, timeout):
        """Return True when /clock arrives within the wall-clock timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.stamps['/clock']:
                return True
        return False

    def measure_rates(self, duration):
        """Count the messages for the window; return the rate table, the real-time factor, IMU."""
        self._reset()
        self.spin_for(duration)
        rates = {}
        for topic, expected in EXPECTED_RATES.items():
            if topic not in self.topics:
                continue
            stamps = self.stamps[topic]
            count = len(stamps)
            span = stamps[-1] - stamps[0] if count >= 2 else 0.0
            rate = (count - 1) / span if span > 0.0 else 0.0
            rates[topic] = {
                'count': count,
                'rate': rate,
                'expected': expected,
                'verdict': rate_verdict(count, rate, expected, self.profile),
            }
        rtf = 0.0
        if len(self.clock_pairs) > 1:
            sim_span = self.clock_pairs[-1][0] - self.clock_pairs[0][0]
            wall_span = self.clock_pairs[-1][1] - self.clock_pairs[0][1]
            if wall_span > 0.0:
                rtf = sim_span / wall_span
        return rates, rtf

    def pose_of(self, topic):
        """Return (x, y, yaw) of the latest odometry message of a topic, or None."""
        message = self.latest.get(topic)
        if message is None:
            return None
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        return (position.x, position.y, yaw)

    def publish_command(self, linear, angular):
        """Publish one velocity command."""
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.cmd_pub.publish(message)

    def run_for_sim(self, linear, angular, seconds):
        """Send the velocity at 20 Hz of simulated time for the given simulated seconds."""
        start = self.sim_now()
        last_sent = None
        while self.sim_now() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = self.sim_now()
            if last_sent is None or now - last_sent >= CMD_PERIOD_S:
                self.publish_command(linear, angular)
                last_sent = now
        self.publish_command(0.0, 0.0)

    def lookup(self, target, source):
        """Return the transform of source in target, or None when it is not available."""
        try:
            return self.tf_buffer.lookup_transform(target, source, Time())
        except tf2_ros.TransformException:
            return None


class Report:
    """Collect the check rows."""

    def __init__(self):
        """Start with no rows."""
        self.rows = []

    def add(self, name, value, expected, verdict):
        """Add one check row."""
        self.rows.append({'check': name, 'value': value, 'expected': expected, 'verdict': verdict})

    def count(self, verdict):
        """Return how many rows have the given verdict."""
        return sum(1 for row in self.rows if row['verdict'] == verdict)


def check_sensors(node, report):
    """Check the content of the camera, depth, IMU, lidar and odometry messages."""
    check_camera_info(node, report)
    check_rgb(node, report)
    check_depth(node, report)
    check_imu(node, report)
    check_scan(node, report)
    odometry_topics = ['/odom'] + (['/odometry/filtered'] if node.odom_tf == 'ekf' else [])
    for topic in odometry_topics:
        message = node.latest.get(topic)
        name = '{} frames'.format(topic)
        if message is None:
            report.add(name, 'none', 'odom / base_footprint', 'FAIL')
            continue
        frames = (message.header.frame_id, message.child_frame_id)
        report.add(name, '{} / {}'.format(*frames), 'odom / base_footprint',
                   verdict_of(frames == ('odom', 'base_footprint')))


def check_camera_info(node, report):
    """Check the camera_info size, focal length, principal point and frame."""
    info = node.latest.get('/camera/camera_info')
    if info is None:
        report.add('camera_info received', 'none', 'one message', 'FAIL')
        return
    width, height = info.width, info.height
    report.add('camera_info size', '{}x{}'.format(width, height),
               '{}x{}'.format(CAMERA_WIDTH, CAMERA_HEIGHT),
               verdict_of((width, height) == (CAMERA_WIDTH, CAMERA_HEIGHT)))
    fx_expected = expected_fx(width, CAMERA_HFOV)
    fx, fy = info.k[0], info.k[4]
    report.add('camera fx', format_number(fx, 2), format_number(fx_expected, 2),
               verdict_of(abs(fx - fx_expected) <= 0.01 * fx_expected))
    report.add('camera fy', format_number(fy, 2), format_number(fx_expected, 2),
               verdict_of(abs(fy - fx_expected) <= 0.01 * fx_expected))
    cx, cy = info.k[2], info.k[5]
    report.add('camera cx', format_number(cx, 2), format_number(width / 2.0, 1),
               verdict_of(abs(cx - width / 2.0) <= 1.0))
    report.add('camera cy', format_number(cy, 2), format_number(height / 2.0, 1),
               verdict_of(abs(cy - height / 2.0) <= 1.0))
    report.add('camera_info frame', info.header.frame_id, 'camera_optical_frame',
               verdict_of(info.header.frame_id == 'camera_optical_frame'))


def check_rgb(node, report):
    """Check the colour image and save it as rgb.png."""
    image = node.latest.get('/camera/image')
    if image is None:
        report.add('rgb image received', 'none', 'one message', 'FAIL')
        return
    rows = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(image.height, image.step)
    rgb = rows[:, :image.width * 3].reshape(image.height, image.width, 3)
    mean = float(rgb.mean())
    spread = float(rgb.std())
    report.add('rgb encoding', image.encoding, 'rgb8', verdict_of(image.encoding == 'rgb8'))
    report.add('rgb frame', image.header.frame_id, 'camera_optical_frame',
               verdict_of(image.header.frame_id == 'camera_optical_frame'))
    report.add('rgb mean', format_number(mean, 1), '20 to 235', verdict_of(20.0 <= mean <= 235.0))
    report.add('rgb std', format_number(spread, 1), '> 10', verdict_of(spread > 10.0))
    Image.fromarray(rgb).save(os.path.join(node.out_dir, 'rgb.png'))


def check_depth(node, report):
    """Check the depth image and save it as depth.png."""
    message = node.latest.get('/camera/depth_image')
    if message is None:
        report.add('depth image received', 'none', 'one message', 'FAIL')
        return
    values = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(
        message.height, message.step // 4)[:, :message.width]
    finite = values[np.isfinite(values)]
    fraction = finite.size / float(values.size)
    median = float(np.median(finite)) if finite.size else float('nan')
    report.add('depth encoding', message.encoding, '32FC1',
               verdict_of(message.encoding == '32FC1'))
    report.add('depth finite fraction', format_number(fraction, 3), '> 0.3',
               verdict_of(fraction > 0.3))
    report.add('depth median (m)', format_number(median, 3), '0.1 to 10',
               verdict_of(0.1 <= median <= 10.0))
    picture = colourise_depth(values, DEPTH_NEAR_M, DEPTH_FAR_M)
    Image.fromarray(picture).save(os.path.join(node.out_dir, 'depth.png'))


def check_imu(node, report):
    """Check the IMU frame, its gravity reading and that it is at rest."""
    if not node.imu:
        report.add('imu received', 'none', 'messages', 'FAIL')
        return
    message = node.latest['/imu']
    acc_z = float(np.mean([sample[4] for sample in node.imu]))
    gyro = float(np.mean([math.sqrt(s[1] ** 2 + s[2] ** 2 + s[3] ** 2) for s in node.imu]))
    report.add('imu frame', message.header.frame_id, 'imu_link',
               verdict_of(message.header.frame_id == 'imu_link'))
    report.add('imu mean acceleration z (m/s^2)', format_number(acc_z, 3), '9.0 to 10.6',
               verdict_of(9.0 <= acc_z <= 10.6))
    report.add('imu mean |angular velocity| (rad/s)', format_number(gyro, 4), '< 0.02',
               verdict_of(gyro < 0.02))


def check_scan(node, report):
    """Check the lidar scan and save it as scan.png."""
    scan = node.latest.get('/scan')
    if scan is None:
        report.add('scan received', 'none', 'one message', 'FAIL')
        return
    ranges = np.array(scan.ranges, dtype=np.float64)
    finite = ranges[np.isfinite(ranges)]
    fraction = finite.size / float(ranges.size)
    minimum = float(finite.min()) if finite.size else float('nan')
    report.add('scan frame', scan.header.frame_id, 'lidar_link',
               verdict_of(scan.header.frame_id == 'lidar_link'))
    report.add('scan finite fraction', format_number(fraction, 3), '> 0.9',
               verdict_of(fraction > 0.9))
    report.add('scan minimum (m)', format_number(minimum, 3), '> 0.12',
               verdict_of(minimum > 0.12))
    picture = scan_image(ranges, scan.angle_min, scan.angle_increment,
                         SCAN_IMAGE_PX, SCAN_RANGE_M)
    Image.fromarray(picture).save(os.path.join(node.out_dir, 'scan.png'))


def check_tf(node, report):
    """Check the transform tree against the frame table and write tf_frames.yaml."""
    node.spin_for(TF_SETTLE_S)
    for target, source, translation, quaternion in TF_EXPECTED:
        name = 'tf {} -> {}'.format(target, source)
        transform = node.lookup(target, source)
        if transform is None:
            report.add(name, 'missing', 'available', 'FAIL')
            continue
        t = transform.transform.translation
        r = transform.transform.rotation
        measured_t = (t.x, t.y, t.z)
        measured_q = (r.x, r.y, r.z, r.w)
        dot = sum(a * b for a, b in zip(measured_q, quaternion))
        sign = 1.0 if dot >= 0.0 else -1.0
        same_t = all(abs(a - b) <= TF_TOLERANCE for a, b in zip(measured_t, translation))
        same_q = all(abs(sign * a - b) <= TF_TOLERANCE for a, b in zip(measured_q, quaternion))
        report.add(name, '({})'.format(', '.join(format_number(v, 4) for v in measured_t)),
                   '({})'.format(', '.join(format_number(v, 4) for v in translation)),
                   verdict_of(same_t and same_q))
    odom = node.lookup('odom', 'base_footprint')
    if odom is None:
        report.add('tf odom -> base_footprint', 'missing', 'available', 'FAIL')
    else:
        t = odom.transform.translation
        report.add('tf odom -> base_footprint', '({:.3f}, {:.3f}, {:.3f})'.format(t.x, t.y, t.z),
                   'available', 'PASS')
    with open(os.path.join(node.out_dir, 'tf_frames.yaml'), 'w', encoding='utf-8') as handle:
        handle.write(node.tf_buffer.all_frames_as_yaml())


def check_drive(node, report):
    """Drive forward and turn in place, then compare the odometry with the ground truth."""
    node.spin_for(0.5)
    sources = ['/ground_truth/odom', '/odom'] + (
        ['/odometry/filtered'] if node.odom_tf == 'ekf' else [])
    starts = {topic: node.pose_of(topic) for topic in sources}
    if starts['/ground_truth/odom'] is None:
        report.add('drive', 'no ground truth', 'odometry messages', 'FAIL')
        return
    node.run_for_sim(DRIVE_SPEED, 0.0, DRIVE_TIME)
    node.run_for_sim(0.0, 0.0, STOP_TIME)
    turn_start = node.sim_now()
    node.run_for_sim(0.0, TURN_RATE, TURN_TIME)
    node.run_for_sim(0.0, 0.0, STOP_TIME)
    node.spin_for(0.5)
    ends = {topic: node.pose_of(topic) for topic in sources}

    truth_distance, truth_yaw = planar_delta(starts['/ground_truth/odom'],
                                             ends['/ground_truth/odom'])
    node.values['ground_truth_distance_m'] = truth_distance
    node.values['ground_truth_yaw_change_rad'] = truth_yaw
    report.add('ground truth distance (m)', format_number(truth_distance, 3), '1.0 to 1.4',
               verdict_of(1.0 <= truth_distance <= 1.4))
    report.add('ground truth turn (deg)', format_number(math.degrees(abs(truth_yaw)), 1),
               '150 to 180 (yaw wrapped to (-pi, pi])',
               verdict_of(math.radians(150.0) <= abs(truth_yaw) <= math.pi))

    for topic in sources[1:]:
        if starts[topic] is None or ends[topic] is None:
            report.add('{} drive'.format(topic), 'no messages', 'odometry', 'FAIL')
            continue
        distance, yaw_change = planar_delta(starts[topic], ends[topic])
        node.values['{}_distance_m'.format(topic)] = distance
        node.values['{}_yaw_change_rad'.format(topic)] = yaw_change
        yaw_error = angle_diff(yaw_change, truth_yaw)
        report.add('{} distance vs truth'.format(topic), format_number(distance, 3),
                   format_number(truth_distance, 3),
                   verdict_of(abs(distance - truth_distance) <= 0.05 * truth_distance))
        report.add('{} turn vs truth (deg)'.format(topic),
                   format_number(math.degrees(yaw_error), 2), 'within 5',
                   verdict_of(abs(yaw_error) <= math.radians(5.0)))

    window_start = turn_start + (TURN_TIME - 3.0) / 2.0
    window_end = window_start + 3.0
    turn_rates = [s[3] for s in node.imu if window_start <= s[0] <= window_end]
    if turn_rates:
        mean_rate = float(np.mean(turn_rates))
        node.values['imu_mean_yaw_rate_turn_rad_s'] = mean_rate
        report.add('imu mean yaw rate in the turn (rad/s)', format_number(mean_rate, 3),
                   '0.6 +- 0.05', verdict_of(abs(mean_rate - TURN_RATE) <= 0.05))
    else:
        report.add('imu mean yaw rate in the turn (rad/s)', 'none', '0.6 +- 0.05', 'FAIL')


def summary_text(environment, rtf, rows):
    """Return the Markdown summary: environment, real-time factor and the check table."""
    lines = ['# sim_check summary', '']
    lines += ['- ' + line for line in environment]
    lines += ['- real-time factor: {:.3f}'.format(rtf), '']
    lines += ['| check | value | expected | verdict |', '|---|---|---|---|']
    for row in rows:
        lines.append('| {} | {} | {} | {} |'.format(
            row['check'], row['value'], row['expected'], row['verdict']))
    failures = sum(1 for row in rows if row['verdict'] == 'FAIL')
    warnings = sum(1 for row in rows if row['verdict'] == 'WARN')
    lines += ['', 'Result: {} failed, {} warnings, {} checks.'.format(
        failures, warnings, len(rows)), '']
    return '\n'.join(lines)


def write_outputs(out_dir, environment, rtf, rows, rates, values):
    """Write environment.txt, results.json and summary.md into the bundle folder."""
    with open(os.path.join(out_dir, 'environment.txt'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(environment) + '\n')
    results = {
        'environment': environment,
        'real_time_factor': rtf,
        'rates': rates,
        'values': values,
        'checks': rows,
    }
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as handle:
        json.dump(results, handle, indent=2, default=float)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as handle:
        handle.write(summary_text(environment, rtf, rows))


def parse_arguments(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(description='Check the RAPID-NAV simulation.')
    parser.add_argument('--out', default=None,
                        help='bundle folder (default: sim_check_<time> in the current folder)')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='wall-clock seconds of the rate window (default 10)')
    parser.add_argument('--profile', choices=['native', 'docker'], default='native',
                        help='native: PASS at 80 %% of the expected rate; docker: presence only')
    parser.add_argument('--odom-tf', choices=['ekf', 'wheel'], default='ekf',
                        help='launch mode of the simulation (default ekf)')
    parser.add_argument('--no-drive', action='store_true', help='skip the scripted drive')
    return parser.parse_args(argv)


def main(argv=None):
    """Run every check, write the bundle and return the exit status."""
    raw_argv = sys.argv[1:] if argv is None else list(argv)
    args = parse_arguments(raw_argv)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    out_dir = args.out or os.path.join(os.getcwd(), 'sim_check_' + stamp)
    os.makedirs(out_dir, exist_ok=True)
    environment = environment_lines(raw_argv)
    report = Report()
    rates = {}
    rtf = 0.0
    values = {}

    rclpy.init()
    node = None
    try:
        node = SimCheck(args.odom_tf, args.profile, out_dir)
        if not node.wait_for_clock(CLOCK_WAIT_S):
            report.add('/clock', 'none', 'messages within {:.0f} s'.format(CLOCK_WAIT_S), 'FAIL')
        else:
            rates, rtf = node.measure_rates(args.duration)
            for topic, row in rates.items():
                report.add('rate {}'.format(topic),
                           '{:.1f} Hz ({} msgs)'.format(row['rate'], row['count']),
                           '{:.0f} Hz'.format(row['expected']), row['verdict'])
            check_sensors(node, report)
            check_tf(node, report)
            if args.no_drive:
                report.add('drive', 'skipped', '-', 'PASS')
            else:
                check_drive(node, report)
            values = node.values
    except KeyboardInterrupt:
        report.add('run', 'interrupted', 'complete', 'FAIL')
    finally:
        if node is not None:
            for _ in range(5):
                node.publish_command(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.01)
            node.destroy_node()
        rclpy.shutdown()

    write_outputs(out_dir, environment, rtf, report.rows, rates, values)
    print(summary_text(environment, rtf, report.rows))
    print('bundle: ' + out_dir)
    return 0 if report.count('FAIL') == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
