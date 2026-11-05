import functools, heapq, math, re
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy as Dur,
                       QoSHistoryPolicy, QoSReliabilityPolicy)
from tf2_ros import TransformException
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
from ardupilot_msgs.srv import ModeSwitch
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

UNKNOWN = -1
N8 = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]

PARAMS = dict(
    map_topic='/map_filtered', costmap_topic='local_costmap/costmap',
    costmap_qos_transient_local=True, global_costmap_topic='/global_costmap/costmap',
    goal_arrival_tolerance_m=0.35, occupied_threshold=65, costmap_lethal_threshold=99,
    min_candidates_before_relax=4, min_frontier_cells=5, safe_distance_m=0.45,
    min_frontier_dist_m=0.35, collision_check_period_sec=0.2, blacklist_radius_m=0.6,
    blacklist_expire_cycles=20, goal_timeout_sec=90.0, replan_period_sec=3.0,
    empty_streak_limit=5, map_qos_transient_local=True, commitment_radius_m=4.0,
    startup_delay_sec=10.0, distance_field_downsample=2, rejection_streak_limit=4,
    min_safe_distance_m=0.32, min_live_clearance_m=0.08, goal_accept_timeout_sec=8.0,
    visit_radius_m=1.0, visit_penalty_m=6.0, visit_memory_cycles=40,
    success_blacklist_cycles=12, direction_weight_m=2.5, final_sweep=True,
    tilt_grace_sec=1.5, tilt_abort_sec=4.0, sensor_range_m=6.0, distance_decay_m=4.5,
    max_safe_tilt_deg=30.0, goal_hard_reset_sec=90.0, idle_recheck_sec=15.0,
    stuck_distance_m=0.35, stuck_timeout_sec=25.0, min_flight_altitude_m=0.5,
    odom_topic='/odometry', odom_frame='odom', robot_radius_m=0.25,
    goal_stall_cycles=3, goal_stall_progress_m=0.15, return_home_on_complete=True,
    home_tolerance_m=0.30, home_goal_timeout_sec=150.0, return_home_max_attempts=6,
    land_on_arrival=True, mode_switch_service='/ap/v1/mode_switch', land_mode_id=9,
    rtl_mode_id=6, landed_altitude_m=0.25)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def secs(a, b):
    return (a - b).nanoseconds / 1e9


def grid_array(grid):
    i = grid.info
    if i.resolution <= 0.0 or i.width == 0 or i.height == 0:
        return None
    data = np.frombuffer(bytearray(grid.data), dtype=np.int8).reshape((i.height, i.width))
    return data, i.resolution, i.origin.position.x, i.origin.position.y


def to_cell(xy, res, ox, oy):
    return int((xy[1] - oy) / res), int((xy[0] - ox) / res)   # (row, col)


def dilate(mask, iterations):
    for _ in range(max(iterations, 0)):
        grown = mask.copy()
        for dy, dx in N8:
            grown |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
        grown[0, :] = grown[-1, :] = grown[:, 0] = grown[:, -1] = False
        mask = grown
    return mask


def nearest_in_window(mask, cy, cx, radius_m, res):
    r = max(1, math.ceil(radius_m / res))
    h, w = mask.shape
    y0, x0 = max(0, cy - r), max(0, cx - r)
    yy, xx = np.nonzero(mask[y0:min(h, cy + r + 1), x0:min(w, cx + r + 1)])
    if yy.size == 0:
        return math.inf
    return float(np.min(np.hypot((xx + x0 - cx) * res, (yy + y0 - cy) * res)))


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)
        self.distance_decay_m = max(0.1, self.distance_decay_m)
        self.distance_field_downsample = max(1, int(self.distance_field_downsample))

        self._phase = 'explore'
        self._home_xy = self._home_goal_handle = self._home_goal_sent_at = None
        self._home_goal_active = self._using_rtl = self._grounded_reported = False
        self._home_attempts = 0
        self._land_requested_at = self._mode_client = self._altitude_m = None
        self._force_far_goal = False
        self.latest_map = self.latest_costmap = self.latest_global_costmap = None
        self.blacklist = []           # (x, y, added_cycle, expire_cycles) - hard and soft merged
        self._visits = []             # (x, y, added_cycle)
        self.goal_active = self._cancel_requested = False
        self.goal_handle = self._current_target = self._goal_start_time = None
        self._goal_stall_count, self._goal_last_dist = 0, None
        self._empty_streak = self._cycle_count = self._rejection_streak = 0
        self._unreachable_streak = self._tf_fail_streak = 0
        self._tf_stale_reported = False
        self._node_start_time = self._first_pose = self._last_robot_xy = None
        self._current_yaw = self._travel_dir = self._idle_since = None
        self._max_distance_from_start = 0.0
        self._final_sweep_done = self._exploration_complete = False
        self._last_progress_xy = self._last_progress_time = None
        self._recovery_level = 0
        self._imu_tilt_deg = self._tilt_exceeded_since = None
        self._tilt_warned = False
        self._explored_m2 = self._last_coverage = 0.0
        self._last_candidates, self._last_marker_target = [], None
        self._sim_time_at_last_check, self._wall_checks_without_sim_time = None, 0

        def qos(transient):
            q = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                           history=QoSHistoryPolicy.KEEP_LAST)
            q.durability = Dur.TRANSIENT_LOCAL if transient else Dur.VOLATILE
            return q
        cq = qos(self.costmap_qos_transient_local)
        store = lambda attr: (lambda m: setattr(self, attr, m))
        self.create_subscription(OccupancyGrid, self.map_topic, store('latest_map'),
                                 qos(self.map_qos_transient_local))
        self.create_subscription(OccupancyGrid, self.costmap_topic, store('latest_costmap'), cq)
        self.create_subscription(OccupancyGrid, self.global_costmap_topic,
                                 store('latest_global_costmap'), cq)
        self.create_subscription(Odometry, self.odom_topic,
                                 lambda m: setattr(self, '_altitude_m', m.pose.pose.position.z), 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/markers', qos(True))
        self.coverage_pub = self.create_publisher(Float32, '/exploration/coverage', 1)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_action_name = 'navigate_to_pose'
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)
        self._clear_clients = {n: self.create_client(ClearEntireCostmap, n) for n in (
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap')}

        self.timer = self.create_timer(self.replan_period_sec, self.explore_step)
        self.create_timer(1.0, self._marker_tick)
        self.create_timer(self.collision_check_period_sec, self._collision_watchdog)
        self.create_timer(5.0, self._check_airborne)
        self.create_timer(5.0, self._liveness_watchdog,
                          clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self._startup_checks()

    def _startup_checks(self):
        log = self.get_logger()
        if not self.get_parameter('use_sim_time').value:
            if self.count_publishers('/clock') > 0:
                self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
                log.warn('use_sim_time was false but /clock is published - enabled it. '
                         'Pass -p use_sim_time:=true to avoid relying on this.')
            else:
                log.warn('use_sim_time is false and no /clock found. In sim, start Gazebo '
                         'first and relaunch with -p use_sim_time:=true.')
        names = [n for n, _ in self.get_node_names_and_namespaces()]
        for node, why in (('slam_toolbox', 'two map->odom publishers break TF'),
                          ('bt_navigator', 'two Nav2 stacks race every goal'),
                          (self.get_name(), 'two explorers cancel each other')):
            if names.count(node) > 1:
                log.fatal(f'{names.count(node)} "{node}" nodes running - {why}. '
                          f'Kill them (pkill -f {node}) and relaunch once.')
                raise SystemExit(1)
        log.info(f"Waiting for '{self.map_topic}' and '{self.costmap_topic}'. If nothing "
                 f"arrives, check QoS durability with `ros2 topic info <topic> --verbose`.")

    def _imu_cb(self, msg):
        q = msg.orientation
        if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 0.0):
            zz = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
            self._imu_tilt_deg = math.degrees(math.acos(zz))

    def _lookup(self, child, timeout):
        return self.tf_buffer.lookup_transform('map', child, rclpy.time.Time(),
                                               timeout=Duration(seconds=timeout))

    def get_robot_pose(self):
        try:
            t = self._lookup('base_link', 0.3)
        except TransformException as ex:
            self._tf_fail_streak += 1
            text = str(ex)
            nums = re.findall(r'time ([0-9]+\.[0-9]+)', text)
            gap = abs(float(nums[1]) - float(nums[0])) if len(nums) >= 2 else 0.0
            if 'extrapolation into the past' in text and self._tf_fail_streak > 20 and gap > 5.0:
                if not self._tf_stale_reported:
                    self._tf_stale_reported = True
                    self.get_logger().fatal(f'Clock stopped while TF moved on (sim restarted '
                                            f'underneath this node). Restart it.\n  {text}')
            elif self._tf_fail_streak in (1, 10) or self._tf_fail_streak % 50 == 0:
                self.get_logger().warn(f'TF lookup failed x{self._tf_fail_streak}: {text}')
            return None
        self._tf_fail_streak = 0
        q = t.transform.rotation
        self._current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        try:
            o = self._lookup(self.odom_frame, 0.1).transform.translation
            first = self._home_xy is None
            if first or math.hypot(o.x - self._home_xy[0], o.y - self._home_xy[1]) > 0.25:
                self.get_logger().info(f'Home {"set" if first else "moved (SLAM correction)"}'
                                       f' at ({o.x:.2f}, {o.y:.2f}).')
            self._home_xy = (o.x, o.y)
        except TransformException:
            pass
        return t.transform.translation.x, t.transform.translation.y

    def _tilt_state(self):
        tilt = self._imu_tilt_deg
        if tilt is None:
            return 'level', None
        now = self.get_clock().now()
        if tilt <= self.max_safe_tilt_deg:
            if self._tilt_exceeded_since is not None and self._tilt_warned:
                self.get_logger().info(f'Airframe level again ({tilt:.1f} deg).')
            self._tilt_exceeded_since, self._tilt_warned = None, False
            return 'level', tilt
        if self._tilt_exceeded_since is None:
            self._tilt_exceeded_since = now
            return 'transient', tilt
        held = secs(now, self._tilt_exceeded_since)
        if held >= self.tilt_abort_sec:
            return 'sustained', tilt
        if held >= self.tilt_grace_sec and not self._tilt_warned:
            self._tilt_warned = True
            self.get_logger().warn(f'Tilt {tilt:.1f} deg > {self.max_safe_tilt_deg} for '
                                   f'{held:.1f}s - ignoring clearance until level.')
        return 'transient', tilt

    def _live_clearance(self, robot_xy=None):
        if self.latest_costmap is not None:
            g = grid_array(self.latest_costmap)
            if g is None:
                return math.inf, 'local costmap'
            data, res, _, _ = g
            h, w = data.shape
            return nearest_in_window(data >= self.costmap_lethal_threshold, h // 2, w // 2,
                                     self.min_live_clearance_m * 6.0, res), 'local costmap'
        if self.latest_map is None:
            return None, None
        robot_xy = robot_xy or self.get_robot_pose()
        g = grid_array(self.latest_map)
        if robot_xy is None or g is None:
            return None, None
        data, res, ox, oy = g
        cy, cx = to_cell(robot_xy, res, ox, oy)
        if not (0 <= cy < data.shape[0] and 0 <= cx < data.shape[1]):
            return 0.0, '/map (fallback)'
        return nearest_in_window(data >= self.occupied_threshold, cy, cx,
                                 self.min_safe_distance_m * 3.0, res), '/map (fallback)'

    def find_frontiers(self, grid):
        g = grid_array(grid)
        if g is None:
            self.get_logger().warn('Invalid/empty map; skipping this cycle.')
            return []
        data, res, ox, oy = g
        free = (data >= 0) & (data < self.occupied_threshold)
        occupied = data >= self.occupied_threshold
        unknown = data == UNKNOWN
        base = np.zeros_like(free)
        for dy, dx in N8:
            base |= free & np.roll(np.roll(unknown, dy, axis=0), dx, axis=1)
        base[0, :] = base[-1, :] = base[:, 0] = base[:, -1] = False

        floor = self.min_safe_distance_m
        margins = [self.safe_distance_m]
        if self.safe_distance_m > floor:
            margins += [max(floor, self.safe_distance_m * 0.5), floor]
        candidates, seen, relaxed_to = [], set(), None
        for margin in margins:
            mask = base & ~dilate(occupied, max(1, math.ceil(margin / res))) if margin > 0 else base
            cells = set(zip(*map(np.ndarray.tolist, np.nonzero(mask))))
            visited = set()
            for start in cells:
                if start in visited:
                    continue
                cluster, q = [], deque([start])
                visited.add(start)
                while q:
                    cy, cx = q.popleft()
                    cluster.append((cy, cx))
                    for dy, dx in N8:
                        n = (cy + dy, cx + dx)
                        if n in cells and n not in visited:
                            visited.add(n)
                            q.append(n)
                if len(cluster) < self.min_frontier_cells:
                    continue
                my = sum(c[0] for c in cluster) / len(cluster)
                mx = sum(c[1] for c in cluster) / len(cluster)
                by, bx = min(cluster, key=lambda c: (c[0] - my) ** 2 + (c[1] - mx) ** 2)
                cand = (ox + (bx + 0.5) * res, oy + (by + 0.5) * res, len(cluster))
                key = (round(cand[0], 2), round(cand[1], 2))
                if key not in seen:
                    seen.add(key)
                    candidates.append(cand)
            if len(candidates) >= self.min_candidates_before_relax:
                break
            if candidates or margin != margins[0]:
                relaxed_to = margin
        if not candidates:
            self.get_logger().warn(f'No frontiers survive the {floor:.2f} m floor margin.')
        elif relaxed_to is not None and relaxed_to < self.safe_distance_m:
            self.get_logger().warn(f'Only {len(candidates)} frontier(s) at '
                                   f'{self.safe_distance_m:.2f} m - relaxed to {relaxed_to:.2f} m.')
        return candidates

    def _distance_field(self, grid, robot_xy, downsample=None):
        g = grid_array(grid)
        if g is None:
            i = grid.info
            return np.full((i.height, i.width), np.inf), i.origin.position.x, i.origin.position.y, i.resolution
        data, res, ox, oy = g
        inflate = max(1, int(math.ceil(self.robot_radius_m / res)))
        free = (data >= 0) & (data < self.occupied_threshold) & \
            ~dilate(data >= self.occupied_threshold, inflate)
        k = self.distance_field_downsample if downsample is None else max(1, int(downsample))
        h, w = free.shape
        hh, ww = (h // k) * k, (w // k) * k
        if k > 1 and hh and ww:
            free = free[:hh, :ww].reshape(hh // k, k, ww // k, k).all(axis=(1, 3))
            res *= k
        h, w = free.shape
        dist = np.full((h, w), np.inf)
        sy, sx = to_cell(robot_xy, res, ox, oy)
        if not (0 <= sy < h and 0 <= sx < w):
            return dist, ox, oy, res
        if not free[sy, sx]:
            seed = next(((sy + dy, sx + dx) for r in range(1, 6)
                         for dy in range(-r, r + 1) for dx in range(-r, r + 1)
                         if 0 <= sy + dy < h and 0 <= sx + dx < w and free[sy + dy, sx + dx]), None)
            if seed is None:
                return dist, ox, oy, res
            sy, sx = seed
        dist[sy, sx] = 0.0
        steps = [(dy, dx, res * math.hypot(dy, dx)) for dy, dx in N8]
        heap = [(0.0, sy, sx)]
        while heap:
            d, cy, cx = heapq.heappop(heap)
            if d > dist[cy, cx]:
                continue
            for dy, dx, c in steps:
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and d + c < dist[ny, nx]:
                    dist[ny, nx] = d + c
                    heapq.heappush(heap, (d + c, ny, nx))
        return dist, ox, oy, res

    @staticmethod
    def _reachable(field, xy, search_radius_m=3.0):
        dist, ox, oy, res = field
        if res <= 0.0:
            return None
        h, w = dist.shape
        gy, gx = to_cell(xy, res, ox, oy)
        if 0 <= gy < h and 0 <= gx < w and math.isfinite(dist[gy, gx]):
            return xy[0], xy[1], float(dist[gy, gx])
        for r in range(1, max(1, int(search_radius_m / res)) + 1):
            ring = [(float(dist[gy + dy, gx + dx]), dy * dy + dx * dx, gy + dy, gx + dx)
                    for dy in range(-r, r + 1) for dx in range(-r, r + 1)
                    if max(abs(dy), abs(dx)) == r and 0 <= gy + dy < h and 0 <= gx + dx < w
                    and math.isfinite(dist[gy + dy, gx + dx])]
            if ring:
                d, _, ny, nx = min(ring)
                return ox + (nx + 0.5) * res, oy + (ny + 0.5) * res, d
        return None

    def _near(self, pt, entries, radius, live=lambda e: True):
        return any(live(e) and math.hypot(pt[0] - e[0], pt[1] - e[1]) < radius for e in entries)

    def _suppressed(self, pt):
        return self._near(pt, self.blacklist, self.blacklist_radius_m,
                          lambda e: self._cycle_count - e[2] < e[3])

    def _blacklist(self, target, cycles=None):
        if target is not None:
            self.blacklist.append((target[0], target[1], self._cycle_count,
                                   cycles or self.blacklist_expire_cycles))

    def _clear_memory(self, visits=False):
        n = len(self.blacklist)
        self.blacklist.clear()
        if visits:
            self._visits.clear()
            self._travel_dir = None
        return n

    def pick_best_frontier(self, candidates, field, robot_xy):
        g = grid_array(self.latest_map)
        scored, dropped = [], 0
        for wx, wy, _size in candidates:
            if self._suppressed((wx, wy)):
                continue
            reach = self._reachable(field, (wx, wy))
            if reach is None:
                dropped += 1
                continue
            goal_xy, dist = reach[:2], reach[2]
            if goal_xy != (wx, wy) and self._suppressed(goal_xy):
                continue
            if dist < self.min_frontier_dist_m:
                continue
            gain = 0
            if g is not None:
                data, res, ox, oy = g
                cy, cx = to_cell((wx, wy), res, ox, oy)
                r = max(1, int(self.sensor_range_m / res))
                y0, y1 = max(0, cy - r), min(data.shape[0], cy + r + 1)
                x0, x1 = max(0, cx - r), min(data.shape[1], cx + r + 1)
                if y0 < y1 and x0 < x1:
                    yy, xx = np.ogrid[y0 - cy:y1 - cy, x0 - cx:x1 - cx]
                    gain = int(np.count_nonzero((data[y0:y1, x0:x1] == UNKNOWN)
                                                & (yy * yy + xx * xx <= r * r)))
            u = max(gain, 1) * math.exp(-dist / self.distance_decay_m)
            dx, dy = goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1]
            norm = math.hypot(dx, dy)
            if self._travel_dir is not None and self.direction_weight_m > 0.0 and norm >= 1e-6:
                cos_turn = (dx * self._travel_dir[0] + dy * self._travel_dir[1]) / norm
                u *= math.exp(-self.direction_weight_m * (1.0 - cos_turn) * 0.5)
            visits = sum(1 for v in self._visits if math.hypot(goal_xy[0] - v[0],
                         goal_xy[1] - v[1]) < self.visit_radius_m)
            u *= math.exp(-self.visit_penalty_m * visits / 6.0)
            if dist <= self.commitment_radius_m and not self._force_far_goal:
                u *= 1.5
            scored.append((u, dist, gain, goal_xy))
        if not scored:
            if dropped:
                self.get_logger().warn(f'All {len(candidates)} frontier(s) rejected; {dropped} '
                                       f'had no reachable approach point within 3 m.')
            return None
        self._force_far_goal = False
        u, d, gain, pt = max(scored, key=lambda t: t[0])
        self.get_logger().debug(f'best {pt}: gain={gain} dist={d:.1f} u={u:.1f} '
                                f'({len(scored)} scored)')
        return pt

    def explore_step(self):
        self._cycle_count += 1
        c = self._cycle_count
        self.blacklist = [b for b in self.blacklist if c - b[2] < b[3]]
        self._visits = [v for v in self._visits if c - v[2] < self.visit_memory_cycles]
        if self._phase == 'return':
            return self._return_step()
        if self._phase in ('land', 'done'):
            return self._land_step()

        now = self.get_clock().now()
        if self._exploration_complete:
            if self._idle_since is not None and secs(now, self._idle_since) < self.idle_recheck_sec:
                return
            self._idle_since = now
            if self.latest_map is None or not self.find_frontiers(self.latest_map):
                return
            self.get_logger().warn('New frontiers appeared after completion - resuming.')
            self._exploration_complete = self._final_sweep_done = False
            self._empty_streak = 0

        if now.nanoseconds == 0:
            return
        if self._node_start_time is None:
            self._node_start_time = now
            self.get_logger().info(f'Clock live; {self.startup_delay_sec:.0f}s startup grace.')
        if secs(now, self._node_start_time) < self.startup_delay_sec:
            return

        robot_xy = self.get_robot_pose()
        if robot_xy is not None:
            self._check_progress(robot_xy)
        if self.goal_active:
            return self._supervise_goal(robot_xy)
        if self.latest_map is None or robot_xy is None:
            return

        self._last_robot_xy = robot_xy
        self._first_pose = self._first_pose or robot_xy
        self._max_distance_from_start = max(self._max_distance_from_start, math.hypot(
            robot_xy[0] - self._first_pose[0], robot_xy[1] - self._first_pose[1]))
        cov = self._publish_coverage()

        candidates = self.find_frontiers(self.latest_map)
        target = None
        if candidates:
            target = self.pick_best_frontier(
                candidates, self._distance_field(self.latest_map, robot_xy), robot_xy)
            if target is None and self.distance_field_downsample > 1:
                target = self.pick_best_frontier(
                    candidates, self._distance_field(self.latest_map, robot_xy, 1), robot_xy)
                if target is not None:
                    self.get_logger().info('Full-resolution pass found a route (narrow gap).')
        self._last_candidates, self._last_marker_target = candidates, target

        if target is not None:
            self._empty_streak = 0
            self.get_logger().info(f'Explored {self._explored_m2:.0f} m2 ({cov:.1f}%) | '
                                   f'{len(candidates)} frontier(s) | goal -> '
                                   f'({target[0]:.2f}, {target[1]:.2f})')
            return self.send_goal(target, robot_xy)

        if candidates:
            self._unreachable_streak += 1
            self.get_logger().info(f'{cov:.1f}% | {len(candidates)} frontier(s), none reachable '
                                   f'({self._unreachable_streak} cycle(s)).')
            if self._unreachable_streak % 10 == 0 and self._clear_memory():
                self.get_logger().warn('Cleared suppression entries after 10 unreachable cycles.')
            return

        self._unreachable_streak = 0
        self._empty_streak += 1
        self.get_logger().info(f'{cov:.1f}% | no frontiers '
                               f'({self._empty_streak}/{self.empty_streak_limit}).')
        if self._empty_streak < self.empty_streak_limit:
            return
        if self.final_sweep and not self._final_sweep_done:
            self._final_sweep_done, self._empty_streak = True, 0
            n = self._clear_memory(visits=True)
            self.get_logger().warn(f'Final sweep: cleared {n} suppression entries, trying again.')
            return
        if self._max_distance_from_start < self.min_frontier_dist_m:
            self.get_logger().error('Stopping without ever moving - likely Nav2 robot_radius/'
                                    'inflation larger than clearance at spawn.')
        else:
            self.get_logger().info(f'Exploration complete. Final coverage {cov:.1f}%.')
        self._exploration_complete, self._idle_since = True, self.get_clock().now()
        if self.return_home_on_complete:
            self._begin_return()

    def send_goal(self, target_xy, robot_xy):
        log = self.get_logger()
        state, tilt = self._tilt_state()
        if state == 'sustained':
            return log.warn(f'Holding goal: tilt {tilt:.1f} deg sustained.')
        if not self.nav_client.server_is_ready():
            return log.warn('Nav2 action server not ready - retrying next cycle.')
        if self.latest_costmap is not None:
            clearance, _ = self._live_clearance()
            if clearance < self.min_live_clearance_m:
                log.error(f'Refusing goal {target_xy}: live clearance {clearance:.2f} m.')
                return self._blacklist(target_xy)
        if self.latest_global_costmap is not None:
            g = grid_array(self.latest_global_costmap)
            if g is not None:
                data, res, ox, oy = g
                cy, cx = to_cell(target_xy, res, ox, oy)
                if 0 <= cy < data.shape[0] and 0 <= cx < data.shape[1] \
                        and data[cy, cx] >= self.costmap_lethal_threshold:
                    log.warn(f'Refusing goal {target_xy}: global costmap cost {data[cy, cx]}.')
                    return self._blacklist(target_xy)
        if self._action_server_count() > 1:
            return log.error('Multiple navigate_to_pose servers - refusing. Kill the stale '
                             'bt_navigator (`ros2 node list`).')
        dx, dy = target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]
        yaw = self._current_yaw if self._current_yaw is not None else (
            math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0)
        self.goal_active, self._cancel_requested = True, False
        self._current_target, self._goal_start_time = target_xy, self.get_clock().now()
        self._goal_stall_count, self._goal_last_dist = 0, None
        self._visits.append((*target_xy, self._cycle_count))
        if math.hypot(dx, dy) > 1e-6:
            n = math.hypot(dx, dy)
            self._travel_dir = (dx / n, dy / n)
        self._dispatch(target_xy, yaw, functools.partial(self._goal_response_cb, target=target_xy))

    def _dispatch(self, xy, yaw, response_cb):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        p, o = goal.pose.pose.position, goal.pose.pose.orientation
        p.x, p.y = float(xy[0]), float(xy[1])
        o.x, o.y, o.z, o.w = yaw_to_quaternion(yaw)
        self.nav_client.send_goal_async(goal).add_done_callback(response_cb)

    def _action_server_count(self):
        try:
            from rclpy.action.graph import get_action_server_names_and_types_by_node
            wanted, count = '/' + self.nav_action_name.lstrip('/'), 0
            for name, ns in self.get_node_names_and_namespaces():
                try:
                    entries = get_action_server_names_and_types_by_node(self, name, ns)
                except Exception:
                    continue
                count += any(a == wanted for a, _ in entries)
            return max(count, 1)
        except Exception:
            return 1

    def _cancel_goal(self, blacklist=False, full_reset=False):
        if self.goal_handle is not None and not self._cancel_requested:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        if blacklist:
            self._blacklist(self._current_target)
        self.goal_active = False
        if full_reset:
            self.goal_handle = self._goal_start_time = self._current_target = None
            self._cancel_requested = False
        else:
            self._cancel_requested = True

    def _supervise_goal(self, robot_xy):
        log = self.get_logger()
        if self._goal_start_time is not None:
            elapsed = secs(self.get_clock().now(), self._goal_start_time)
            if elapsed > self.goal_hard_reset_sec:
                log.error(f'HARD RESET: goal active {elapsed:.0f}s with no result from Nav2.')
                self._cancel_requested = False
                return self._cancel_goal(blacklist=True, full_reset=True)
            if not self._cancel_requested:
                if self.goal_handle is None:
                    if elapsed > self.goal_accept_timeout_sec:
                        log.error(f'No goal-accepted response after {elapsed:.1f}s - check for '
                                  f'duplicate bt_navigator. Resetting.')
                        self.goal_active = False
                        self._goal_start_time = self._current_target = None
                elif elapsed > self.goal_timeout_sec:
                    log.warn(f'Goal {self._current_target} exceeded {self.goal_timeout_sec}s.')
                    self._cancel_requested = True
                    self.goal_handle.cancel_goal_async()
        if robot_xy is None or self._current_target is None:
            return
        d = math.hypot(robot_xy[0] - self._current_target[0], robot_xy[1] - self._current_target[1])
        if self._goal_last_dist is None or self._goal_last_dist - d >= self.goal_stall_progress_m:
            self._goal_last_dist, self._goal_stall_count = d, 0
            return
        self._goal_stall_count += 1
        if self._goal_stall_count >= self.goal_stall_cycles:
            log.warn(f'Goal {self._current_target} not closer than {d:.2f} m in '
                     f'{self.goal_stall_cycles} cycles - blacklisting.')
            self._cancel_goal(blacklist=True)
            self._goal_stall_count, self._goal_last_dist = 0, None
            self.timer.reset()

    def _goal_response_cb(self, future, target=None):
        handle = future.result()
        log = self.get_logger()
        if handle is None or not handle.accepted:
            self._blacklist(target)
            self.goal_active = False
            if handle is None:
                log.error('Nav2 returned no goal handle - check for duplicate action servers.')
                self._goal_start_time = self._current_target = None
                return
            self._rejection_streak += 1
            log.warn(f'Goal rejected ({self._rejection_streak}/{self.rejection_streak_limit}).')
            if self._rejection_streak >= self.rejection_streak_limit:
                if self.safe_distance_m > self.min_safe_distance_m:
                    self.safe_distance_m = max(self.min_safe_distance_m, self.safe_distance_m * 0.5)
                    log.warn(f'Shrinking safe_distance_m to {self.safe_distance_m:.2f} m. '
                             f'Real fix: align Nav2 robot_radius/inflation_radius.')
                self._rejection_streak = 0
            return
        self._rejection_streak = 0
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            functools.partial(self._goal_result_cb, target=target, handle=handle))

    def _goal_result_cb(self, future, target=None, handle=None):
        if handle is not None and self.goal_handle is not None \
                and handle.goal_id != self.goal_handle.goal_id:
            return
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal {target} did not succeed - blacklisting.')
            self._blacklist(target)
        else:
            where = self.get_robot_pose()
            if where is not None and target is not None:
                short = math.hypot(where[0] - target[0], where[1] - target[1])
                if short > self.goal_arrival_tolerance_m:
                    self.get_logger().warn(f'Nav2 SUCCEEDED but {short:.2f} m from {target}; '
                                           f'match Nav2 xy_goal_tolerance to this node.')
            self.get_logger().info(f'Reached frontier at {target}.')
            self._blacklist(target, self.success_blacklist_cycles)
        self.goal_active = self._cancel_requested = False
        self.goal_handle = self._current_target = self._goal_start_time = None
        self.timer.reset()

    def _check_progress(self, robot_xy):
        now = self.get_clock().now()
        if self._last_progress_xy is None:
            self._last_progress_xy, self._last_progress_time = robot_xy, now
            return
        moved = math.hypot(robot_xy[0] - self._last_progress_xy[0],
                           robot_xy[1] - self._last_progress_xy[1])
        if moved >= self.stuck_distance_m:
            if self._recovery_level:
                self.get_logger().info('Moving again - recovery ladder reset.')
            self._last_progress_xy, self._last_progress_time = robot_xy, now
            self._recovery_level = 0
            return
        stalled = secs(now, self._last_progress_time)
        if stalled < self.stuck_timeout_sec:
            return
        self._recovery_level += 1
        lvl = self._recovery_level
        log = self.get_logger()
        log.error(f'STUCK: moved {moved:.2f} m in {stalled:.0f}s. Recovery level {lvl}.')
        self._cancel_requested = False
        self._cancel_goal(full_reset=True)
        if lvl >= 2:
            for name, cli in self._clear_clients.items():
                try:
                    if cli.service_is_ready():
                        cli.call_async(ClearEntireCostmap.Request())
                        log.warn(f'Recovery 2: cleared {name}')
                    else:
                        log.warn(f'{name} unavailable - is Nav2 active?')
                except Exception as ex:
                    log.warn(f'Could not clear {name}: {ex}')
        if lvl >= 3:
            log.warn(f'Recovery 3: dropped {self._clear_memory(visits=True)} suppression entries.')
        if lvl >= 4:
            self._force_far_goal, self._recovery_level = True, 0
            log.warn('Recovery 4: ignoring nearby-frontier bonus for one cycle.')
        self._last_progress_time = now

    def _collision_watchdog(self):
        state, tilt = self._tilt_state()
        if state == 'sustained':
            if self.goal_active and self.goal_handle is not None and not self._cancel_requested:
                self.get_logger().error(f'COLLISION GUARD [tilt {tilt:.1f} deg sustained] - '
                                        f'cancelling goal.')
                self._cancel_goal()
            return
        if state == 'transient':
            return
        clearance, source = self._live_clearance()
        if clearance is None or clearance >= self.min_live_clearance_m:
            return
        msg = (f'COLLISION GUARD [{source}]: clearance {clearance:.2f} m < '
               f'min_live_clearance_m={self.min_live_clearance_m:.2f} m')
        if self.goal_active and self.goal_handle is not None and not self._cancel_requested:
            self.get_logger().error(msg + ' - cancelling goal.')
            self._cancel_goal(blacklist=True)
        else:
            self.get_logger().warn(msg + ' (no goal in flight).')

    def _check_airborne(self):
        if self._altitude_m is None:
            return
        if self._altitude_m >= self.min_flight_altitude_m:
            self._grounded_reported = False
        elif not self._grounded_reported:
            self._grounded_reported = True
            self.get_logger().fatal(f'DRONE ON THE GROUND (alt {self._altitude_m:.2f} m). '
                                    f'In MAVProxy: mode 4 / arm throttle / takeoff 2')

    def _liveness_watchdog(self):
        now = self.get_clock().now().nanoseconds
        prev, self._sim_time_at_last_check = self._sim_time_at_last_check, now
        if prev is None:
            return
        if now != 0 and now != prev:
            self._wall_checks_without_sim_time = 0
            return
        self._wall_checks_without_sim_time += 1
        if self._wall_checks_without_sim_time > 2 and self._wall_checks_without_sim_time % 12:
            return
        if self.get_parameter('use_sim_time').value:
            self.get_logger().error('ROS TIME NOT ADVANCING (no /clock). All timers frozen. '
                                    'Check `ros2 topic hz /clock`, `ros2 topic info /clock` '
                                    '(exactly one publisher), `pgrep -af "gz sim"`.')
        else:
            self.get_logger().error('ROS time not advancing and use_sim_time is false.')

    def _begin_return(self):
        if self._home_xy is None:
            self.get_logger().error('No home recorded - falling back to autopilot RTL.')
            return self._go_rtl()
        if self.goal_active and self.goal_handle is not None:
            self._cancel_goal()
        self._phase, self._home_attempts, self._home_goal_active = 'return', 0, False
        self.get_logger().info(f'EXPLORATION COMPLETE - returning to ({self._home_xy[0]:.2f}, '
                               f'{self._home_xy[1]:.2f}).')

    def _go_rtl(self):
        self._phase, self._using_rtl = 'land', True
        self._request_mode(self.rtl_mode_id, 'RTL')

    def _return_step(self):
        robot_xy = self.get_robot_pose()
        if robot_xy is None:
            return
        dist = math.hypot(robot_xy[0] - self._home_xy[0], robot_xy[1] - self._home_xy[1])
        if dist <= self.home_tolerance_m:
            if self._home_goal_active and self._home_goal_handle is not None:
                self._home_goal_handle.cancel_goal_async()
            self._home_goal_active, self._phase, self._land_requested_at = False, 'land', None
            return self.get_logger().info(f'Back over the pad ({dist:.2f} m).')
        if self._home_goal_active:
            if self._home_goal_sent_at is not None and \
                    secs(self.get_clock().now(), self._home_goal_sent_at) > self.home_goal_timeout_sec:
                self.get_logger().warn('Return leg timed out - cancelling and retrying.')
                if self._home_goal_handle is not None:
                    self._home_goal_handle.cancel_goal_async()
                self._home_goal_active = False
            return
        if self._home_attempts >= self.return_home_max_attempts:
            self.get_logger().error(f'Nav2 failed {self.return_home_max_attempts} return '
                                    f'attempts ({dist:.2f} m away) - handing to RTL.')
            return self._go_rtl()
        if not self.nav_client.server_is_ready():
            return self.get_logger().warn('Nav2 not ready for the return leg yet.')
        self._home_attempts += 1
        self.get_logger().info(f'Return leg {self._home_attempts}/'
                               f'{self.return_home_max_attempts}: {dist:.2f} m to home.')
        self._home_goal_active, self._home_goal_sent_at = True, self.get_clock().now()
        yaw = math.atan2(self._home_xy[1] - robot_xy[1], self._home_xy[0] - robot_xy[0])
        self._dispatch(self._home_xy, yaw, self._home_goal_response_cb)

    def _home_goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as ex:
            self.get_logger().error(f'Return goal failed to dispatch: {ex}')
            self._home_goal_active = False
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the return goal.')
            self._home_goal_active = False
            return
        self._home_goal_handle = handle
        handle.get_result_async().add_done_callback(self._home_goal_result_cb)

    def _home_goal_result_cb(self, future):
        try:
            status = future.result().status
        except Exception as ex:
            self.get_logger().error(f'Return goal result error: {ex}')
            status = -1
        self._home_goal_active, self._home_goal_handle = False, None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Return leg succeeded.')
        else:
            self.get_logger().warn(f'Return leg ended with status {status} - will retry.')
        self.timer.reset()

    def _request_mode(self, mode_id, label):
        if self._mode_client is None:
            self._mode_client = self.create_client(ModeSwitch, self.mode_switch_service)
        if not self._mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'{self.mode_switch_service} unavailable - cannot command '
                                    f'{label}. Check AP_DDS: ros2 service list | grep ap/')
            return False

        def done(f):
            try:
                r = f.result()
                (self.get_logger().info if r.status else self.get_logger().error)(
                    f'{label} {"accepted" if r.status else "REFUSED"} (mode {r.curr_mode}).')
            except Exception as ex:
                self.get_logger().error(f'{label} request errored: {ex}')
        self._mode_client.call_async(ModeSwitch.Request(mode=int(mode_id))).add_done_callback(done)
        return True

    def _land_step(self):
        if self._phase == 'done':
            return
        landed = self._altitude_m is not None and self._altitude_m <= self.landed_altitude_m
        if not self.land_on_arrival:
            self.get_logger().info('Home reached; land_on_arrival is false - holding.')
        elif self._using_rtl:
            if not landed:
                return
        elif self._land_requested_at is None:
            self._land_requested_at = self.get_clock().now()
            self.get_logger().info('Landing.')
            self._request_mode(self.land_mode_id, 'LAND')
            return
        elif not landed:
            if self._altitude_m is not None and \
                    secs(self.get_clock().now(), self._land_requested_at) > 15.0:
                self._land_requested_at = self.get_clock().now()
                self.get_logger().warn(f'Still at {self._altitude_m:.2f} m - re-issuing LAND.')
                self._request_mode(self.land_mode_id, 'LAND')
            return
        self._phase = 'done'
        xy = self.get_robot_pose()
        err = (math.hypot(xy[0] - self._home_xy[0], xy[1] - self._home_xy[1])
               if xy is not None and self._home_xy is not None else float('nan'))
        cov = self._coverage()[0] if self.latest_map is not None else 0.0
        self.get_logger().info(f'RUN COMPLETE | {self._explored_m2:.0f} m2 ({cov:.1f}%) | '
                               f'home error {err:.2f} m | altitude {self._altitude_m}')

    def _coverage(self):
        data = np.frombuffer(bytearray(self.latest_map.data), dtype=np.int8)
        if data.size == 0:
            return 0.0, 0, 0, 0
        unk = int(np.count_nonzero(data == UNKNOWN))
        occ = int(np.count_nonzero(data >= self.occupied_threshold))
        self._explored_m2 = (data.size - unk) * self.latest_map.info.resolution ** 2
        return 100.0 * (data.size - unk) / data.size, data.size - unk - occ, occ, unk

    def _publish_coverage(self):
        self._last_coverage = self._coverage()[0]
        self.coverage_pub.publish(Float32(data=float(self._last_coverage)))
        return self._last_coverage

    def _marker_tick(self):
        if self.latest_map is None:
            return
        self._publish_coverage()
        stamp = self.get_clock().now().to_msg()

        def mk(ns, mid, mtype, scale, rgba, x=0.0, y=0.0, z=0.0, action=Marker.ADD):
            m = Marker(ns=ns, id=mid, type=mtype, action=action)
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), z
            m.pose.orientation.w = 1.0
            return m

        arr = MarkerArray()
        arr.markers.append(mk('', 0, Marker.SPHERE, 0.0, (0.0, 0.0, 0.0, 0.0), action=Marker.DELETEALL))
        pts = mk('frontier_candidates', 0, Marker.SPHERE_LIST, 0.18, (1.0, 0.85, 0.0, 0.85))
        pts.points = [Point(x=float(wx), y=float(wy), z=0.05) for wx, wy, _ in self._last_candidates]
        arr.markers.append(pts)
        t = self._last_marker_target
        if t is not None:
            arr.markers.append(mk('chosen_goal', 1, Marker.SPHERE, 0.5, (0.1, 1.0, 0.2, 0.95),
                                  t[0], t[1], 0.1))
        info = self.latest_map.info
        txt = mk('coverage', 2, Marker.TEXT_VIEW_FACING, 0.6, (1.0, 1.0, 0.25, 1.0),
                 info.origin.position.x + 2.2,
                 info.origin.position.y + info.height * info.resolution + 0.9)
        txt.text = (f'{self._explored_m2:.0f} m2 explored   |   {self._last_coverage:.1f}% of '
                    f'mapped area   |   {len(self._last_candidates)} frontiers')
        arr.markers.append(txt)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_active and node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
