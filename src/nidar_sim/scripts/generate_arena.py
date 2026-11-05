import argparse
import math
import os
import random
import string

CELLS = 13
THICKNESS = 0.10
WALL_H = 2.60
NET_Z = 2.50
NET_BAR = 0.03
PAD = 0.61
ROOM_CELLS = 2

MIRRORS = {"ne": (1, 1), "nw": (-1, 1), "se": (1, -1), "sw": (-1, -1)}

class Arena:

    def __init__(self, rng, n_rooms, n_survivors, loop_prob, start_mode,
                 start_cells):
        self.rng = rng
        self.n = CELLS
        self.n_rooms = n_rooms
        self.n_survivors = n_survivors
        self.loop_prob = loop_prob
        self.start_mode = start_mode
        self.start_cells = start_cells

        if start_mode == "porch":
            k = 2
            self.start_origin = (self.n - k, self.n)
            self.pad_cell = (self.n - 1, self.n)
            self.entry = (self.n - 1, self.n - 1)
        else:
            k = start_cells
            self.start_origin = (self.n - k, self.n - k)
            self.pad_cell = (self.n - 1, self.n - 1)
            self.entry = None
        self.start_size = k
        self.start_set = {(self.start_origin[0] + a, self.start_origin[1] + b)
                          for a in range(k) for b in range(k)}

        self.rooms = []
        self.room_of = {}
        self.vwall = {}
        self.hwall = {}
        self.doors = []
        self.survivors = []

    def in_arena(self, i, j):
        return 0 <= i < self.n and 0 <= j < self.n

    def is_corridor(self, i, j):
        return (self.in_arena(i, j) and (i, j) not in self.room_of
                and (i, j) not in self.start_set)

    def open_cells(self):
        cells = [(i, j) for i in range(self.n) for j in range(self.n)]
        if self.start_mode == "porch":
            cells += sorted(self.start_set)
        return cells

    def _init_walls(self):
        self.vwall = {(i, j): True
                      for i in range(-1, self.n) for j in range(self.n)}
        self.hwall = {(i, j): True
                      for i in range(self.n) for j in range(-1, self.n)}

        if self.start_mode != "porch":
            return

        for j in (self.n, self.n + 1):
            self.vwall[(self.n - 3, j)] = True
            self.vwall[(self.n - 1, j)] = True
        for i in (self.n - 2, self.n - 1):
            self.hwall[(i, self.n + 1)] = True

        self.hwall[(self.n - 1, self.n - 1)] = False

    def _carve(self):
        start = self.entry
        if start is None or not self.is_corridor(*start):
            start = next((i, j) for i in range(self.n) for j in range(self.n)
                         if self.is_corridor(i, j))
        stack = [start]
        seen = {start}
        while stack:
            i, j = stack[-1]
            nbrs = []
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if self.is_corridor(ni, nj) and (ni, nj) not in seen:
                    nbrs.append((ni, nj, di, dj))
            if not nbrs:
                stack.pop()
                continue
            ni, nj, di, dj = self.rng.choice(nbrs)
            if di:
                self.vwall[(min(i, ni), j)] = False
            else:
                self.hwall[(i, min(j, nj))] = False
            seen.add((ni, nj))
            stack.append((ni, nj))
        return seen

    def _braid(self):
        for (i, j), present in list(self.vwall.items()):
            if not present or not (0 <= i < self.n - 1):
                continue
            if self.is_corridor(i, j) and self.is_corridor(i + 1, j):
                if self.rng.random() < self.loop_prob:
                    self.vwall[(i, j)] = False
        for (i, j), present in list(self.hwall.items()):
            if not present or not (0 <= j < self.n - 1):
                continue
            if self.is_corridor(i, j) and self.is_corridor(i, j + 1):
                if self.rng.random() < self.loop_prob:
                    self.hwall[(i, j)] = False

    def _place_rooms(self):
        self.rooms, self.room_of = [], {}
        tries = 0
        while len(self.rooms) < self.n_rooms and tries < 400:
            tries += 1
            ri = self.rng.randrange(0, self.n - ROOM_CELLS + 1)
            rj = self.rng.randrange(0, self.n - ROOM_CELLS + 1)
            taken = [(o, ROOM_CELLS) for o in self.rooms]
            if self.start_mode != "porch":
                taken.append((self.start_origin, self.start_size))
            if any(_blocks_clash((ri, rj), ROOM_CELLS, o, k) for o, k in taken):
                continue
            cells = [(ri + a, rj + b)
                     for a in range(ROOM_CELLS) for b in range(ROOM_CELLS)]
            if self.entry in cells or self.start_set & set(cells):
                continue
            self.rooms.append((ri, rj))
            for c in cells:
                self.room_of[c] = len(self.rooms) - 1
        if len(self.rooms) < self.n_rooms:
            return False

        corridors = [(i, j) for i in range(self.n) for j in range(self.n)
                     if self.is_corridor(i, j)]
        if not corridors:
            return False
        seen, stack = {corridors[0]}, [corridors[0]]
        while stack:
            i, j = stack.pop()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (i + di, j + dj)
                if self.is_corridor(*nb) and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        return len(seen) == len(corridors)

    def _open_block(self, ri, rj, k=ROOM_CELLS, doors=1):
        for b in range(k):
            for a in range(k - 1):
                self.vwall[(ri + a, rj + b)] = False
        for a in range(k):
            for b in range(k - 1):
                self.hwall[(ri + a, rj + b)] = False
        for b in range(k):
            self.vwall[(ri - 1, rj + b)] = True
            self.vwall[(ri + k - 1, rj + b)] = True
        for a in range(k):
            self.hwall[(ri + a, rj - 1)] = True
            self.hwall[(ri + a, rj + k - 1)] = True

        cands = []
        for b in range(k):
            if self.is_corridor(ri - 1, rj + b):
                cands.append(("v", ri - 1, rj + b))
            if self.is_corridor(ri + k, rj + b):
                cands.append(("v", ri + k - 1, rj + b))
        for a in range(k):
            if self.is_corridor(ri + a, rj - 1):
                cands.append(("h", ri + a, rj - 1))
            if self.is_corridor(ri + a, rj + k):
                cands.append(("h", ri + a, rj + k - 1))
        if not cands:
            return False
        chosen = cands if doors is None else [self.rng.choice(cands)]
        for kind, i, j in chosen:
            (self.vwall if kind == "v" else self.hwall)[(i, j)] = False
            self.doors.append((kind, i, j))
        return True

    def _open_rooms(self):
        for ri, rj in self.rooms:
            if not self._open_block(ri, rj):
                return False
        if self.start_mode != "porch":
            return self._open_block(
                *self.start_origin, k=self.start_size,
                doors=None if self.start_mode == "open" else 1)
        return True

    def _connected(self):
        cells = set(self.open_cells())
        start = self.pad_cell
        seen, stack = {start}, [start]
        while stack:
            i, j = stack.pop()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (i + di, j + dj)
                if nb not in cells or nb in seen:
                    continue
                blocked = (self.vwall.get((min(i, nb[0]), j), False) if di
                           else self.hwall.get((i, min(j, nb[1])), False))
                if not blocked:
                    seen.add(nb)
                    stack.append(nb)
        return len(seen) == len(cells)

    def _place_survivors(self, geom):
        picks = self.rng.sample(range(len(self.rooms)),
                                min(self.n_survivors, len(self.rooms)))
        for n, idx in enumerate(sorted(picks), start=1):
            ri, rj = self.rooms[idx]
            cx = (geom.cx(ri) + geom.cx(ri + ROOM_CELLS - 1)) / 2.0
            cy = (geom.cy(rj) + geom.cy(rj + ROOM_CELLS - 1)) / 2.0
            half = (ROOM_CELLS * geom.corridor +
                    (ROOM_CELLS - 1) * THICKNESS) / 2.0
            slack = max(0.0, half - 0.45)
            self.survivors.append({
                "n": n,
                "room": idx,
                "x": cx + self.rng.uniform(-slack, slack),
                "y": cy + self.rng.uniform(-slack, slack),
                "yaw": self.rng.uniform(-math.pi, math.pi),
                "cell": (ri + self.rng.randrange(ROOM_CELLS),
                         rj + self.rng.randrange(ROOM_CELLS)),
            })
        for s in self.survivors:
            s["cell"] = (geom.cell_of_x(s["x"]), geom.cell_of_y(s["y"]))

    def build(self, geom):
        for _ in range(200):
            self.doors = []
            if not self._place_rooms():
                continue
            self._init_walls()
            self._carve()
            self._braid()
            if not self._open_rooms():
                continue
            if not self._connected():
                continue
            self._place_survivors(geom)
            return True
        return False

class Geom:

    def __init__(self, corridor):
        self.corridor = corridor
        self.pitch = corridor + THICKNESS

    def cx(self, i):
        return i * self.pitch

    def cy(self, j):
        return j * self.pitch

    def cell_of_x(self, x):
        return int(round(x / self.pitch))

    def cell_of_y(self, y):
        return int(round(y / self.pitch))

def _blocks_clash(a_origin, a_size, b_origin, b_size):
    ai, aj = a_origin
    bi, bj = b_origin
    sep_x = ai + a_size < bi or bi + b_size < ai
    sep_y = aj + a_size < bj or bj + b_size < aj
    return not (sep_x or sep_y)

def merge_runs(indices):
    out, run = [], []
    for k in sorted(indices):
        if run and k == run[-1] + 1:
            run.append(k)
        else:
            if run:
                out.append((run[0], run[-1]))
            run = [k]
    if run:
        out.append((run[0], run[-1]))
    return out

def wall_boxes(arena, geom):
    boxes = []
    lines = {}
    for (i, j), present in arena.vwall.items():
        if present:
            lines.setdefault(("v", i), []).append(j)
    for (i, j), present in arena.hwall.items():
        if present:
            lines.setdefault(("h", j), []).append(i)

    p = geom.pitch
    for (kind, line), idxs in lines.items():
        for a, b in merge_runs(idxs):
            length = (b - a + 1) * p
            mid = (a + b) / 2.0 * p
            edge = (line + 0.5) * p
            if kind == "v":
                boxes.append((edge, mid, THICKNESS, length))
            else:
                boxes.append((mid, edge, length, THICKNESS))
    return boxes

HEADER = """<?xml version="1.0" ?>
<!-- Generated by nidar_sim/scripts/generate_arena.py - DO NOT EDIT BY HAND.
     Regenerate with the command recorded in {name}.yaml. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-air-pressure-system"
      name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system"
      name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-altimeter-system"
      name="gz::sim::systems::Altimeter"/>
    <plugin filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-magnetometer-system"
      name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system"
      name="gz::sim::systems::NavSat"/>

    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
      <sky/>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <spherical_coordinates>
      <latitude_deg>-35.3632621</latitude_deg>
      <longitude_deg>149.1652374</longitude_deg>
      <elevation>10.0</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><diffuse>0.28 0.28 0.30 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

BOX_LINK = """      <link name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <collision name="{name}_col">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="{name}_vis">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>
"""

def box_link(name, x, y, z, sx, sy, sz, rgba):
    r, g, b, a = rgba
    return BOX_LINK.format(name=name, x=x, y=y, z=z, sx=sx, sy=sy, sz=sz,
                           r=r, g=g, b=b, a=a)

def survivor_model(s):
    skin = (0.85, 0.70, 0.55, 1)
    hivis = (0.95, 0.45, 0.05, 1)
    parts = [
        box_link("torso", 0.00, 0, 0.14, 0.52, 0.36, 0.28, hivis),
        box_link("legs", -0.52, 0, 0.10, 0.52, 0.30, 0.20, hivis),
        '      <link name="head">\n'
        '        <pose>0.3700 0 0.1600 0 0 0</pose>\n'
        '        <collision name="head_col"><geometry><sphere><radius>0.11</radius></sphere></geometry></collision>\n'
        '        <visual name="head_vis"><geometry><sphere><radius>0.11</radius></sphere></geometry>\n'
        '          <material><ambient>%s %s %s %s</ambient><diffuse>%s %s %s %s</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n' % (skin + skin),
    ]
    return (
        '    <model name="survivor_{n}">\n'
        '      <static>true</static>\n'
        '      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>\n'
        '{parts}'
        '    </model>\n'
    ).format(n=s["n"], x=s["x"], y=s["y"], yaw=s["yaw"], parts="".join(parts))

def emit_sdf(name, arena, geom, mx, my, pad_xy):
    px, py = pad_xy

    def wx(x):
        return mx * (x - px)

    def wy(y):
        return my * (y - py)

    out = [HEADER.format(name=name)]

    wall_rgba = (0.72, 0.72, 0.74, 1)
    out.append('    <model name="arena">\n      <static>true</static>\n')
    for k, (cx, cy, sx, sy) in enumerate(wall_boxes(arena, geom)):
        out.append(box_link("wall_%03d" % k, wx(cx), wy(cy), WALL_H / 2.0,
                            sx, sy, WALL_H, wall_rgba))

    p = geom.pitch
    net = (0.15, 0.15, 0.15, 1)
    x_lo, x_hi = -0.5 * p, (CELLS - 0.5) * p
    y_hi = (CELLS + 1.5) * p if arena.start_mode == "porch" else (CELLS - 0.5) * p
    y_lo = -0.5 * p
    x_mid, y_mid = (x_lo + x_hi) / 2.0, (y_lo + y_hi) / 2.0
    for k in range(CELLS + 1):
        at = (k - 0.5) * p
        out.append(box_link("net_x_%02d" % k, wx(at), wy(y_mid), NET_Z,
                            NET_BAR, y_hi - y_lo, NET_BAR, net))
    for k in range(CELLS + 3 if arena.start_mode == "porch" else CELLS + 1):
        at = (k - 0.5) * p
        out.append(box_link("net_y_%02d" % k, wx(x_mid), wy(at), NET_Z,
                            x_hi - x_lo, NET_BAR, NET_BAR, net))
    out.append('    </model>\n')

    out.append('    <model name="launch_pad">\n      <static>true</static>\n')
    out.append(box_link("pad", 0, 0, 0.005, PAD, PAD, 0.01,
                        (0.10, 0.75, 0.25, 1)))
    out.append('    </model>\n')

    for s in arena.survivors:
        out.append(survivor_model(dict(s, x=wx(s["x"]), y=wy(s["y"]))))

    out.append("  </world>\n</sdf>\n")
    return "".join(out)

def ascii_plan(arena, mx, my):
    n = arena.n
    rows = 2 * (n + 2 if arena.start_mode == "porch" else n) + 1
    cols = 2 * n + 1
    m = [[" "] * cols for _ in range(rows)]

    for (i, j), present in arena.vwall.items():
        if present:
            m[2 * j + 1][2 * i + 2] = "|"
    for (i, j), present in arena.hwall.items():
        if present:
            m[2 * j + 2][2 * i + 1] = "-"
    for r in range(0, rows, 2):
        for c in range(0, cols, 2):
            near = []
            if r > 0:
                near.append(m[r - 1][c])
            if r < rows - 1:
                near.append(m[r + 1][c])
            if c > 0:
                near.append(m[r][c - 1])
            if c < cols - 1:
                near.append(m[r][c + 1])
            if any(ch in "|-" for ch in near):
                m[r][c] = "+"

    for i, j in arena.start_set:
        m[2 * j + 1][2 * i + 1] = "."
    for (i, j), idx in arena.room_of.items():
        m[2 * j + 1][2 * i + 1] = string.ascii_lowercase[idx]
    for s in arena.survivors:
        i, j = s["cell"]
        m[2 * j + 1][2 * i + 1] = str(s["n"])
    pi, pj = arena.pad_cell
    m[2 * pj + 1][2 * pi + 1] = "P"

    if mx < 0:
        m = [row[::-1] for row in m]
    if my < 0:
        m = m[::-1]
    return "\n".join("".join(row) for row in reversed(m))

def main():
    global CELLS

    ap = argparse.ArgumentParser(
        description="Generate a NIDAR AirMouse arena world for Gazebo.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--name", required=True, help="World name and file stem.")
    ap.add_argument("--out", default=".", help="Output directory.")
    ap.add_argument("--seed", type=int, required=True,
                    help="Layout seed. Same seed + same corridor width "
                         "reproduces the arena exactly.")
    ap.add_argument("--cells", type=int, default=CELLS,
                    help="Grid is --cells x --cells. 13 at the 1.0 m spec "
                         "corridor is the largest square that still fits the "
                         "brief's 15 m cap; widen the corridor and you must "
                         "drop cells to stay under it.")
    ap.add_argument("--corridor", type=float, default=1.0,
                    help="CLEAR corridor width in metres. The brief's floor is "
                         "1.0. Note Nav2 here runs robot_radius 0.35, so a 1.0 m "
                         "corridor leaves only a ~0.3 m band of non-inscribed "
                         "cells down the centre - tight enough that SLAM noise "
                         "can momentarily close it. Build the algorithm at 1.4 "
                         "and prove it at 1.0.")
    ap.add_argument("--rooms", type=int, default=8,
                    help="2x2 m rooms. More than --survivors on purpose: empty "
                         "rooms are what stop a detector scoring by visiting "
                         "every room and declaring a survivor in each.")
    ap.add_argument("--survivors", type=int, default=6,
                    help="Survivors to place, one per room (brief caps it at 6).")
    ap.add_argument("--loop-prob", type=float, default=0.18,
                    help="Chance of removing any given corridor-corridor panel "
                         "after carving, creating junctions and ring routes. 0 "
                         "gives a perfect maze (one route everywhere).")
    ap.add_argument("--start", default="open",
                    choices=("open", "chamber", "porch"),
                    help="What the launch/entry/exit point looks like. 'open' "
                         "is an open bay cut into the corner, continuous with "
                         "the maze on every side that faces one - the drone "
                         "lifts off in clear space, not in a box. 'chamber' "
                         "seals the same bay behind a single doorway. 'porch' "
                         "hangs a sealed 2x2 vestibule OUTSIDE the perimeter so "
                         "the drone flies in through a hole in the wall; that "
                         "is closest to the real event but makes the model two "
                         "cells longer than the maze, which may put it over the "
                         "brief's 15 m. Nothing sticks out in the other two.")
    ap.add_argument("--start-cells", type=int, default=3,
                    help="Side of the recessed start bay, in cells. 3 gives a "
                         "~3.2 m clear square at the default corridor width - "
                         "enough to take off, yaw and get a first full scan "
                         "before entering the maze. Ignored for --start porch.")
    ap.add_argument("--entry-corner", default="ne", choices=sorted(MIRRORS),
                    help="Corner holding the entry/exit and launch pad, north "
                         "= +y, east = +x. 'ne' is top-right in Gazebo and in "
                         "any x-right/y-up plot; RViz TopDownOrtho draws +x up "
                         "and +y left, so use 'se' for top-right on screen "
                         "THERE.")
    args = ap.parse_args()

    CELLS = args.cells

    if args.start_cells < 1 or args.start_cells > CELLS - 3:
        ap.error("--start-cells %d does not leave a maze around it on a %d-cell "
                 "grid; use 1..%d." % (args.start_cells, CELLS, CELLS - 3))
    if args.survivors > args.rooms:
        ap.error("--survivors cannot exceed --rooms: one survivor per room.")
    outer = CELLS * (args.corridor + THICKNESS) + THICKNESS
    if outer > 15.0:
        ap.error("--corridor %.2f over %d cells gives a %.2f m arena; the "
                 "brief caps it at 15 m. Either keep %d cells and cap the "
                 "corridor at %.3f m, or keep the corridor and drop to "
                 "--cells %d."
                 % (args.corridor, CELLS, outer, CELLS,
                    (15.0 - THICKNESS) / CELLS - THICKNESS,
                    int((15.0 - THICKNESS) // (args.corridor + THICKNESS))))

    rng = random.Random(args.seed)
    geom = Geom(args.corridor)
    arena = Arena(rng, args.rooms, args.survivors, args.loop_prob,
                  args.start, args.start_cells)
    if not arena.build(geom):
        raise SystemExit(
            "could not place %d rooms in a %dx%d grid and keep every cell "
            "reachable after 200 attempts - lower --rooms or raise --cells."
            % (args.rooms, CELLS, CELLS))

    mx, my = MIRRORS[args.entry_corner]
    pad_xy = (geom.cx(arena.pad_cell[0]), geom.cy(arena.pad_cell[1]))

    os.makedirs(args.out, exist_ok=True)
    stem = os.path.join(args.out, args.name)

    with open(stem + ".sdf", "w") as f:
        f.write(emit_sdf(args.name, arena, geom, mx, my, pad_xy))

    cmd = ("generate_arena.py --name %s --seed %d --corridor %.2f --rooms %d "
           "--survivors %d --loop-prob %.2f --entry-corner %s --cells %d "
           "--start %s --start-cells %d"
           % (args.name, args.seed, args.corridor, args.rooms, args.survivors,
              args.loop_prob, args.entry_corner, CELLS, args.start,
              args.start_cells))

    def world_cell(i, j):
        wi = i if mx > 0 else CELLS - 1 - i
        wj = j if my > 0 else CELLS - 1 - j
        return "%s%d" % (string.ascii_uppercase[wi], wj + 1)

    px, py = pad_xy
    with open(stem + ".yaml", "w") as f:
        f.write("# Ground truth for %s. Generated, do not edit.\n" % args.name)
        f.write("# Regenerate: %s\n" % cmd)
        f.write("# grid_box: column A..%s runs west->east (+x), row 1..%d runs\n"
                "#           south->north (+y). Cell pitch %.2f m.\n"
                % (string.ascii_uppercase[CELLS - 1], CELLS, geom.pitch))
        f.write("arena:\n")
        f.write("  seed: %d\n" % args.seed)
        f.write("  cells: %d\n" % CELLS)
        f.write("  corridor_width_m: %.3f\n" % args.corridor)
        f.write("  wall_thickness_m: %.3f\n" % THICKNESS)
        f.write("  cell_pitch_m: %.3f\n" % geom.pitch)
        f.write("  maze_size_m: [%.3f, %.3f]\n" % (outer, outer))
        f.write("  outer_size_m: [%.3f, %.3f]   # incl. the start area\n"
                % (outer, outer + 2 * geom.pitch
                   if args.start == "porch" else outer))
        f.write("  start_area: %s\n" % args.start)
        if args.start != "porch":
            f.write("  start_bay_m: %.3f\n"
                    % (args.start_cells * args.corridor
                       + (args.start_cells - 1) * THICKNESS))
        f.write("  room_interior_m: %.3f\n"
                % (ROOM_CELLS * args.corridor + (ROOM_CELLS - 1) * THICKNESS))
        f.write("  net_height_m: %.3f\n" % NET_Z)
        f.write("  entry_corner: %s\n" % args.entry_corner)
        f.write("  launch_pad_xy: [0.0, 0.0]   # world origin, by construction\n")
        f.write("  launch_pad_size_m: %.3f\n" % PAD)
        f.write("  rooms: %d\n" % len(arena.rooms))
        f.write("survivors:\n")
        for s in arena.survivors:
            f.write("  - id: survivor_%d\n" % s["n"])
            f.write("    grid_box: %s\n" % world_cell(*s["cell"]))
            f.write("    xy: [%.3f, %.3f]\n"
                    % (mx * (s["x"] - px), my * (s["y"] - py)))
            f.write("    room: %d\n" % s["room"])
        f.write("empty_rooms: %d\n" % (len(arena.rooms) - len(arena.survivors)))

    with open(stem + ".txt", "w") as f:
        f.write("%s - plan view, north (+y) up, east (+x) right\n" % args.name)
        f.write("%s\n\n" % cmd)
        f.write("P = launch pad (world origin, inside the entry vestibule)\n")
        f.write("digits = survivor, letters = room cell, blank = corridor\n")
        f.write("one cell = %.2f m clear; arena is %.2f m square\n\n"
                % (args.corridor, outer))
        f.write(ascii_plan(arena, mx, my))
        f.write("\n")

    print("wrote %s.sdf / .yaml / .txt  (%d rooms, %d survivors, %.2f m arena)"
          % (stem, len(arena.rooms), len(arena.survivors), outer))

if __name__ == "__main__":
    main()
