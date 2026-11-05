# NIDAR AirMouse competition arenas

Six generated Gazebo worlds built to the Track 1 Mission Brief. **The full guide
is [`../docs/ARENAS.md`](../docs/ARENAS.md)** — launching, scoring survivors,
generating new layouts, and why the arena is shaped the way it is. This file is
just the reference card.

```bash
cd ~/ardu_ws && source install/setup.bash
ros2 launch nidar_sim arena.launch.py arena:=1_wide
```

| arena | grid | corridor | full footprint | rooms | loop-prob | stresses |
|---|---|---|---|---|---|---|
| 1 | 13x13 | 1.0 m | 14.40 x 14.40 m | 8 | 0.10 | dense maze, deep dead ends |
| 2 | 13x13 | 1.0 m | 14.40 x 14.40 m | 7 | 0.20 | balanced corridors + junctions |
| 3 | 13x13 | 1.0 m | 14.40 x 14.40 m | 9 | 0.32 | open plan, many ring routes |
| 1_wide / 2_wide / 3_wide | 9x9 | 1.4 m | 13.60 x 13.60 m | 6 | as above | the same three, with room to fly |

All six sit inside the brief's 15 m x 15 m with nothing protruding. Use the
`_wide` set first: at `robot_radius: 0.35` the 1.0 m spec corridor leaves only a
~0.3 m ribbon of non-inscribed cells down the centre.

Per world: `.sdf` (the world, self-contained - no `model://`, no Fuel, loads
offline), `.yaml` (survivor grid boxes and true xy - the scoring ground truth),
`.txt` (ASCII plan, north up). `arenas.png` / `arenas_wide.png` are the plans
side by side.

**`*.sdf` is generated output.** Edit `../scripts/generate_arena.py` and rerun
`../scripts/make_arenas.sh`; hand edits are discarded by the next regeneration.
