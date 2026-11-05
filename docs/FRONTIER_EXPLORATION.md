# Frontier Exploration — How Our Drone Explores a Building It Has Never Seen

**NIDAR 2026–27 · Track 1, Problem Statement 2 — Air Mouse**
*Autonomous GPS-Denied Indoor Search, Mapping & Survivor Localisation*

---

## 1. The problem we are solving

The Air Mouse arena is an earthquake-damaged indoor maze, up to **15 m × 15 m**,
with corridors, turns, junctions and rooms, and **up to 6 survivors** hidden in
different parts of it. There is **no GPS**. The drone has to go in blind, build
its own map, find every survivor, and mark each one's position on that map so a
rescue team knows which room to go to.

Strip that down and there are three questions the drone must answer, over and
over, on its own:

1. **Where am I?** — with no GPS, nothing tells it. It has to work this out from
   what it can see.
2. **What does this building look like?** — it has to draw its own map while
   moving through it.
3. **Where should I go next?** — and this is the one this document is about.

Question 3 is the exploration problem. A human in an unfamiliar building solves
it without thinking: you walk toward the doorway you haven't been through yet.
Frontier exploration is that instinct, written down precisely enough for a
computer to follow.

---

## 2. The core idea in one picture

While the drone flies, it builds an **occupancy grid** — the map divided into
5 cm squares, each square in exactly one of three states:

| State | Meaning | On screen |
| --- | --- | --- |
| **Free** | The lidar looked through this square. Nothing there. | white |
| **Occupied** | The lidar hit something here. Wall. | black |
| **Unknown** | The lidar has never seen this square. | grey |

Now the key definition:

> A **frontier** is a free cell that touches an unknown cell.

That is the whole idea. A frontier is the **edge of what we know** — standing
there, you are in open space you have already mapped, looking directly at space
you have not. It is exactly the doorway you haven't walked through.

```
  ████████████████████        █ = wall (occupied)
  █ . . . . . █ ? ? ? ?       . = free, already mapped
  █ . . . . . F ? ? ? ?       ? = unknown
  █ . . D . . F ? ? ? ?       F = FRONTIER  <-- free, but touching unknown
  █ . . . . . F ? ? ? ?       D = drone
  █ . . . . . █ ? ? ? ?
  ████████████████████
```

The algorithm is then almost embarrassingly simple:

1. Find every frontier cell on the current map.
2. Group neighbouring ones into clusters (a doorway is one frontier, not forty).
3. Pick the best cluster.
4. Tell the navigation system to fly there.
5. Flying there reveals new space, which moves the frontier further out.
6. Repeat.
7. **When there are no frontiers left, the building is fully explored** — because
   "no free cell touches unknown space" is the literal definition of a finished
   map.

That last point is why we chose this method. It has a **natural, provable
stopping condition**. We do not guess when we are done; the map tells us.

---

## 3. Why not something simpler?

Worth being able to answer, because it will be asked.

| Approach | Why we rejected it |
| --- | --- |
| **Random walk** | Might cover the arena. Might not. No completion guarantee and no bound on time. |
| **Wall following** (keep the right wall) | Fails on any maze with loops or islands — the classic failure is circling one internal block forever and never entering the rest. |
| **Pre-planned lawnmower sweep** | Needs the floor plan in advance. We do not have it; that is the entire challenge. |
| **Frontier exploration** | Needs no prior map, adapts as the map grows, and finishes provably when frontiers run out. |

Frontier exploration is the standard answer in robotics for this exact
situation, first published by **Brian Yamauchi in 1997**. What follows is that
classic idea plus the engineering needed to make it survive on a real flying
drone in a tight maze.

---

## 4. Picking the *best* frontier — the part that actually matters

Finding frontiers is easy. Choosing between them is where exploration succeeds
or fails, and it is where almost all of our engineering effort went.

### 4.1 What we tried first, and why it stalled

Our first rule was additive, and picked the **lowest** score:

```
score = travel_distance − size_weight × cluster_size + penalties
```

It stalled with the map about two-thirds explored. Two structural reasons:

1. **Cluster size is the wrong measure of value.** It says how *wide* an
   opening is, not how much lies *behind* it. A wide doorway into a cupboard
   outscored a narrow one into the rest of the building.
2. **The units don't mean anything.** Subtracting a count of cells from a
   distance in metres is dimensionally meaningless, so `size_weight` had no
   principled value — it was a fudge factor that had to be capped to stop it
   swamping distance entirely.

### 4.2 What we use now: cost–utility

The standard formulation (González-Baños & Latombe): **maximise the
information you expect to gain, discounted by what it costs to go and get it.**

```
utility = information_gain
          × exp( −travel_distance / distance_decay )
          × direction_factor
          × visit_factor                              ← HIGHEST wins
```

Every term is a dimensionless multiplier on one quantity that has real physical
meaning — **unknown cells revealed** — so they compose without any arbitrary
unit conversion.

### 4.3 Information gain — *how much do we actually learn?*

For each frontier we count the **unknown cells within sensor range of it**.
That is an estimate of how much new map we get by flying there.

```
   ████████████          ████████████
   █ . . . . F ? ? ?     █ . . . . F ?█
   █ . . . . F ? ? ?     █ . . . . F ?█
   █ . . . . F ? ? ?     ██████████████
   ████████████
   gain ≈ 400 cells      gain ≈ 8 cells
   (a whole room)        (a cupboard)
```

Identical cluster size. Wildly different value. **This is the single change
that fixed the stall** — near-worthless frontiers can no longer win just by
being close.

It's an upper bound (it ignores occlusion, so unknown cells behind a wall are
counted as if visible). That's fine: it's cheap, and the thing it gets right is
the thing that decides the outcome.

### 4.4 Distance as a discount, not a subtraction

`exp(−distance / decay)` means a frontier twice the decay constant away must
offer **e times** the information to win. This is the key structural
difference: distance now *scales* value rather than being traded against it, so
there is no exchange rate to fudge.

### 4.5 Travel distance — *how far is it, really?*

Straight-line distance is **actively misleading** in a maze. Two points can be
one metre apart with a wall between them and need a 15-metre detour.

```
      D  |  F            Straight line:  1 m   ("it's close!")
         |               Real route:    15 m   (all the way around)
    ─────┘
```

So we compute real travel distance with a **Dijkstra flood-fill** from the
drone's cell, spreading only through connected free space. Diagonal steps cost
√2. This also gives reachability for free: if the fill never arrives, the
frontier isn't reachable and we discard it rather than sending the drone on an
impossible errand.

### 4.6 Direction factor — *stop turning around*

With two frontier groups either side of the drone, **every individual decision
is correct and the behaviour is still broken**:

```
   F_A  ←  D  →  F_B      A is 4.0 m, B is 4.1 m. Picks A, flies left.
   F_A ← D    →  F_B      New frontier appears behind. B now nearer.
   F_A   D →     F_B      Switches to B. Which makes A nearer again.
```

The drone thrashes one junction while the map stops growing. Nothing is wrong
with any single choice — **the loop is the bug**. The cause: the score depended
only on *current* position, and the drone's own movement kept flipping the
winner.

The fix gives the score a **memory of heading** — continuing forwards is
undiscounted, reversing is heavily discounted. The drone sweeps through a
region and leaves it, which is also what covers the map fastest.

### 4.7 Visit factor — *don't go back where you've been*

Every goal the drone *sets out for* is remembered and discounted for a while.
Note: goals **dispatched**, not goals **reached** — a goal we set out for and
failed to reach is precisely the one to stop re-choosing.

### 4.8 Region commitment — now a bonus, not a rule

Frontiers within 4 m get a ×1.5 bonus, so the drone finishes the room it's in
before crossing the arena.

This used to be a **hard gate** — only nearby frontiers were considered at all.
That was itself a way to get stuck: if every nearby candidate was unproductive,
nothing further away could *ever* be considered, and the drone sat re-picking
dead ends. Proximity is already rewarded by the exponential; the bonus
preserves the behaviour without ever making a distant frontier unreachable.

## 5. Keeping it alive — the safety layers

An exploration algorithm that flies into a wall scores zero. Three independent
layers protect the drone, and the layering is deliberate: each one catches what
the one above it misses.

### Layer 1 — Don't choose a dangerous goal
Frontier candidates are filtered against an **inflated** obstacle map: every
wall is grown outward by a safety margin, and any frontier inside that grown
region is discarded. In a corridor narrower than twice the margin this can erase
every candidate, so the margin relaxes in stages — but never below a hard floor.

### Layer 2 — Don't *send* a goal while already too close
Before any goal is dispatched we re-check the **live local costmap**. This
matters because the SLAM map lags: it is republished periodically, and a newly
seen wall needs several scans before it is confidently marked. A wall can be
right next to the drone and still read as "free" on the SLAM map for a second or
two. The local costmap is fed straight from the live laser and does not lag.

### Layer 3 — Continuous collision watchdog
A separate timer runs at **5 Hz from the moment the node starts** — not only
while a goal is active — checking the drone's live clearance. If it drops below
the minimum, the active goal is **cancelled immediately**, without waiting for
the next 3-second planning cycle.

> **Honest scope:** this node does not command motor speeds. It can *ask* the
> navigation stack to stop; it cannot force an instant physical stop. A true
> motion-level emergency stop belongs in the node that owns velocity commands.

### Layer 4 (upstream) — The tilt gate
This one is specific to flying, and is worth highlighting because it is the kind
of bug that does not exist on ground robots at all.

**A 2D lidar assumes it stays level. A multirotor cannot.** It has to lean to
accelerate, brake and turn. While it is leaning, beams that should graze along a
nearby wall angle over or under it and come back with **no return** — and both
SLAM and the costmap interpret "no return" as "clear", **erasing a wall they had
already mapped correctly.**

```
   Level:                      Tilted:
     ╔═══                        ╔═══
   D ────►  hits wall ✓       D╱ ─ ─ ─►  passes over the wall ✗
     ╚═══                        ╚═══     → "no obstacle here"  → wall deleted
```

This is why the map appeared to "glitch" — walls flickering in and out. No
amount of reading data *faster* fixes it, because the problem is not stale data;
the fresh data is itself wrong for as long as the airframe is leaned over.

Our fix: a **tilt gate** sits between the lidar and everything downstream. While
level, scans pass through untouched. While tilted beyond 8°, it republishes the
**last known-good scan, unchanged, including its original timestamp** — so
downstream TF places that range data at the pose where it was actually captured.
A real wall stays marked through the excursion.

The trade-off, stated plainly: we get no *new* information while tilted. That is
the correct trade. An honest "I have no fresh data right now" is safe;
forwarding corrupted data that pretends to be fresh is not.

### The mistake we made with tilt — worth knowing

Our first attempt also made the **planner** react to tilt: if the drone was
leaning past 8°, cancel the goal, because clearance readings could not be
trusted.

That deadlocked the drone completely, and the reason is pure physics. **A
multirotor can only accelerate by leaning.** The lean required is
`tilt = atan(a / g)`:

| Horizontal acceleration | Lean required |
| --- | --- |
| 1.0 m/s² | 5.8° |
| 1.5 m/s² | **8.7°** |
| 3.0 m/s² (our Nav2 limit) | **17.0°** |

So an 8° ceiling was crossed by the drone's own **normal departure from every
single goal**:

```
send goal → drone accelerates → tilt > 8° → cancel goal
          → drone levels out  → send goal → accelerates → cancel → ...
```

It never completed one leg. The map never grew. From outside it looked exactly
like *"the algorithm is stuck and does nothing"* — with no error anywhere,
because every individual step did precisely what it was written to do.

**The error was conceptual, not numerical.** Tilt does not mean "there is a
hazard". It means "my lidar is not level, so my clearance numbers are
temporarily untrustworthy". The correct response to untrustworthy data is to
**abstain from acting on it** — not to cancel the very motion producing it,
which is just normal flight.

So the planner now recognises three states, not two:

| State | Condition | Response |
| --- | --- | --- |
| **Level** | under 20° | Trust clearance readings normally |
| **Transient** | over 20°, under 1.5 s | **Abstain.** Ordinary manoeuvring — cancel nothing |
| **Sustained** | over 20° for more than 4 s | **Cancel.** A healthy drone does not hold a big lean |

The threshold also moved from 8° to **20°**, above the 17° that normal
acceleration produces. The upstream tilt gate still protects the *map* at 8° —
that is the right place for it, because holding a scan costs nothing. Making
the *planner* refuse to act was what caused the paralysis.

---

## 6. Knowing when to stop — and when *not* to

The algorithm ends when no frontier has been viable for several consecutive
cycles. But we are careful about two ways that conclusion can be wrong:

**"Finished" vs "never started."** If the drone declares completion without ever
having moved, that is not a finished map — it means every goal near the start
point was rejected until none were left, almost always a mismatch between the
configured drone size and the actual clearance at the spawn point. We detect
this and say so explicitly, rather than reporting a successful finish.

**Never stop irreversibly.** Completion used to cancel the planning timer
outright, which made the conclusion permanent — if it was wrong, or if SLAM
revealed new space later, the drone could never notice and simply sat there.
It now drops to a slow idle re-check and resumes by itself if frontiers
reappear. A conclusion this consequential should always be revisable.

**The final sweep.** All three anti-oscillation mechanisms (blacklists, revisit
penalties) *suppress* candidates, and together they can hide a region that is
perfectly reachable — which looks identical to being done. So before declaring
completion we **clear every suppression list and do one more honest pass**. Only
if that also comes up empty is the map actually complete.

---

## 6a. Checking the goal itself, not just the robot

Every guard described above checks whether the *drone* is safe — live
clearance, tilt, stall detection. None of them asked whether the *goal* was
safe, and the only thing filtering a goal's location was `/map_filtered`:
one SLAM update stale, thickened by the grid resolution, and blind to Nav2's
own inscribed collision ring. The frontier margin also relaxes under
pressure (down to a hard floor that sits only a few cm outside that ring),
so a goal could be sent to a spot Nav2 was structurally certain to reject.

Two additions close that gap:

- **The global-costmap veto.** Before dispatch, the explorer reads the cost
  of the goal cell directly from Nav2's own global costmap — the actual grid
  the planner routes on. A cell reading at or above the inscribed threshold
  is refused and blacklisted in the same cycle, instead of being sent and
  failing four seconds later.
- **Independent arrival confirmation.** Nav2's own "SUCCEEDED" and this
  node's sense of "arrived" are two numbers living in two different config
  files (`nav2_params.yaml`'s goal tolerance, and this node's own
  `goal_arrival_tolerance_m`). They have drifted apart before — which looked
  exactly like "the explorer moves to the next goal before the drone is
  anywhere near the current one." On every reported success, the explorer
  now measures the real distance to the target and logs loudly if the two
  numbers disagree, instead of silently trusting Nav2's word.

---

## 7. Reporting progress

Two numbers, live, on the map display and on a ROS topic:

- **Area explored (m²)** — this only ever goes up. It is the honest progress
  number.
- **% of mapped area known** — useful, but note the denominator *grows* as the
  SLAM map expands, so this figure can **fall while real progress is being
  made**. It is not "% of the arena".

We show both deliberately, because showing only the percentage invites exactly
the wrong conclusion at exactly the wrong moment.

---

## 8. Where this sits in the full system

```
   Lidar  ──►  Tilt Gate  ──►  SLAM (slam_toolbox)  ──►  map + drone position
                   │                                          │
                   │                                          ▼
                   └──────────────────────►  Nav2  ◄──  FRONTIER EXPLORER
                                              │              (this document)
                                              ▼
                                       velocity commands
                                              │
                                              ▼
                                      cmd_vel bridge
                                              │
                                              ▼
                                      ArduPilot  ──►  motors
```

The frontier explorer is deliberately **small and narrow**. It does one job:
decide where to go next, and hand that as a goal to the navigation stack. It
never computes a path and never commands a motor. Every other component is
replaceable without touching it.

---

## 9. Thirty-second summary

> A frontier is the boundary between mapped free space and the unknown. Our
> drone repeatedly finds every such boundary and picks the one offering the
> most **new map per metre flown** — information gain, discounted
> exponentially by real wall-aware travel distance, and further discounted for
> doubling back or revisiting. Each trip pushes the boundary outward. When no
> frontiers remain, the building is provably fully mapped. Three independent
> safety layers plus a lidar tilt gate keep it from flying into anything.

**The one-line version of what we learned:** measuring how *wide* an opening
is tells you nothing about how much is *behind* it — and that distinction was
the difference between stalling at two-thirds coverage and finishing.

**Reference:** B. Yamauchi, *"A Frontier-Based Approach for Autonomous
Exploration"*, IEEE CIRA, 1997. · H. González-Baños & J.-C. Latombe,
*"Navigation Strategies for Exploring Indoor Environments"*, IJRR, 2002
(the cost–utility formulation in §4.2).
