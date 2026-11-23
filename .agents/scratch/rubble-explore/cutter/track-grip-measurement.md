# Track grip on loose pieces: measurement

Subject: a video game's vehicle physics (game state only): where a tracked hull's support and
drive come from when it stands on loose rubble pieces. Measurement only, no law changed. Code at
`7b46ea46` (the capacity fold), built with `--features dev_ui,bitprobe` and a temporary probe,
never committed. It answers the first measurement of `hull-on-pieces-plan.md` (same folder, on
the main checkout). Every number is MEASURED unless marked; two runs a pass; untimed (load
average 4 to 16), so no timing is claimed.

## How it reads

- One line a fixed tick, after the physics step, for the one tank.
- **Solver channels** come from `ContactTransfers::solver_groups`: every solver point on the
  hull, its whole impulse over the step (normal and friction), divided by the step. The hull's
  collider names the channel: soft run (`SoftRun`), bump stop (`BumpStop`), side plate, and the
  hull's collision proxies split by the contact's outward direction (under, top, front, rear,
  side). The other body names what it rests on: a loose piece, standing cells, or static
  (terrain).
- **Sampled channel** comes from `TrackContacts`: each contact's support, traction and rolling
  force, split by what it sampled (the terrain field, standing cells).
- Support is the world-up force as a share of the hull's weight W = 559,170 N (mass times
  9.81, DERIVED from the measured mass). "Friction forward" is the friction part of a channel's
  force along the hull's forward axis.
- "On belts" is the transmission's force on the two belts (the last substep's, each tick):
  what the engine asks the contacts to return.
- Slip is belt speed less hull forward speed, over belt speed.
- For belt-face rows: `|f|/μN` is the sum of friction magnitudes over the sum of coefficient
  times normal force; "sliding" is the belt surface's tangential speed over the other body at
  the point (normal-weighted, velocities at the step's end).
- The channels' forward forces plus gravity along forward reproduce the hull's momentum change
  in every window below to within 2 kN (cruise −4.6 + 4.2 against −0.4 kN; the hold −8.5 + 6.3
  against −1.5 kN), and the supports sum to 95 to 102 % of W: no force on the hull is outside
  these channels.

Passes: `scripts/perf/ram-hall-junction.json` at `20kmh`; the house long wall at 15 km/h
(`smoke-house-15.json`, `house_0046`, from the earlier probe's plans); and the `pile_park.py`
pose (`Drive::Park`, `house_0001`) with a probe lever that holds full throttle from script tick
1920, after 12.5 s parked. Probe, scripts and per-window tables: `target/track-grip/notes/` in
bay-2 (`grip-probe-on-7b46ea46.patch`, `grip.py`, `runs.sh`).

## Where the weight sits, what drives, the slip

Run B of each pass (the second, with the friction readings). Support in % of W; forces in kN.

| pass, window | hull / belt m/s, slip | sampled | soft run on pieces | bump stop on pieces | hull proxies on pieces | on belts | friction forward: sampled / run / stop |
|---|---|---|---|---|---|---|---|
| junction, cruise before the wall (2 s) | 5.42 / 5.42, 0 % | 100.1 | 0 | 0 | 0 | 19.7 | 19.7 / 0 / 0 |
| junction, onto the heap (5.5 s) | 1.04 / 1.07, 2 % | 44.8 | 15.0 | 28.2 (+13.9 on terrain) | −1.0 | 126.8 | 33.8 / 23.3 / 51.7 |
| junction, on the heap (2.75 s) | 1.02 / 1.42, 28 % | 0 | 32.9 | 47.8 | under 22.0, top −4.0 | 157.3 | 0 / 47.9 / 98.3 |
| junction, the hold (last 1.9 s) | 0.18 / 3.07, 94 % | 0 | 6.1 | 11.4 | under 86.2, top −4.1 | 84.5 | 0 / 20.2 / 38.7 |
| house 15, through the wall (2 s) | 3.04 / 3.04, 0 % | 17.4 | 36.6 | 46.9 | top −4.1 | 77.2 | 8.8 / 27.2 / 26.0 |
| house 15, leaving (3.2 s) | 3.94 / 3.94, 0 % | 73.4 | 17.4 | 12.3 | top −2.5 | 29.2 | 22.1 / 4.5 / 3.2 |
| pile, parked (12.5 s) | 0 / 0 | 34.1 | 57.1 | 11.1 | top −2.6 | −34.5 (park) | −37.3 / 4.2 / −1.1 |
| pile, full throttle from rest (3 s) | 1.61 / 1.62 (window mean), 1 % | 28.3 | 56.1 | 17.2 | top −2.6 | 62.0 | 17.4 / 31.2 / −0.5 |

- The parked split in run A: soft run 63.2, stop 7.3, sampled 32.0 (`work-form.md`, "Parked on pieces",
  read 58.6 / 5.9 / 38.1).
- A hull started from rest with 73 % of its weight on pieces drives off them: 4.8 m in 3 s at
  1 % slip, the soft runs delivering the largest share of the drive (31.2 kN of 48.1 kN).
- Crossing the house the belts carry 84 % of the weight on pieces and deliver 53.2 kN of the
  62.0 kN of friction, with no slip.
- Where the friction returned is under what the belts were given, the rest went into the belts'
  own speed and the rows' other directions; it was not read apart.
- Reported capacity (`traction_capacity_n`) parked on the pile: 340.5 kN, of which the sampled
  contacts are 104.0 kN, the figure it reported before the fold.

## The junction: one run creeps, one is held

The two runs of the same recipe part ways on the heap (an observation: the approach is not
bit-repeatable between runs).

**Run A** did not stop in the pass's 10 s. On the heap (last 4.5 s) it crept at 0.9 to 1.05 m/s
with the belts at 1.5 to 1.65 m/s (36 to 39 % slip). Support: bump stops 41 to 51 %, soft runs
27 to 36 %, the hull's underside on pieces 24 to 25 %. The belts returned 162.4 kN of friction
forward of the 169.5 kN they were given. Against it: the underside's rows −82.6 kN, pieces
ahead of the hull's front −82.5 kN, gravity up the heap −44.4 kN; the remainder, −47 kN, is the
hull slowing.

**Run B** is held from 9.9 s after contact: 0.18 m/s under full throttle for its last 1.9 s.
- The hull is high-centred. Its underside proxies carry 86.2 % of W on pieces (482 kN of
  normal force over 8 points a step); the bump stops 11.4 %, the soft runs 6.1 %; pieces riding
  on top −4.1 %; the sampled channel nothing.
- The belts spin at 3.07 m/s and slide over the pieces at 1.9 (stops) and 2.4 m/s (runs). Their
  friction is at its Coulomb limit: `|f|/μN` 0.97 and 0.99, with 93 and 99 % of the normal load
  on points at or over 0.9 of the limit. That limit is small because the load is: μN 74.4 kN
  both sides, 58.9 kN of it forward, for 84.5 kN on the belts.
- What holds: the underside's friction on the pieces it rests on, −62.2 kN forward of a
  −62.5 kN row total, at 0.45 of its own limit (μN 274.9 kN; 9 % of its load on saturated
  points): a static hold with room to spare. Front and top rows add −3.4 kN, side rows +2.1 kN,
  gravity +6.3 kN.
- How it got there: in half-second means the proxies' share went 9.5 %, 67 %, 85 % across
  one second, the hull's height falling 11 cm in the first half of it, with −208 kN on the
  proxies as it came down. Through the 2.75 s before, the pieces under the belts moved rearward
  at 0.17 to 0.23 m/s (normal-weighted, along the hull), and at 0.5 m/s in the hold. DERIVED,
  not read directly: the belts carry pieces out from under themselves until the hull comes
  down on its underside. In the hold the left bump stop carries 10.9 %, the right 0.5 %.

Against the plan's decision table: this is its first row. The belt faces deliver their Coulomb
limit, the underside's hold exceeds it, and each force is one an existing law states. The
drive is not lost at the belt: it is lost with the load.

## What the laws do not explain

Nothing in the force balance. Left to the eye, not to a law found missing:
- Whether a hull should come down onto its underside on a heap this size within 3 s. The
  readings that decide it are the heap's, not the belt's: how fast pieces leave from under a
  slipping belt, and the 20 cm of soft-run overlap a parked hull shows (`work-form.md`, "Parked on pieces").
- The two runs' different ends from one recipe.

## Not verified

- Timing: untimed runs on a loaded machine.
- Two runs a pass; run A has no friction-limit or sliding readings (the probe gained them for
  run B).
- The sliding speed uses velocities at the step's end with the step's opening points.
- A point's friction is its vector sum over six substeps, so `|f|/μN` under 1 can still be a
  point at its limit in each substep with a turning direction.
- "On belts" is the last substep's force each tick, not the tick's mean.
- Whether a braking forecast on pieces now stops where it plans: the one reader of the capacity
  is the scripted battle driver (`tank::battle::drive`), whose lanes cross no rubble in any run
  here.
- `TrackGripEffect::tangential_force`, `tangential_torque` and `belt_reaction` still sum the
  sampled contacts alone; their readers are probes and tests.
