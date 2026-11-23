# The fork's contact solve and the heavy-through-light case: facts from source

Subject: the game's server physics, rigid-body contact between a heavy hull and light loose
pieces (game state only). Code study, 2026-11-23. Nothing was built or run in the engine.

Tree read: `.claude/worktrees/bay-1`, branch `lane/work-form` at `52c3c995`.
`F/` = `vendor/avian3d-0.8.0-overmatch/src/`. Game paths are relative to the bay.

Labels. **DERIVED**: read from the cited lines, nothing run. **DERIVED (arithmetic)**: the
recurrence derived from those lines, evaluated in a throwaway pure-Python script (four corner
points, two cubes and a slab, the fork's solve order and coefficients); it is not the engine.
**MEASURED**: quoted from `.agents/scratch/rubble-explore/cutter/work-form.md`, "One budget: fix
pass", R1b (bay-1), unless another source is named.

## Summary

1. The solve is 6 substeps of one biased sweep and one relaxing sweep: 12 Gauss-Seidel sweeps
   a step, no iteration count to raise. Contacts are soft: 48 Hz against a dynamic body, 96 Hz
   against a static one, damping ratio 10. Every value is Avian's default; the game sets none.
2. Order is a greedy colouring kept from step to step. Pairs of two non-static bodies are solved
   before a body's first three static manifolds. Nothing orders by mass, depth or distance.
3. A row's gain is its effective mass, which for a heavy body on a light one is about the light
   body's mass. With the light body held by something behind it, each sweep passes on at most
   about `m_light × (closing speed + push-out speed)`. The accumulated impulse leaks 5.8 % a
   substep, so it has a ceiling proportional to the light mass. The recurrence reproduces the
   fixture's numbers (35.5 / 84.8 / 100 against MEASURED 35.5 / 82.5 / 100; 11.8 … 81.6 against
   11.8 … 84.3).
4. The heavy body's lost energy is the transmitted impulse times its closing speed, dissipated
   in the row between the two dynamic bodies. It is not stored and does not reach the holder.
5. The belt's soft run is not limited this way in what it finally carries (its spring is in
   absolute units), only in how fast it gets there. The rigid stop and the hull proxies are.

## 1. The solve, step by step

All DERIVED.

**Step and substeps.** The tick is 15.625 ms (`src/net/server.rs:45`). `SubstepCount` defaults to
6 (`F/dynamics/solver/schedule.rs:191-195`); no non-test game code inserts it (the inserts in
`src/track/sim.rs:2153,2263,2472` and `src/headless_test/track_scorecard.rs:3484` are under
`cfg(test)` or the headless harness). Substep `h` = 2.604 ms. The game composes physics in
`src/lib.rs:911-916` and `src/net/physics.rs:31-37`: it sets `recycle_distance: 0.0` and adds
`avian_ext::plugin`; it writes no `SolverConfig` field outside tests.

**Before the substeps** (`F/dynamics/solver/schedule.rs:32-47`): prepare solver bodies, joints,
contact constraints; then the game's `compliance::apply_laws` and `reaction_bound::locate`
(`src/avian_ext/compliance.rs:105-107`, `reaction_bound.rs:258-261`).

**Each substep** (`schedule.rs:60-70`, single-threaded executor):

1. `IntegrationSystems::Velocity`: forces, gravity, damping (`F/dynamics/integrator/mod.rs:373`).
   The game's `reaction_bound::deal` runs between this and the warm start
   (`reaction_bound.rs:276-277`).
2. Warm start: every point's accumulated normal and tangent impulse is applied again, times
   `warm_start_coefficient` = 1.0 (`F/dynamics/solver/plugin.rs:766-797`,
   `contact/mod.rs:263-315`).
3. Biased solve: `solve_contacts::<true>`, **one sweep** (`plugin.rs:161`, `852-902`). Normal
   rows only; ordinary friction does not run here (`contact/mod.rs:379-380`).
4. `IntegrationSystems::Position`: `delta_position += v·h` (`integrator/mod.rs:546-581`).
5. Relax solve: `solve_contacts::<false>`, **one sweep**, normal rows without bias and
   softness, then friction (`plugin.rs:165`, `contact/mod.rs:190-194`, `380-414`).
6. XPBD joints and joint damping, after Relax (`F/dynamics/solver/xpbd/plugin.rs:33-42`).

A row reads the current separation each solve from the bodies' accumulated substep motion
(`contact/mod.rs:333-342`), so depth grows inside a step as bodies move.

**After the substeps**: restitution, one iteration, threshold 1 m/s, skipped for a manifold of
restitution 0 (`plugin.rs:958-1054`; `Restitution` defaults to 0,
`F/dynamics/rigid_body/physics_material.rs:324-331`); swept CCD; write-back; impulses stored for
the next step's warm start (`plugin.rs:1058-1091`).

**Sweeps.** `SolverConfig` has no iteration field (`plugin.rs:515-601`); the doc says substeps
are the iteration (`plugin.rs:841-842`). 12 normal sweeps a step.

**Softness** (`plugin.rs:625-649`, `F/dynamics/solver/softness_parameters/mod.rs:64-79`).
`SolverConfig` defaults (`plugin.rs:590-601`): `contact_damping_ratio` 10,
`contact_frequency_factor` 1.5, `max_overlap_solve_speed` 4 m/s, `warm_start_coefficient` 1,
`restitution_threshold` 1, `restitution_iterations` 1. Frequency is
`1.5 × min(1/(2·dt), 0.25/h)` = 1.5 × min(32, 96) = **48 Hz** against a dynamic body and twice
that, **96 Hz**, against a static or kinematic one or under dominance (`plugin.rs:637-647`,
`contact/mod.rs:181-185`). With `a1 = 2ζ + ωh`, `a2 = ωh·a1`, `a3 = 1/(1+a2)`:

| | ω rad/s | bias `ω/a1`, 1/s | mass scale `a2·a3` | impulse scale `a3` |
|---|---|---|---|---|
| dynamic, 48 Hz | 301.6 | 14.51 | 0.9423 | 0.0577 |
| non-dynamic, 96 Hz | 603.2 | 27.96 | 0.9713 | 0.0287 |

(DERIVED from the formulas at 64 Hz and 6 substeps.)

**The normal row** (`F/dynamics/solver/contact/normal_part.rs:143-206`), `vn` the separating
speed, `s` the separation (negative in overlap), `λ` the point's accumulated impulse, `em` its
effective mass:

- `s > 0`, speculative: `Δλ = −em·(vn + s/h)`.
- overlap, biased: `Δλ = −mass_scale·em·(vn + max(bias·s, −4 m/s)) − impulse_scale·λ`.
- overlap, relax: `Δλ = −em·vn`.
- then `λ` is clamped to `[0, max_impulse]` (the upper side is the fork's).

`em = 1 / (1/m1 + 1/m2 + angular terms)` (`normal_part.rs:118-130`). The impulse is applied
with the bodies' own inverse masses (`contact/mod.rs:372-376`).

The push-out speed `bias·s` reaches the 4 m/s clamp at 27.6 cm of overlap against a dynamic
body and 14.3 cm against a static one (DERIVED: 4 / bias).

Speculative margin: opt-in per body (`SpeculativeCcd`,
`F/collision/narrow_phase/system_param.rs:776`), floored at `contact_tolerance` 0.02 m
(`F/collision/narrow_phase/mod.rs:224`; fork `OVERMATCH_PATCH.md`, "Minimum speculative margin").

## 2. Order

All DERIVED from `F/dynamics/solver/constraint_graph.rs` and `plugin.rs`.

- **One edge per manifold.** Each manifold of a pair is a constraint with its own colour
  (`constraint_graph.rs:136`, `209-287`). 24 colours; index 23 is overflow (`:55-64`).
- **Two non-static bodies**: the first of colours 0..19 holding neither body (`:224-239`).
  Kinematic bodies count as non-static here (the flag is `rb.is_static()`,
  `system_param.rs:585-586`).
- **One static body**: colours tried from 22 down to 1, the first not holding the dynamic body
  (`:240-264`). So a body's first three static manifolds sit in 22, 21, 20, which no
  dynamic pair can take; its fourth and later share colours 19..1 with dynamic pairs.
- **Overflow**: a manifold that finds no colour (a body already in 20 dynamic colours, or in
  all of 1..22 for a static one) goes to colour 23, solved serially and **first**
  (`plugin.rs:867-878`).
- **Sweep order**: overflow, then colours 0, 1, … 22 (`plugin.rs:881-895`). Warm start and
  restitution use the same order.
- **Within a colour**: push order, changed by `swap_remove` on removal
  (`constraint_graph.rs:320-334`); solved in parallel from 64 constraints up
  (`plugin.rs:887`, `F/utils.rs:141-152`). No two constraints of a colour share a non-static
  body, so the order inside a colour cannot change a result. Overflow order can.
- **Within a constraint**: its points in manifold order, sequentially (`contact/mod.rs:336`).
- **Stability**: a manifold keeps its colour while its pair generates constraints and its
  manifold count does not drop; a count change pops the pair's last handle or pushes a new one
  (`plugin.rs:340-360`). The assignment is a function of contact history, not of the step's
  state.
- **Nothing orders by body, mass, depth or distance.** The only priority is static-late and
  overflow-first.

Consequence for a chain (DERIVED): hull–piece rows are solved before that piece's rows on
standing cells or terrain (while the piece has at most three static manifolds). Along a chain
of pieces the colour order is whatever order the contacts began in; where it runs against the
chain, an impulse advances one link a sweep.

Standing constructions are static bodies (`src/construction.rs:332`); terrain is static
(`src/terrain_grid.rs:1623`); pieces and hulls are dynamic.

## 3. Why a heavy body sinks into a light one

### The recurrence (DERIVED)

Heavy body H (mass M), light body L (mass m ≪ M), holder S behind L. Rows: H–L with dynamic
softness and `em₁ ≈ κ·m` (the reduced mass is about m; κ is the points' lever-arm factor, of
order 1); L–S after it in the sweep. H closes on L at speed `c`; `d` is H's overlap into L.

One sweep:

1. H–L finds an increment proportional to `em₁`. It changes L's velocity toward S by `Δλ/m`
   and H's by `Δλ/M`, which is nothing.
2. L–S removes L's velocity toward S. While S answers (below any bound), L is at rest again.

H–L never sees a closing speed above `c`, because L does not stay moving. So per substep:

```
λ₁ ← (1 − is)·λ₁ + em₁·[ (1 + ms)·c + ms·min(β·d, v_max) ]
     biased sweep: ms·em₁·(c + min(β·d, v_max)) − is·λ₁      relax sweep: em₁·c
```

with `ms` 0.9423, `is` 0.0577, `β` 14.51 1/s, `v_max` 4 m/s. `λ₁` is the impulse the contact
applies each substep (the warm start reapplies it whole). Its ceiling:

```
λ₁* = em₁·[ (1 + 2·a2)·c + a2·min(β·d, v_max) ]          a2 = ωh(2ζ + ωh) = 16.33
    = em₁·[ 33.65·c + 16.32·min(14.51·d, 4) ]   N·s a substep
F*  = em₁·[ 12,920·c + 90,960·min(d, 0.276) ]   N        (90,960 = ω²)
```

- **Linear in the light body's mass**, independent of M and of how strong S is.
- The gap to the ceiling closes by 5.8 % a substep, 30 % a step: a time constant of about 2.9
  steps (45 ms) whatever the masses.
- At rest (`c` = 0) the row is a spring of stiffness `Σem·ω²`, about 91 kN/m per kg of
  effective mass, that stops stiffening at 27.6 cm: it never carries more than about
  25 kN per kg of effective mass.
- Every `SolverConfig` value scales the bracket; none removes the factor `em₁`.

### Against the fixture

Fixture (`src/tests/avian_reaction_bound.rs:119-203`, `534-569`): 1 m cubes, angular inertia
`0.25·m`, so a corner point has `em = m/3` and one four-point sweep passes `89/81·m·c` (κ ≈ 1.1;
at rest the four points share and κ = 4/3). Driver 2,000 kg, both at 4 m/s, the piece's slab
points in a group of 100 N·s a step: 16.67 N·s a substep.

| piece | closed form (κ 1.1) | DERIVED (arithmetic), N·s a step | MEASURED |
|---|---|---|---|
| 0.2 kg | ceiling 178 a step at `d` = 0; bound reached after ~14 substeps | 35.5, 84.8, 100 | 35.5, 82.5, 100 |
| 0.067 kg | ceiling 59 a step at `d` = 0, 88 at the push-out clamp: under 100 | 11.8, 28.5, 41.3, 51.8, 60.8, 68.8, 76.0, 81.6 | 11.8, 27.6, 40.7, 51.6, 61.0, 70.9, 78.5, 84.3 |
| 2 kg | ceiling 1,780 | 99.8 from step 1 | on time |

Also DERIVED (arithmetic) against MEASURED for 0.2 kg: driver speed 3.983 then 3.940 m/s (3.98,
3.94); driver's overlap into the piece 6.2 then 10.8 cm (6.2, 10.9); piece speed after step 1
−0.238 m/s (−0.24); driver's loss 138 then 337 J (138, 327). For 0.067 kg the driver is 35.7 cm
into the piece after step 8 (36 cm). First-step impulse scales with mass: 35.5 / 11.8 = 3.0 =
0.2 / 0.067.

The fixture's threshold (DERIVED): the bound of 100 N·s a step is reachable at `d` = 0 for a
piece above about 0.11 kg (ratio 18,000) and at full push-out above about 0.076 kg
(ratio 26,000). MEASURED: late at 10,000, never at 30,000.

Step 2 of the 0.2 kg arm differs by 3 % (84.8 against 82.5) and the piece's speed after step 2
differs (1.57 against −0.01 m/s). The script has no work budget and no limiter; I did not
establish that this is the cause.

**Order alone does not change it** (DERIVED, arithmetic): solving the slab row before the
driver row gives 37.6, 89.6, 100 for 0.2 kg. The overlap moves from driver–piece (0.6 cm after
step 1) to piece–slab (5.9 cm).

### Where the heavy body's energy goes

DERIVED. Each substep the H–L row applies `λ₁` to H while H still closes on L at `c`, and L,
held by S, has no net motion. The row's work is `−λ₁·c`, and nothing receives it: S is static,
L's kinetic energy does not rise. A rigid contact does no work because it applies force at zero
closing speed; this one applies force while closing, as a damper does (ζ = 10 makes the soft row
damper-dominated, and the relax sweep adds as much again).

Check against MEASURED: 35.5 N·s × 3.98 m/s = 141 J (138 J lost); 82.5 × 3.94 = 325 J (327 J).

It is not stored as recoverable overlap. The bias later removes the overlap at up to 4 m/s, and
that push acts through `em ≈ m`, so it moves the light body, not the heavy one. MEASURED: once
the slab gave, the 0.2 kg piece was thrown ahead at 2.4 m/s (step 5).

## 4. Levers the fork already has

DERIVED unless marked.

| Lever | Where | Exists | Effect on heavy-through-light |
|---|---|---|---|
| Substep count | `schedule.rs:189`, resource | yes, global | More sweeps a step; the `c/h` term of `F*` grows with it, the spring term `em·ω²` does not. Costs every body. |
| `contact_frequency_factor`, `contact_damping_ratio`, `max_overlap_solve_speed` | `plugin.rs:515-601`, resource | yes, global | Scale `a2`, `ω²` and the clamp depth. All still times `em`. The code comment bounds the frequency by Nyquist (`plugin.rs:635-638`). |
| Per-point softness | `ContactNormalPart::softness` is `pub` (`normal_part.rs:31`); writable between `PrepareContactConstraints` and `PreSubstep`, as `apply_laws` does | mechanism yes, no game writer | A pair's own ω and ζ with no fork line. Same form, still times `em`. |
| Compliant row | `normal_part.rs:155-169`, `src/avian_ext/compliance.rs` | yes | Spring in absolute N/m: final force independent of mass, no 4 m/s clamp. Gain per solve `m/(m + h(c + hk))`, so a light body converges slowly (section 5). |
| `max_impulse` clamp, `deal` | `normal_part.rs:198`, `reaction_bound.rs:557-608` | yes | Only lowers what a point returns. |
| `NORMAL_LIMIT` hook | `contact/mod.rs:49-53`, `359-367` | yes | Called after each normal solve of a point that names a limiter, with the part and both solver bodies mutable; the solve applies whatever increment it returns. It can set that point's impulse to anything and write its two bodies. It cannot see a third body or another constraint, change the masses the impulse is applied with, or change order. One process-wide function. |
| `SURFACE_FRICTION` hook | `contact/mod.rs:32-42`, `251-260`, `305-314`, `415-424` | yes | Called at warm start and after both normal passes, with the whole constraint and both bodies mutable, for constraints with a bound surface coordinate; it replaces ordinary friction there. Same reach and same limits as above. |
| Collision hooks | `F/collision/hooks.rs:164-189`; game `src/track/stop_contacts.rs:170` | yes | Narrow phase only: reject a pair, drop or edit manifolds and points (normal, depth, friction, restitution, tangent velocity). No access to masses, softness or impulses. |
| Dominance | `F/dynamics/rigid_body/mod.rs:654`, `contact/mod.rs:157-185`, `plugin.rs:829-833` | yes; the game uses none | The dominant body takes no impulse from the row and the row uses the 96 Hz softness. `em` stays about m, so the ceiling stays; the hull would stop losing momentum to pieces at all, and would not feel what holds them. |
| Per-contact mass or inertia scaling | — | **no** (no such field; the impulse is applied with the bodies' inverse masses, `contact/mod.rs:328-331`) | — |
| Per-body solver settings | — | none beyond `Dominance`, `LockedAxes`, `SleepThreshold`, CCD and margin components | — |
| Joints | `F/dynamics/joints/` (fixed, revolute, spherical, prismatic, distance), XPBD, after Relax each substep, outside the colouring (`constraint_graph.rs:95`) | yes; no game code uses one | A piece jointed to its pusher would still meet the holder through a row of `em ≈ m`. What the joint then returns to the pusher in one substep: not determined by reading. |
| Attached colliders | `ColliderOf` children of one body (how the hull's proxies and runs are built) | yes | A collider on the hull's body is the hull's mass. Whether any game code rehomes a loose piece's collider onto another body (`src/debris/freeze.rs` was not read): not checked. |
| Islands and sleep | `F/dynamics/solver/islands/`, `src/avian_ext/island_wake.rs`, `rest_test.rs` | yes | A sleeping body has no solver body. Islands hold which non-static bodies are linked by touching contacts; static bodies are not members (`islands/mod.rs:147-155`). |
| Contact reduction | `src/avian_ext/contact_reduction.rs` | yes, piece–piece only (`ReducibleContacts`, `src/construction/breakage.rs:716`) | Fewer rows between two pieces; no change to `em`. |
| Collision layers | `src/lib.rs:614-634` | yes | Pieces (`Debris`) meet `Terrain`, `Vehicle`, `Debris`, `TrackBelt` (`breakage.rs:2964-2974`): terrain and standing cells, hull proxies and stops, other pieces, soft runs. Not side plates. |

## 5. Where the game's own laws already bypass the solver

### The building model

DERIVED from `src/construction/fracture/bound.rs`, `src/construction/breakage/bounded.rs`,
`src/avian_ext/reaction_bound.rs`.

- **What it reads.** `Standing` (`bound.rs:151-158`): one structure's joint graph, its live
  state (which joints are separated or cracked, which chunks stand), the chunk shapes, the
  gravity preload on each joint, and the work each chunk's bonds have taken.
- **"What stands behind this contact."** `plug` (`bound.rs:200-233`) walks from the pressed
  cells through every touching joint the push closes more than it slides along (cos 45°), to a
  free face, or to ground or buried matter (`grounded`: nothing leaves that way). `boundary`
  (`:246-300`) sums the plug's boundary joints into a `Price`: a force and a work. `shed`
  (`:307-346`) prices leaving sideways.
- **How it reaches the solver.** `bound_contacts` (`bounded.rs:402-470`) takes every active
  pair between a dynamic body and a standing owner's collider, groups the points by connected
  pressed cells, and hands `Bounds` a group per push (`price.force × dt`, N·s a step) and one
  work budget per block. `deal` splits a group's bound over its points before every substep;
  the limiter cuts a point whose work would pass the budget.
- **Who presses is not read** (`bounded.rs:4-7`). A piece on a block is priced from the piece's
  own velocity and applied acceleration (`:275-310`), and the budget counts work at the piece's
  points from the piece's closing speeds (`reaction_bound.rs:435-503`). A stalled piece closes
  at no speed and counts nothing, whatever presses it. That is R1b.
- **The belt on standing matter** has no solver point: its force is staged and rationed by the
  same price before the step (`bound_belts`, `bound.rs:76-94`, `371-383`).

What loose pieces lack for the same accounting:

1. **A graph.** Standing cells have authored joints with an area, a normal and a strength. Loose
   pieces have only this step's contact pairs (`ContactGraph`: manifolds, normals, last step's
   impulses) and their island.
2. **A price.** A joint's strength and fracture energy are data. A contact between two loose
   pieces has friction and the pieces' inertia, and no law in the tree prices it.
3. **A fixed frame.** `plug` runs in one structure's frame over cells that do not move. Pieces
   move every substep, so "behind along the push" is only true for the step it is read in.
4. **A single mover.** A standing cell is static: the law is the only thing that decides whether
   it moves. A loose piece is already moved by the solver, so a second account of the same
   push would have to replace the piece's rows or be reconciled with them.
5. **The far end.** The chain ends in something that holds: terrain (unbounded), a standing
   block (a `Price`), or nothing (a free heap, which gives by inertia and friction). `plug`
   knows only the first two, and only inside one structure.

### The track law and loose pieces

DERIVED from `src/track/soft_run.rs`, `bump_stop.rs`, `stop_contacts.rs`, `rubble.rs`,
`src/avian_ext/compliance.rs`.

- **Three hull surfaces meet a piece.** The soft run (`TrackBelt`, meets `Debris` only,
  `soft_run.rs:89-91`): compliant row. The bump stop and the hull's collision proxies
  (`Vehicle`, `bump_stop.rs:219-228`, `src/tank/template.rs:1043-1060`): rigid Avian rows; the
  stop also carries the moving-face friction hook. Side plates never meet a piece.
- **Soft-run parameters.** Each link carries the sampled support's own law per metre times its
  length, at its deepest overlap, shared among the manifolds on it; a common `rest` length makes
  the manifold's law affine (`soft_run.rs:383-472`). The per-metre law is computed at startup
  from the tank's data: stiffness = weight over the rest pose's contact area, damping from the
  ride frequency and damping ratio (`src/track/envelope.rs:179-186`), and logged
  (`src/track/sim.rs:742`). **Shipped values not determined by reading.** Test fixtures use
  0.4 and 2 MN/m per metre.
- **The compliant row** (`compliance.rs:54-71`, `normal_part.rs:162-169`): coefficients from
  absolute `k`, `c` at the point's own `em`: with `a = h(c + hk)`, bias `k/(c + hk)`, mass scale
  `a/(em + a)`, impulse scale `em/(em + a)`. Solved alike in both sweeps, uncapped by
  `max_overlap_solve_speed`.
- **Vertical, soft run.** With the piece on something that holds, each solve moves the row's
  impulse the fraction `em/(em + a)` of the way to `h·(k·depth + c·closing)` (DERIVED, same
  argument as section 3). The final force is the spring's, whatever the mass ratio; the row
  has no ceiling. The rate does depend on the light mass. At the soft-run test fixture's law
  (0.4 MN/m and 40 kN·s/m per metre, 0.125 m links: `a` = 13.4 kg) the gap left after one step
  of 12 solves is 2 % for `em` 5 kg, 19 % for 2 kg, 84 % for 0.2 kg. Not evaluated at the
  shipped law.
- **Vertical, rigid rows** (the stop at full bump travel, a hull proxy on a heap). The section 3
  ceiling applies. DERIVED (arithmetic), a heavy cube at rest under gravity on a light cube on
  static ground:

  | light | heavy | settles at | |
  |---|---|---|---|
  | 5 kg | 2,000 kg | 3.2 cm | `W / (Σem·ω²)`, Σem = 4m/3 |
  | 2 kg | 2,000 kg | 8.1 cm | |
  | 50 kg | 57,000 kg | 9.3 cm | |
  | 500 kg | 57,000 kg | 0.9 cm | |
  | 5 kg | 57,000 kg | does not settle | 1.9 m in after 1 s and falling at 4 m/s; needs 92 cm, past the 27.6 cm clamp |

  So a light piece alone under a rigid hull surface carries at most about 25 kN per kg of its
  effective mass. In play the ground under the same belt carries the hull through the sampled
  support, which is a force and no solver row (`soft_run.rs:25-27`, `rubble.rs:13-20`), so
  this is the load a piece is asked to carry beyond that, not the hull's weight.

## 6. Hull against hull

DERIVED.

- **Colliders that meet**: collision proxies (convex hulls, `Vehicle`, filter all,
  `template.rs:1043-1060`), bump stops (`Vehicle`, filter all), side plates (`TrackSide`, which
  meet `Default`, `Vehicle`, `Armor`, `TrackSide`, `bump_stop.rs:177-188`). Soft runs do not
  meet a hull.
- **The belt on another hull** is not a solver row: the sampled support bears on it and hands it
  the opposite reaction as an impulse (`src/track/rubble.rs:13-20`).
- **Mass ratio**: each hull's mass is its tank's data (`template.rs:179`, `341`); two of one
  kind are 1:1, `em` about half a hull, so the row's stiffness `em·ω²` is of order 10⁹ N/m and
  the section 3 limit does not bind.
- **Special handling**: none for the pair as such. The `BeltFace` hook filters stop and plate
  manifolds and sets their surface velocity and friction (`stop_contacts.rs:170-216`). Contact
  reduction does not apply (hulls are not `ReducibleContacts`). No dominance.

## Not determined by reading, and what would settle each

1. **The shipped soft-run law** (stiffness and damping per metre, link length). Read the
   startup log line `envelope law: …` for the shipped tank.
2. **What a standing block's bound is in play** (`price.force × dt`, N·s a step) against the
   section 3 ceiling for a 2 to 6 kg piece at driving speeds. The ram instrument's `BlockRow`
   already carries `price` and `returned`; it does not read the row between two dynamic bodies
   (MEASURED gap, R1b's last sentence).
3. **Which hull surface drives pieces into a wall in play**: soft run (compliant) or proxy and
   stop (rigid). A census of hull–piece manifolds by hull collider kind during a wall pass.
4. **Colour order in real heaps**: how many hull–piece manifolds overflow (more than 20 on one
   hull), and how often a piece's static rows fall in colours below 20. `solver_cost` already
   reads constraint counts per colour.
5. **Why step 2 of the 0.2 kg arm differs by 3 %**: the same fixture with no budget.
6. **XPBD joint between a pusher and a pressed piece**: what it returns to the pusher in one
   substep. A two-cube fixture with a `FixedJoint`.
7. **The soft run's lag for sub-kilogram pieces** at the shipped law: the same pressed fixture
   with a compliant upper row.

## Levers that look relevant to a design discussion

Listed, not proposed.

- The ceiling is `em × (…)` and `em` is the light body's mass in every existing row form.
  Global settings and per-point softness only scale the bracket.
- Mechanisms that reach inside the solve and already exist: the two process-wide hooks, which
  can set a point's impulse and its two bodies' velocities under the colouring's ownership. They
  see one constraint at a time.
- Mechanisms that sit outside the solve and already exist: the staged belt force (a force with
  no row, bounded by a price before the step), and the group and budget store, which is keyed
  by point and already read for pieces on blocks.
- State that already exists for "what is behind a piece": the contact graph with last step's
  impulses, and islands. State that does not: any price for a contact between two loose pieces.
- Absent from the fork: per-contact mass scaling, any ordering by distance from a heavy body,
  any multi-sweep or block solve.
