# Strength-limited contact

Subject: the game's building model (game state only): what a moving body and a standing, bonded
structure do to each other in the physics step. The design check below was written before anything
was built or run; it cites code at `main` (`d10f30d9`) unless a lane is named; `bay-1` is `lane/cutter-geo` at `c1380911`. Every number is
MEASURED (with its record), DERIVED (with its inputs) or ASSUMED. The 57.6 t hull, `house_1`,
`hall_1` and `fir_1` are fixtures, never the spec.

## As built (2026-11-23)

This section is the law the game runs; it owns the narrative. Everything below it is the
pre-build design check of 2026-11-21, kept as written: where the two differ, this section is the
code. Measurements and the checkpoint tables are in
`.agents/scratch/rubble-explore/cutter/mass-limited-build.md`, `commitment-pass.md` and
`work-form.md`.

### The law

A standing cell returns a body no more than its bonds can. Each physics step, after the narrow
phase, the cells one body presses on one construction are grouped into blocks of connected cells
(`breakage::bounded`). A block is pushed along the body's travel at the block, along the force
that persists on the body (drive, weight), and along each face the body presses. For each push
the block has one way to give, the one that holds least, which a rising force moves first
(`fracture::bound`):

- **Shear-out.** The pressed cells and the matter behind them along the push, up to a free face,
  leave through that face. The price is a sum over the boundary joints of that plug: each opens at
  its tensile strength and slides at its cohesion plus the friction of its gravity preload, and
  spends its fracture energy. The work also carries a `StandIn` for the grip of standing matter
  on a loosened plug, which the solver does not carry: each sliding joint's preload friction over
  the plug's depth along the slide. Joints inside the plug carry nothing, so a wall is no stronger the
  finer it is cut. Shut when the push runs into the ground or buried matter.
- **Shed.** The pressed cells slide out sideways, through the cheapest side whose matter can
  move. The joints the push closes rub under the push itself, so the bonds' resistance `R` is
  passed at `P = R / (1 − μ)`. This form is a `StandIn`: it turns the whole push sideways
  (`κ = 1`). The missing law is the sideways share of a push on a block, by the push's direction
  and what backs the block.

A price is a force `F` (N) and a work `W` (J); the give `W / F` is never authored. A block is
bounded twice, and a seam in `avian_ext` (`reaction_bound`) carries both into the solve:

- **Strength.** Over a step the points that lean a way's push return at most that way's `F · dt`
  between them. The bound belongs to the way, not to its points: the seam deals it to them before
  every substep, each keeping what it carries and the rest going to the points the last solve
  left wanting, so the points' bounds always sum to the way's.
- **Work.** Over a step all the points of one body on the block do at most the least work its
  ways have left (one budget per body and block). The work of an impulse `p` over a substep, on
  bodies closing at the point at `c0` before the substep's forces and `c1` after its last solve,
  is `p · ½(c0 + c1)` (DERIVED: a body's kinetic-energy change is each impulse on it times the
  mean of the speeds across it, so the two speeds carry every other point, body and force). The
  seam counts that per point between substeps and takes it from the budget. Inside a solve a
  limiter hook in the fork judges the impulse the solve found: if it does more than the budget
  has left, it is cut to the impulse that does exactly that; if it does less, it stands. So a
  body that cannot do the work is stopped at the block's strength and the block stands, and a
  body held still does none however hard it presses. The work is judged at the impulse the
  solve found, never at a lesser one that would do the work left: a point's own response is not
  the response of the bodies behind it, so at a lesser impulse a light body a heavy one holds
  against a block reads as flying into it alone. Counterexample, cutting whenever some lesser
  impulse does the work left (MEASURED, the seam's fixtures): a 2 kg box held still under a
  2,000 kg box had its 100 J budget spent at rest on step 19, and the same pair at 4 m/s lost
  1,055 J to a budget of 1,000 J, outside its allowance. A budget a solve cut, or whose work is
  done, is spent: its points do no more work that step.

The fork carries the per-point impulse clamp and the limiter's call (`OVERMATCH_PATCH.md`,
"Bounded contact"); the arithmetic is the seam's.

After the step, every contact point on a standing cell adds to that cell the work it did, as
counted inside the solve, negatives adding nothing. Nothing removes the work while the cell
stands: it is kept per cell on the construction (`Live::work`), and a cell that leaves carries
none as a piece. A block parts by the way that takes least work when the work on its cells has
reached that way's, or its budget was spent. Its boundary joints part, and its cells and
whatever they alone held up leave at rest as ordinary rubble.

Residuals of the form, each stated where it is tested (`src/tests/avian_reaction_bound.rs`,
`breakage/bounded/tests.rs`):

- **Solve lag.** A point's work inside a solve is estimated at its own last solve; a later
  point's impulse changes the speed there. The substep that spends a budget is off by less than
  that substep's work at the block's strength, `F · dt / substeps · v` (DERIVED); the fixtures
  land within 0.03 % of the budget (MEASURED: 1,000.2 J of 1,000 on sixteen points).
- **Several bodies.** The work left is read once, before the step, and a budget is one body's:
  `n` bodies parting one block in one step do up to `n` times the work it had left (MEASURED:
  two boxes on one block of 3,111 J did 6,222 J).
- **Rounding of the count.** The budget is signed and f32, the cell's count floored per point:
  a block can part on its budget with the count a fraction of a joule under `W` (MEASURED:
  6,213.12 J of 6,213.36).
- **Restitution** adds impulse after the last substep, which no pair of speeds spans: it is in
  the impulse a point returned and not in its work. Standing matter's pairs have none today.

A belt's sampled support on standing matter has no solver point, so nothing is solved for it
and the budget cannot reach it. Its staged force, support and both tangent terms together, is
bounded whole before the step by the same price at the belt's speed at the step's start: the
way that returns it least, `F · dt` or, when it would cross the give left within the step, the
work left over that speed. Its work is counted along the whole force by the mean of the step's
start and end speeds, and its block parts when that work is reached or the belt crossed the
give while pressing harder than the ration. Its residual is the speed it could not read: the
change of the belt's speed inside the step. The support check hears no contact: it reads
weights.

**Work is one scalar per cell**, whoever did it and along whichever way: a block is charged with
all the work on its cells, so a way may part on work done along another way or by another body.
The cost, MEASURED (`work-form.md`, the fix pass; wall, house and hall, a 57 t hull): 12 to 31 %
of the cells that part leave through a way whose own body added under a tenth of the work on it,
and 30 to 45 % of the blocks a loose piece parts are of that kind. Accepted: a wall worked on from
several sides is a damaged wall. A charge per direction is the alternative, not built.

What stays rigid: an unbreakable element, a buried cell, a face slid along, and a block with no
way open. `buried` is judged against the terrain at spawn (`Breakage::sited`), and a sited
construction with a loose group is refused.

### What was tried and lost

- **The hull sweep** (main's law until this lane: a cast before physics, a minimum cut, a plastic
  exchange). It predicted the body's path and charged a plug's strength over the tick; three
  builds each stopped the hull on matter a real wall gives up. Deleted with its account, hook,
  stage and tests.
- **A crush price** for a boxed-in cell. 99 % of the crush-priced return at the hall junction was
  pushes into the ground (MEASURED, checkpoint 2): it was a hold, not a way to give.
- **The shed with a free closed joint** (the draft's literal form). It sheds at 77 kN (p50) and
  strips the hall to 48 % in place (MEASURED, checkpoint 4): the closed joint's friction is real.
- **An even share of a block's bound per point** (`EVEN_SHARE`, tried at 0.1, 0.25 and 0.5). Every
  pass moved with it and not monotonically: the number did a law's work. The block-sum deal
  replaced it and needs no number.
- **A soft spring on yielding cells, a cap at the body's weight, turning freed cells to dust.**
  Each was a rule on one kind of contact; none survived its own run.
- **Spent when clamped**: give counted at the price on any step a block's point ended at its
  bound, steps bounded by force included, and forgotten on any other step (the law until the
  work form above) or never forgotten. Kept, the long wall crossed slower than either arm alone
  and the junction still did not cross (MEASURED, checkpoint 1); forgotten, a plain wall held a
  slowed hull inside cells that never left: at 2 km/h, from a standing start and in 2 of 3
  passes at 10 km/h (MEASURED, `work-form.md` checkpoint 1, the `TODAY` arm). The spent clause
  above differs in firing only when the body crosses all the give left within the step.
- **The work bound dealt per way and the second spent clause** (the law until the budget above):
  each way's work left was dealt to its points as `W / v` at the solve's closing speed, the count
  was taken after the step from the impulse and the mean of the step's start and end speeds, and
  a way was also spent when the body crossed its give at the step's start speed and the last
  solve left a point at the dealt work bound. The dealt bound and the count read different
  speeds, so the clause parted blocks by one point at one substep: 68 to 82 % of its partings
  were by loose pieces of about 30 kg whose own energy was under 1 % of the work left, and a
  hull's blocks parted at a median 0.78 of their work (MEASURED, `work-form.md`, the second
  clause).
- **The work counted at the step's end speed alone** (the work form's first build). Off by the
  impulse times half the body's speed change: nothing for a piece stopped inside the step, double
  for one thrown (DERIVED); the fixture's stopped box was charged 14 J of the 186 J it lost
  (MEASURED).
- **Depth read from the contact** (the advance taken from the narrow phase's overlap) and
  **depth gained** (work only for travel past the deepest the body had been in the cell, one
  mark per contact). Replayed from logged rows, never built: a hull's pose barely advances on
  the steps where cells return most while its velocity is kept (84, 7, 84 mm at a steady
  5.38 m/s), so depth counts 55 and 50 % of the hull's work at 10 km/h, 34 % at 20 km/h and 16 %
  at the junction, while loose pieces' work rises (MEASURED, `work-form.md` checkpoint 2).
- **A slower push-out speed** (0.5 m/s for Avian's 4). The house keeps less and frees twice the
  pieces; the engine's default stays and the game states no number for it.
- **Leaving with the pusher's momentum** (a plastic exchange at release) and **shared plugs**
  (blocks whose plugs share cells answering from one bound). Neither was built: birth at rest is
  at most 30 % of the worst junction tick (DERIVED from its rows), and shared plugs return 14 to
  29 % of the stall's load, each group far under its largest bound (MEASURED, checkpoint 6).

### Known and accepted

The hall junction does not cross at 20 km/h. At the hold the hull is lifted off the soil onto
loose pieces: the belt law carries nothing (it samples soil and standing cells, never a loose
piece), so there is no traction, and the solver carries the weight through the belt-face
colliders (8101 N·s a step) and the belly (721 N·s) (MEASURED, `work-form.md` checkpoint 3).
That is a fault of the track law on rubble, not of this law (`.agents/lenses/backlog.md`,
driving-22). The shed price's `1 / (1 − μ)` stays the stand-in above; no number is tuned for
either.

## The design check (2026-11-21)

**Verdict: BUILD IT WITH CHANGES.** The candidate's core is right and is what the engine can carry:
a standing cell answers through the solver with a bounded reaction, and it breaks loose by a
criterion on that reaction. Three changes are needed, and the first decides the outcome:

1. **The bound is on work, not only on force.** Capping a cell at `strength × tick` repeats build 2:
   a cell priced at crushing (1.9 MN) returns 30 kN·s in a tick, 0.5 m/s off the hull per cell
   (DERIVED, section 2). A brittle bond gives way after a fraction of a millimetre, so what it can
   take from a fast body is `work / speed`, a few N·s. That is the missing truth of all three
   builds: they charged a cell's strength over the cell's depth (0.25 m) or over the tick; the
   material's give is 0.2 to 5 mm.
2. **Bearing counts.** "Pressed into the cell behind, the chain passes on" cannot be the rule for
   two standing cells: a falling beam would then cut a slot down a wall to the ground. A backed
   cell leaves through the free face behind it, or is crushed in place; both have a price in data.
3. **Bonds between cells pressed together carry nothing.** Summing each touched cell's own joints
   makes a wall stronger the finer it is cut. The block that moves is priced at its boundary.

Rubble needs nothing new. No path prediction, no train, no plastic exchange rule, no cast.

## The law, in plain words

A wall is pieces glued together. Glue is strong but brittle: it holds hard, then lets go after
moving a hair's width. So a piece of wall can push back on a tank with at most its glue's strength,
and only until the glue has stretched that hair's width. For a fast tank that is over in a few
thousandths of a second: the wall takes almost nothing from it. After that the pieces are loose
rubble, and what slows the tank is their weight, which it must shove along. A slow tank leaning on
the wall is a contest of force: if its push is below the glue's strength, the wall holds; above, it
gives. A piece that is boxed in by wall on every side has nowhere to go and must be crushed, which
costs far more: that is why a thick wall stops what a thin wall cannot.

### The rules

1. **Bound.** A standing cell's reaction on any body, over one tick, is at most
   `J = min(F · dt, (W − W_done) / v) / (n·m)`: its strength for the tick, or the work it has left
   over the body's speed, whichever is less.
2. **Block.** The matter that moves is the cells a body presses plus the cells behind them along
   the push, up to a free face. Joints inside the block carry nothing; `F` and `W` are sums over
   the block's boundary joints.
3. **Two ways to fail**, each with its `F` (N) and `W` (J); the cell yields by whichever gives the
   smaller `J`:
   - *shear-out*: the block's boundary bonds part and it leaves through the free face behind it.
     Not available when the push runs into the ground or buried matter.
   - *crush in place*: the pressed cells are crushed against their backing. Cheap where a side
     face is free (the cell splits); dear where it is boxed in (it must be compacted).
4. **Direction.** `m` is the way the block leaves: the body's travel direction, the contact normal
   `n`, or the direction of the force that persists behind the body (drive, weight); the least `J`.
   `n·m` near zero is a face slid along: no bound binds, ordinary contact and friction.
5. **Failure.** A cell fails when the work done on it reaches `W`. It is freed after the step, at
   rest: the impulse it returned went into the ground through its bonds. A cell whose bound did not
   bind in a tick forgets the work (no damage memory from resting or jittering contacts).
6. **Rubble is rubble.** A freed cell is an ordinary loose body. Pressed against standing matter it
   is one more body under rule 1. The body that pushes it pays for its mass through ordinary
   contact. Overlap left by the release is eased apart by the engine's bounded push-out.
7. **Any body, one law**: hull, gun barrel, loose piece, falling roof member. The track's sampled
   support on standing cells obeys the same `F`.
8. **The support check hears only weights**, as built.
9. **Creep price.** For a block that held a load through a tick, `F` is the least force that cuts
   any block containing the pressed cells free (the directional minimum cut, used as a price
   only). Until then `F` is the pressed block's own boundary, an upper bound.
10. **No prediction.** Nothing is judged before the narrow phase has found what is touched.

### The terms

| Term | Meaning | Unit | Job and source |
|---|---|---|---|
| `F` shear-out | `Σ` over boundary joints: `σ_t A` on opening, `(c + μ σ_pre) A` on sliding | N | data: `tensile`, `shear_cohesion`, `friction` (`materials.ron`, `attachments.ron`); `σ_pre` DERIVED by the support check's gravity solve. Today's `JointStrength::translation_resistance`, `bay-1 src/construction/fracture/bondsolve.rs:466-483` |
| `W` shear-out | `Σ (G_I or G_II) A` + clamp friction `μ σ_pre A × d`, `d` the block's depth along `m` | J | data: `fracture_energy`, `shear_fracture_energy`. The clamp term is a `StandIn` for the grip of standing matter on a loosened block, which the solver does not carry (a static neighbour bears on nothing) |
| `F` crush | `σ_c × A_face` | N | data: `blast.compressive_strength` |
| `W` crush, a side face free | `σ_c² / 2E × V` (the elastic energy at crushing) | J | DERIVED from `compressive_strength`, `youngs_modulus`. The compressive fracture energy is `Unknown`; a measured value would replace the form |
| `W` crush, boxed in | `σ_c × (1 − ρ/ρ_s) × V` (work to compact to lock-up) | J | DERIVED from `compressive_strength`, `density`, `solid_density`: the blast law's `Compaction` facts (`bay-1 src/construction/fracture.rs:98-119`) |
| give | `δ = W / F` | m | DERIVED, never authored |
| `v` | the body's closing speed along `m` at the point, at the tick's start | m/s | state. `Resolution`: one speed per tick |
| `dt`, equal shares among a block's points, cells as the smallest thing that fails | | | `Resolution` |
| the elastic branch | rigid up to `F` | | `Resolution`, derived below |

No new authored number. Missing data: a concrete row (all facts), the compressive fracture energy
of brick and wood, the work of a root plate (`attachments.ron` states `fracture_energy_j_per_m2:
(1.0, Unknown("StandIn ..."))` for every root plate), green-wood shear facts (carried as `Unknown`
from the dry row).

---

## 1. What the engine offers (verified at source)

Paths under `vendor/avian3d-0.8.0-overmatch/src/` unless they start with `src/`.

| Mechanism | Fact | Where |
|---|---|---|
| How a contact is solved | Soft constraint (bias, mass scale, impulse scale) in a biased pass, then a relax pass without bias; accumulated impulse clamped at zero from below only | `dynamics/solver/contact/normal_part.rs:152-181`; clamp `:179` |
| Softness | Global only: `contact_damping_ratio` 10, `contact_frequency_factor` 1.5; frequency `1.5 × min(1/(2dt), 0.25/h)`, doubled against static bodies. At 64 Hz and 6 substeps: 48 Hz, and 96 Hz against standing matter (DERIVED) | `dynamics/solver/plugin.rs:593-600`, `:625-649`; choice per constraint `contact/mod.rs:170-173`. The game sets none of it (`src/net/physics.rs:32-35` sets only `recycle_distance: 0.0`) |
| Push-out speed | Global `max_overlap_solve_speed`, 4.0 m/s, applied as a cap on the bias | `plugin.rs:549`, `:595`; `normal_part.rs:167` |
| Speculative contacts | Opt-in per body (`SpeculativeCcd`); otherwise the margin is the contact tolerance, 0.02 m. A point with positive separation is a rigid stop at the gap | `collision/narrow_phase/system_param.rs:776-799`; `collision/narrow_phase/mod.rs:224`; `normal_part.rs:141-143`. No game body carries `SpeculativeCcd` (grep of `src/`; `src/debris/freeze.rs:1386` says the game has not decided). `freeze.rs:1390` and `:1735` call the tolerance 5 mm: the code says 0.02 m, so those comments are stale |
| Substeps | 6 (the default; no production insert in `src/`, the three inserts are fixtures). Manifolds, normals and anchors are fixed for the step; each substep recomputes only separation (from the bodies' accumulated motion) and relative velocity | `dynamics/solver/schedule.rs:189-217`; `contact/mod.rs` `solve` (the `delta_translation`, `separation` lines) |
| Per-point spring law | Fork: `ContactNormalPart::{compliant, spring}`, 25 fork lines, written game-side per manifold by `Laws` | `normal_part.rs:33-40`, `:137-151`; `src/avian_ext/compliance.rs`. Used today by one thing: the track belt's soft run against loose pieces (`compliance.rs:19`; `OVERMATCH_PATCH.md`, "Compliant contact") |
| Contact hooks | One hook set for the app; read-only access plus deferred `Commands`; may drop a pair, edit points, normal, friction, restitution, tangent velocity. No access to the contact graph | `collision/hooks.rs:139-187`; the game's is `WorldContacts` (`src/lib.rs:915`), which chains the sweep's |
| Editing constraints inside the step | Native: `ConstraintGraph` is a public resource and `SubstepSchedule` a public schedule. The game already writes prepared constraints (`compliance.rs:117-161`) and already runs systems between substeps (`src/contact_transfer.rs:783`, `src/avian_ext/solver_cost.rs:31`) | |
| Static to dynamic inside the step | Not possible. Solver bodies and constraints are prepared once before the substeps, and the fork panics if solver membership changes between the build and the clear of velocity increments | `schedule.rs:86-93`; `OVERMATCH_PATCH.md`, "Sleeping bodies' velocity increments" |
| Maximum impulse per contact | **Absent** on `main` and on `lane/cutter-geo` (grep of `bay-1/vendor`: no `max_impulse`) | |
| The clamp from `lane/cutter` | `0a063f89`: `max_impulse: f32` on `ContactNormalPart`, infinite by default, and `.min(max_impulse)` in `solve_impulse` and `apply_restitution`; 13 fork lines (the commit's count). Seam `src/avian_ext/reaction_bound.rs`, 102 lines: `Bounds` keyed by pair, shared equally by the pair's points, warm start clamped. Friction needs no line: the tangent solve clamps to `friction × normal impulse` (`contact/mod.rs` `solve`; the belt adapter reads the same field, `src/avian_ext/surface_motion.rs:324`) | `git show 0a063f89` |

**Fork statement.** Native: soft contact (global), bounded push-out (global), hooks, opt-in
speculative contact, the between-substep slot. The fork already carries the per-point spring. The
smallest further change is the 13-line clamp, re-landed from `0a063f89`. It is a **divergence with
its seam** (`src/avian_ext/reaction_bound.rs`), not a fix upstream would adopt without argument:
upstream's contact is rigid by design. It is the form PhysX (`setMaxImpulse`), Havok and Blast
ship (prior-art survey, sections 1b and 1c). The seam changes in one way: bounds are per point, not
per pair, because a pair against an unswapped element spans several cells. The spring law is not
used by this design.

## 2. Is it sound as physics

### The force-displacement law

`reaction(x) ≤ F` for compression `0 ≤ x < δ`, zero after; `δ = W / F`. Rigid below `F`.
Integrated over a tick for a body that is not stopped: `J = F × min(dt, (δ − x)/v)`, which is
rule 1 with `x` carried as work done.

**Why the elastic branch is a `Resolution` omission.** Brick, 0.21 m cell: axial stiffness
`E A / L = 8.6e9 × 0.0525 / 0.21 = 2.15e9 N/m` (DERIVED). Two limits:
- frequency: a spring is resolved when `k < m / h²`. At `h = 1/384 s`: 8.5e9 N/m for the whole hull,
  2.2e9 for a 15 t contact mass, 3.8e6 for a 26 kg piece (DERIVED). The implicit soft constraint is
  stable at any `k` (its mass scale tends to one), so nothing blows up; a stiffer spring is simply
  rigid.
- position: the give must be crossed in more than one substep. `δ_c = 2 G_II / c = 0.34 mm` for a
  mortar bed (DERIVED, the ram design's figure); the body covers that in one substep above
  `0.34 mm × 384 = 0.13 m/s`.
So above a walking crawl no solver at this rate can sample the curve. The law is integrated in
closed form instead, which is exact for a body the cell does not stop.

### What the three builds charged, and what the material allows (DERIVED)

| | Force | Over | Work | Off a 57.6 t hull at 10 m/s |
|---|---|---|---|---|
| Builds 1 and 3: a cell backed by wall, hall cut | 6.9 MN (MEASURED hold, round-2 report) | depth 0.27 to 0.58 m | 1.9 to 4.0 MJ | all of it (3.25 MJ at contact, MEASURED) |
| Build 2: the same class of hold as a per-tick cap | 3.0 to 3.7 MN (MEASURED) | 1 tick, held for 30 | | 0.8 m/s a tick, braked to a stop (MEASURED) |
| The material, house cell crushed with a side free | 1.9 MN | 0.45 mm | 0.85 kJ | 0.0015 m/s |
| The material, house cell sheared out of a single leaf | 36 kN | 11 mm | 0.4 kJ | 0.0007 m/s |

Breaking a bond does cost the body energy: `W`. It is negligible for brick at speed and decisive
at a crawl, where `½ m v²` falls below it: the hull's threshold on a crush-priced house cell is
`√(2 × 850 / 57600) = 0.17 m/s`, on a hall cell (5.9 kJ) 0.45 m/s. For timber the bond work is
also small (Wood `shear_fracture_energy` 740 J/m², MEASURED row); what a tree really costs is
bending and root-plate work, which the data does not hold (below).

### Losses (DERIVED; fixture: hull 57.6 t; drive 503 kN ASSUMED in the history, not re-verified; patch 3.76 × 1.5 m ASSUMED; brick and wood rows as read)

"Bonds" is `W / (M v)`. "Mass" is the plastic pick-up of everything freed, an upper bound (pieces
spill). Rubble drag afterwards is the solver's and is not in this table (MEASURED on build 3:
3.16 m/s through the whole house at 9.4 m/s).

| Case | `F`, `W` | Bonds at 3 / 6 / 10 m/s | Mass at 3 / 6 / 10 | Worst tick at 10 | From rest under drive | Believable | Absurd |
|---|---|---|---|---|---|---|---|
| Single-leaf brick, face-on | 250 to 470 kN; 3 to 10 kJ | 0.06 / 0.03 / 0.02 | 0.21 / 0.42 / 0.69 (4.3 t) | ≤ 0.7 | passes, margin 6 to 50 % | loses 0.1 to 1.5 m/s per wall, crosses | stopped; a tick ≥ 2.3; no opening |
| 12 m brick wall along its length | crush 13.7 MN, 29 kJ per metre; strip 1.1 MN, 1.5 MJ | per 0.21 m layer 0.035 / 0.018 / 0.011; over 12 m: stops at 8.9 m / 1.1 / 0.6 | 0.58 / 1.15 / 1.9 (13.7 t) | 0.04 | held (1.1 MN) | at 10 m/s loses 1 to 4 m/s over the wall, consumed as the hull advances; coasting at 3 m/s may stop inside; under drive passes above 0.5 m/s | stopped at first contact above 1 m/s; whole wall down at first touch; no loss |
| Double-leaf brick, 0.5 m | 870 to 940 kN; 38 kJ | 0.22 / 0.11 / 0.07 | 0.39 / 0.78 / 1.3 (8.7 t) | 0.5; 1.3 if the plug stays one piece | held or marginal | loses 0.5 to 2.5, crosses above 1.2 m/s | stopped at 6 m/s; the same loss as a single leaf |
| 1 m concrete | **no data row**. Brittle ASSUMED numbers (3 MPa, about 0.2 MJ): passes at 3 m/s. With reinforcement work ASSUMED 0.3 MJ/m² (3.2 MJ): held | | 1.9 at 10 (13.5 t) | up to 8.6 when held | held | stops the hull within one or two ticks; that is a crash and reads as one | driven through at 10 km/h |
| Timber board wall, 0.05 m | 4.7 MN (seams), 0.4 kJ | < 0.01 | < 0.03 (126 kg) | < 0.05 | **held on the pressed block's price; passes on rule 9** (nails: about 3 kN, DERIVED from `nailed` 3.24 kPa) | no perceptible loss, crosses at any speed | held at any speed |
| Thin tree, 0.30 m | 1.26 MN, 1.0 kJ | < 0.01 | < 0.1 | < 0.1 | held on the block's price; passes on rule 9 (root plate 71 kN, `StandIn` cohesion) | falls, loss under 0.3 m/s | stops the hull above a crawl |
| Thick tree, 0.60 m | 5.0 MN, 7.5 kJ | 0.04 / 0.02 / 0.01 | up to 0.4 at 10 (2.4 t stem, DERIVED cone) | 0.4 | held on the block's price; passes on rule 9 (283 kN) | falls; loses 0.2 to 1.5 at 3 m/s, 0.3 to 3 at 10 | no loss at 3 m/s; stops the hull at 10 |

Three honest readings of this table:
- **The thick tree is under-charged.** Reference from data: overturning `154.6 N·m/kg × 2,400 kg =
  371 kN·m` (MEASURED coefficient, DERIVED mass); over an ASSUMED 0.3 rad that is about 110 kJ,
  0.64 m/s at 3 m/s. The law charges 7.5 kJ. The missing truths are the root plate's work
  (`Unknown` in data) and a member turning about a joint (a lever), which no translation price
  holds. Named, not built: the tree fixtures will show it.
- **Concrete depends on a row that does not exist.** A bunker today is an unbreakable element
  (`nearest_chunk` returns `None` for one, `bay-1 src/construction/breakage.rs:1164-1171`): rigid
  contact, a crash. That is the honest behaviour of a strong obstacle; softening it is a vehicle
  fact (how the vehicle's own structure gives), not a wall law.
- **A wall is stronger along its length**, as the owner expects: it holds a standing-start push
  that a face-on wall does not, and it costs about three times as much to drive through. It is not
  stronger by its bonds at speed; it is stronger by the mass it puts in the way and by the work of
  crushing its end layer by layer.

## 3. Timing

- **When a contact is first seen.** With no speculative margin beyond 0.02 m, a body is first seen
  touching a cell when it is within 2 cm or already overlapping: up to one tick's travel deep
  (0.156 m at 10 m/s, 0.088 at 20 km/h, 0.044 at 10 km/h; DERIVED).
- **When the cell is released.** After that step: the work reading comes from the solve
  (`ContactTransfers`, `src/contact_transfer.rs`), and a body cannot enter the solver inside a step
  (section 1). The body moves one more tick in the meantime, paying at most rule 1.
- **Sinking.** Up to two ticks' travel at the release: 0.31 / 0.18 / 0.09 m at 10 m/s, 20 km/h,
  10 km/h; about 1.5 ticks on average. The freed piece is then an ordinary body overlapping the
  hull and is eased out at up to 4 m/s: 80 / 45 / 22 ms.
- **0.21 m brick cell and 3 cm board: the same.** Every standing cell a body overlaps at a tick
  boundary has a contact that tick, whatever its size; all are bounded and judged in the same
  step. A board is wholly inside the hull by then and leaves through the nearest face. A small fast
  piece cannot skip a thin cell: Avian sweeps fast bodies against static matter (the fork's
  per-collider eligibility, `OVERMATCH_PATCH.md`, "Per-collider swept CCD eligibility").
- **So "cells at least a tick's travel wide" is not needed.** What makes the outcome independent of
  cell size and tick rate:
  1. the fast branch of rule 1 holds no `dt`: `W / v`;
  2. `F` and `W` are sums over a block's boundary (rule 2), so the cut inside the block does not
     enter; through the thickness, the block runs to the free face;
  3. release is by work done, not by a count of ticks.
  The slow branch is a force, which has no step either. What does scale with `dt` is the sinking,
  which costs no momentum.
- **Test.** One wall fixture at two in-plane cuts and one and two layers through its thickness, at
  64 Hz × 6 and 128 Hz × 3, struck by one block at one speed: the block's momentum change within
  15 %, the freed volume within one coarse cell ring. Mutants: `J = F · dt`; each cell's own joints
  summed.
- **Release inside the step is not needed and needs no fork change if it ever is**: a game system
  in `SubstepSchedule` can zero a spent cell's `max_impulse` between substeps. Rule 1 already
  integrates the give over the tick.
- **Look-ahead.** Not built. If the sinking shows (see the kill line), the clean form is the native
  one: `SpeculativeCcd` with a bounded distance on vehicles, which reports a cell in the tick the
  body arrives and halves the sinking. Nothing of the swept test remains, and its cost (3.4 to
  14 ms a tick, MEASURED) goes. The price of `SpeculativeCcd` is that it changes every vehicle
  contact in the game, which is why it is a separate decision (#371).
- **A strong obstacle met fast** is first seen up to a tick deep and then pushed out at up to
  4 m/s. That is today's behaviour against every static thing; this design neither causes nor
  cures it. `SpeculativeCcd` cures it.

## 4. Rubble that compresses

| Option | What it does | Heap under a tank | Pieces at rest | Truth or knob |
|---|---|---|---|---|
| Nothing added | Standing cells are bounded, so a piece freed with matter behind it presses on that matter with at most rule 1 and it yields in its turn. Fresh overlap is eased by the engine's 4 m/s push-out, already on | unchanged from today: the belt's soft run on loose pieces, and piece on piece rigid | unchanged | the bound is a truth (bond strength and work); the push-out is an existing `Resolution` constant |
| Soft contact for fresh rubble | A spring and a decay time per fresh pair, through the fork's spring law | pieces under load sink and creep; a heap that carries 57 t would need the softness switched off by some rule | sinking and slow jitter while soft | a knob: softness and decay have no material owner (the survey's pattern f) |
| Lower push-out speed | Gentler separation of everything in the game | terrain and hull contacts all change | deeper rest overlap | a knob, global |

**Decision: nothing added.** The jammed plug of build 3 (4.16 m/s in a tick, MEASURED) was a loose
piece between the hull and a standing wall that answered rigidly. Under rule 1 that wall returns at
most `W / v` per cell: for the two cross-wall cells of that record, `2 × 5.9 kJ / 10 m/s =
1.2 kN·s`, 0.02 m/s (DERIVED). "Phasing until the space is freed" is what happens, between a piece
and bounded standing matter for one tick, never between loose pieces.

## 5. Kept, removed, and the objections

| Measured failure | Under this law |
|---|---|
| Build 1: held cells stop the hull; normals with a vertical part buy bearing at crushing | Crushing's work is 0.85 kJ a cell: `J` = 85 N·s at 10 m/s. Rule 4 also lets the block leave along the travel |
| Build 2: slid-along cells held at MN brake by friction for 30 ticks | Friction is bounded by `μ J`. A cell whose bound binds is freed after that step; one whose bound does not bind carries only the real normal load |
| Build 3: an interior cross wall met along its length holds (7 to 12 MN) | Holds only a standing-start push. Above 0.5 m/s it is crushed layer by layer, 29 kJ per metre |
| Build 3: the freed plug jams | Section 4 |
| Owner's session: every building immovable at 10 to 20 km/h | 2.8 to 5.6 m/s is 6 to 30 times the highest threshold in the table (0.5 m/s for a crush-priced hall cell, 1.2 m/s for a double leaf) |
| Build 3: the overlap casts put p95 over budget | No cast. One pass over the step's contact pairs on standing matter, one sum per pressed block |
| What works: the support check hears no contact | Kept (rule 8) |

**Objections.**
- *A wall no stronger along its length than its bonds.* False at a crawl (held, 1.1 MN), true at
  speed for its bonds, and the mass (13.7 t against 4.3 t) and the crushing work make the
  difference a player can feel. Within the believable range; the owner judges.
- *A hull grazing a wall strips it.* A face slid along has `n·m` near zero and a small closing
  speed, so the force branch binds: stripping needs a side push above the block's strength (tens to
  hundreds of kN). A corner entering at 5° at 10 m/s closes at 0.87 m/s and gouges a line of cells.
  Believable.
- *A slow or parked vehicle leaning on a wall.* Parked: the reaction is far below `F`, nothing
  yields, no work accrues. Under full drive: a single-leaf brick face gives after about a
  centimetre; a wall end, a double leaf and anything boxed in hold. Timber and trees need rule 9.
  Should a 57 t vehicle with 50 t of thrust push through one brick's thickness: yes.
- *A falling roof member or thrown piece.* Energy against `W`. 100 kg at 5 m/s onto a wall top:
  1.25 kJ against 0.85 kJ, at most one top cell split and the next holds. A 26 kg piece against a
  wall face at mid-height: `W` 0.4 kJ, held below 5.5 m/s, punched out above. Top-course cells
  carry no clamp and come off at under 1 m/s. Not modelled: a soft striker crushing a harder cell
  (timber splitting brick); it is one cell, named for the eye.
- *The gun barrel.* Same law. Along its axis at speed it holes the wall for nothing; no law yet
  says what that does to the barrel. Swung by the turret its contact mass is small and the wall
  mostly holds.
- *First touch on an unswapped element.* An untouched element is one collider
  (`src/construction/breakage.rs:11-14`), so its manifold has at most four points and the pressed
  block is under-reported for one tick. The first failure swaps it (existing rule). Cost: one more
  tick of sinking on first touch. A `Resolution` limit, to be measured.
- *The track.* The belt's front and side faces are solver contacts and fall under rule 1. Its
  sampled support on standing cells is not a solver contact (`src/track/rubble.rs:478-507`); it is
  staged and can already be cancelled. Rule 7 bounds it by the same `F`.

## 6. Deleted and added (approximate; file sizes MEASURED by `wc`)

| | Against `lane/cutter-geo` | Against `main` |
|---|---|---|
| Deleted, production | `sweep.rs` casting, judge, train, exchange (about 800), `sweep/source.rs` (358), most of `sweep_intake.rs` (about 1,200 of 1,500), the hook's unjudged filter: about 2,400 | the same plus the contact intake in `breakage.rs` (about 300) and the resting half of `contact_transfer.rs` (about 390): about 3,100 |
| Deleted, tests | about 2,300 | about 2,900 |
| Kept | departure and routing, the staged belt cancel, `mincut.rs` (584) as rule 9's price | |
| Added, production | law module about 250, wiring about 180, seam 102, instrument about 70: about 600 | about 600 |
| Added, fork | 13 lines | 13 lines |
| Added, tests | about 400 | about 400 |

Read "added" as up to twice that (the record of these estimates).

**Instrument.** The hull row already has what the verdict needs: solver momentum by the other
body's kind, the standing split into normal and friction with each point's cell and collider,
freed cells, the largest one-tick change (`bay-1 scripts/perf/collapse.md:51-60`). Missing, to add:
per block its way, `F`, `W`, closing speed, bound and returned impulse; each freed cell's overlap at
release; the belt's force on standing cells (today "not channels"); the largest freed piece's mass.
The `verdicts` row goes.

## 7. Proving ground

**What exists.** Constructions: `house_1`, `hall_1`, `fir_1` (`assets/constructions/`). Placed:
`house_0001`, `village_hall_0001` and trees on kalinovo, the only map (`assets/maps/`). The
generator makes houses and trees only (`tools/src/construction/spec.rs`: `House`, `CrossWall`,
`RoofFrame`, pieces); a lone wall is not expressible. In-code fixtures exist
(`src/construction.rs:348`, `column_construction`). Loose objects in the game today: construction
pieces, the debris stress lever's cubes, a launched turret, other hulls. No boulder, no vehicle
wreck (grep of `prototypes.ron`).

**Where it lives.** Law and physics fixtures in code, headless (a `wall_fixture(length, height,
thickness, pitch, layers, material)` beside `column_construction`), struck by a block of stated
mass and speed and by the shipped hull. The house, hall and tree passes on kalinovo through
`scripts/perf/collapse.sh --ram`, at three speeds, with clips. A visible strip of walls on the map
needs a wall spec kind in the generator: the owner's call, after the first build.

| Fixture and pass | Expected (fixed now) | Absurd |
|---|---|---|
| Brick, single leaf, face-on, 3 / 6 / 10 m/s | crosses; standing channel ≤ 0.1 m/s; total 0.1 to 1.5; opening hull-wide, ragged | stopped; tick ≥ 2.3 |
| Same wall, coarse and fine cut, one and two layers | momentum change within 15 %, freed volume within one coarse ring | differs by more |
| Same wall at 64 Hz × 6 and 128 Hz × 3 | the same | differs by more than 25 % |
| Brick, along its length, 12 m | consumed as the hull advances; 1 to 4 m/s lost at 10 | stopped at contact; whole wall down at once |
| Brick, 45° | crosses; gouge wider than the hull | glances off; stopped |
| Brick corner, on the diagonal | crosses, both returns broken near the corner | stopped |
| Double leaf | crosses above 1.2 m/s, loses up to 2.5 | as cheap as a single leaf; stopped at 6 |
| Concrete (needs its row), or an unbreakable element | hull stopped in one or two ticks | driven through |
| Timber board wall | no perceptible loss | held at speed |
| Thin and thick tree, 3 and 10 m/s | both fall; the thick one costs visibly more | thin stops the hull; thick costs nothing at 3 m/s (expected miss, section 2) |
| House face-on and end-on, at 10 km/h, 20 km/h and 95 % of governed speed | crosses every time; 40 to 60 % in place face-on; opening 6 to 12 m² | under 35 % in place; stopped |
| Hall: wall away from a junction, wall at a junction (the cell 178 pass), gable | crosses; cross wall consumed progressively; roof stays where its bearings stand | stopped at the junction |
| Slow push from rest, full drive: brick face, brick end, board wall, thin tree | face gives within 3 s; end holds; board wall and tree give once rule 9 is in | brick face holds for good |
| A 26 kg block thrown at a wall at 2 and 10 m/s; a 100 kg beam dropped on a wall top | held at 2, one cell out at 10; at most one cell split | the wall comes down |

## 8. Verdict, lines, build order

### Pass line (three runs a pass; one binary must not straddle a threshold)

- Every house and hall pass crosses, at 10 km/h, 20 km/h and 95 % of governed speed, with no time
  under 0.5 m/s.
- The standing channel (normal plus friction from standing cells) never exceeds the sum of the
  bounds in a tick, and totals under 0.5 m/s per wall crossed.
- Largest one-tick forward change under 1.0 m/s on single-leaf passes, under 1.5 on the hall.
- House face-on: 40 to 60 % in place, opening 6 to 12 m², least forward speed at least 4 m/s from
  9.4 (today 5.63, build 3 3.63, both MEASURED; references, not targets).
- The cut and tick-rate fixtures agree within their tolerances.
- Tick p95 at most 12 ms on the house passes on this machine (budget 15.6; today 7.6 face-on,
  MEASURED), and the law's own systems under 1 ms p95.

### Kill line (any one: stop, no fix round)

- A hull meeting single-leaf brick or timber at 2.8 m/s or more spends 0.25 s under 0.5 m/s.
- A one-tick forward change of 2.3 m/s or more on a breakable fixture that the freed pieces' mass
  does not explain (`largest freed piece × v / M`).
- The standing channel exceeds the sum of the bounds after one implementation fix.
- House face-on under 35 % in place.
- The two tick rates differ by more than 25 % in the body's momentum change.
- Tick p95 over budget on a house pass with the law's systems above 2 ms.

### Build order (one builder, a clean rebuild from `main`; `lane/cutter-geo` is not merged)

Before the first commit: drain the backlog rows naming `sweep.rs`, `sweep_intake.rs`,
`breakage.rs`, `contact_transfer.rs`.

1. **The support check hears no contact** (the content of `4f6f7089`). Test
   `a_standing_structure_hears_no_contact`; mutant: a contact press restored.
2. **The clamp and its seam** (from `0a063f89`, bounds per point). Guard
   `a_bounded_point_returns_its_bound_over_the_step`; mutants: clamp removed; the bound given whole
   to each substep.
3. **The law as pure functions**: block, ways, `F`, `W`, `J`, from the structure and the material
   rows. No wiring.
   - `joints_between_cells_pressed_together_carry_nothing`: two cuts, one price. Mutant: each
     cell's joints summed.
   - `a_backed_block_leaves_through_the_free_face_behind_it`: two layers price as one. Mutant: the
     front layer alone.
   - `a_boxed_in_cell_costs_its_compaction_and_an_open_one_its_splitting`, read from the rows.
     Mutant: the two swapped.
   - `a_cell_returns_its_strength_for_no_longer_than_its_give`: the same impulse at two step
     lengths. Mutant: `F · dt`.
4. **Wiring, and the sweep goes.** Bounds written after the narrow phase; work read from
   `ContactTransfers` after the solve; spent cells freed at rest through the existing departure
   path; the belt's staged force bounded. Deleted in the same commit: casting, judge, train,
   exchange, the hook's filter, their tests.
   - `a_cell_that_took_its_work_is_freed_at_rest_and_no_other`. Mutants: freed with the impulse;
     freed on a bound that did not bind.
   - `a_light_body_is_held_where_a_heavy_one_passes`, thresholds read from the rows. Mutant: the
     speed dropped from the bound.
   - `a_belt_bears_on_a_cell_up_to_its_strength`. Mutant: the staged force unbounded.
   - `net::ram`: the hull's standing channel is within the bounds, on the shipped plugin.
   **This is the first playable build.**
5. **Measure**: section 7, the tables, the attribution of every tick above 0.5 m/s, clips. No code.
6. **Creep price** (rule 9): the minimum cut for blocks that held under load, cached per block
   until the structure changes; or, if its cost or its prices fail, delete `mincut.rs` and state
   the standing-start holds as the law. Tests: a board wall and a tree pushed from rest.
7. Only if the kill line's sinking item or the clips ask: `SpeculativeCcd` on vehicles.

**What the owner should feel in the first playable build.** At 10 to 20 km/h into a house wall: no
hitch at the wall; the speed sags as the tank shoves rubble, and recovers. Along the hall's cross
wall: the wall crumbles ahead of the hull, not all at once. Leaning on a brick face from a
standstill: it gives after a moment. Leaning on a wall's end: it holds until bumped.

### The three biggest risks

1. **The loose-piece channel.** Build 3 lost 3.16 m/s to pushed pieces on the house and 6.65 m/s on
   the gable before it stalled (MEASURED). This law bounds standing matter only. Pieces pinned
   among the hull, the ground and each other are unbounded, and a plug that stays one 8.7 t piece
   is a 1.3 m/s tick at 10 m/s.
2. **The release overlap.** Up to two ticks' travel. Pieces start inside the hull's colliders and
   may leave through its top or sides; the belts add their own contact law. Held obstacles show
   today's push-back.
3. **The crawl prices.** The pressed block's own price over-holds timber and backed masonry from
   a standstill. The minimum cut has an unknown cost and priced the hall's cross wall at 7 to 12 MN
   where a row sliding derives to about 1 MN (the difference is unexplained). No lever is
   modelled, so a thick tree is under-charged.

**What would end this family of laws.** If the instrument shows the standing channel inside its
bounds and under 0.5 m/s, and the hull still stalls or jolts, then no law about standing cells can
help: the stopper is loose matter. The next question is then what fresh rubble is to a vehicle
(how large pieces break, whether a piece crushes under a hull), not how a wall answers. A second
ending: if the crawl prices need a third mechanism patched on, stop and state the standing-start
behaviour as it is.

## Not verified

- The hull's mass and drive (taken from the history; no tank file key found by grep).
- The hull front's height and so the patch.
- Why the minimum cut priced the hall cross wall at crushing.
- That the belt's friction adapter is bounded by the clamped normal impulse in every stage (one
  read site seen, `src/avian_ext/surface_motion.rs:324`).
- That freed cells inside a multi-collider hull leave forward.
- The cost of one pass over standing contact pairs with hundreds of resting pieces.
- Whether replicas need the pressed-block state (none is recorded; the record still carries
  separations only, so `PROTOCOL_REV` should stay 107).
- Every tree number: stem mass, rotation to failure, and the root plate's work, which is `Unknown`
  in data.
