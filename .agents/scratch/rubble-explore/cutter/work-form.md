# Work form: the lane's checkpoints, as run

The law as built is stated once, in `.agents/scratch/strength-limited-contact-design-2026-11-21.md`, "As built". The checkpoints below are kept as they were run; a statement marked SUPERSEDED describes code a later checkpoint replaced.

# Work form: checkpoint 1

Subject: the game's building model (game state only). Lane `lane/work-form`, law commit `e9751c45` on `4edd1852`.

The stall is gone at the plain wall: 9 of 9 passes at 10 km/h cross, and 2 km/h and the standing start cross too (MEASURED).
Over-freeing is real: with the work form 29 to 96 % of the cells the contact law frees leave on ticks when only loose pieces, not the hull, parted a block (today 8 to 81 %), and every building keeps less in place (MEASURED).

## The law as built at checkpoint 1 (SUPERSEDED: the count reads the step's mean speed and a second clause spends a way; see the design doc)

Every step, each contact point on a standing cell adds to that cell the normal impulse the solver returned there times the speed its body advances into the cell there as the solve left it (negatives add nothing). Impulse (N s) times speed (m/s) is the step's work (J): the step length is inside the impulse. Nothing removes it. A block is charged with all the work on its pressed cells; when that reaches the work `W` of a way the block gives, that way's `leaves` part.

## Design points settled at checkpoint 1

- **Where the work lives.** `Live::work`, one `f64` per law chunk, on the owner's `Breakage` (`src/construction/breakage.rs`). Zero at spawn, built from the structure, never from an asset.
- (SUPERSEDED in part: an owner's cut now runs in its task; entries, deposits and departures commit in owner order.) **Written** only in `release` (`src/construction/breakage/bounded.rs`), after the physics step, by `settle`, once per owner. Owners settle in parallel (`per_owner`, compute pool): each writes only its own `Live::work` and reads the solver's impulses (`Bounds`) and the bodies' post-step velocities, both immutable there. Cuts and departures then commit serially in owner order.
- **Read** in `bound_contacts` and `bound_belts` before the step (`Standing::work`, `give_along` takes `taken(work, pressed)`), to size the bound's work left, and in `settle` to decide parting.
- **Ordering relied on.** `bound_belts` and `bound_contacts` before the solve; `read_bounds` after the last substep; `release` after that and before the support check. Within an owner the sum runs belts first, then contacts in body-entity then narrow-phase order, so it does not depend on worker assignment (DERIVED from the code; no test).
- **Impulse source.** `Bounds` now keeps the returned impulse per contact point (`Bounds::read`, `Bounds::impulse`), summed over substeps. `ContactTransfers` was not used: it has no contact-id key. The two agree at warm-start coefficient 1 (DERIVED, not tested).
- (SUPERSEDED in part: a block parts by the first spent of its ways.) **A give's sum** is over the block's pressed cells, whatever body or direction did the work. Of a block's ways the least `W` is reached first (work only rises), so that one parts. Parting frees the give's `leaves`.
- **No double count.** Work is added once per point to its cell; gives only read it. `Bounds::spent`, the group's `room` and the per-group `returned` state are gone.
- **Belts: same form.** A staged belt force on standing matter is capped at the cells' bound as before; the kept impulse times the belt body's advance into the cell is added to the cell; the cells leave when their work reaches `W`. The belt no longer frees a cell merely for pushing harder than `F`.
- **Rollback and wire.** The work is server state only. It is not in a replicated type and not in a rollback-registered component, so `net::protocol` is unchanged and `PROTOCOL_REV` stays 108 (DERIVED from the diff: no replicated type edited; the ADR texts were not re-read in full).
- (SUPERSEDED: a piece carries no work; the cell's count is dropped when it leaves.) **Cells that leave take their work.** `Live::group` clones the vector into the piece; nothing reads it there. Remaining cells keep theirs.
- **Points with no open give** are still read and still add work.

## What the statement does when taken literally

- **Residue lag** (DERIVED, seen in the belt fixture). The solver's work bound is exactly the work left, while the count uses the post-solve speed, so a rationed step delivers a little under what is left. The sum reaches `W` a few steps late (fixture: up to 5 ticks); in those steps the block returns almost nothing while still standing. Not isolated in the passes.
- **A block can part with no press that tick.** `W` depends on the push direction; when it falls to the work already on the cells, the block parts (fixture, tick 21 at 4.5 m/s).
- **Work under a body at rest.** A 57 t box stopped inside the fixture wall adds about 135 J/s from solver jitter (MEASURED, fixture). In the house observation, pieces lying on standing cells add 0.04 J/s at the median and 0.78 J/s at most, against `W` of 750 to 1950 J (MEASURED), and stop when they sleep.

## Passes

MEASURED, release build, ordinary input path (`scripts/perf/collapse.sh --ram`), one tank, compute pool capped at two threads (`pools=2/1/1`; the machine has ten cores and the schedule thread is not confined). Both arms re-recorded in this session with the same probe: `today` is `4edd1852`. Tick 15.625 ms.

Columns per arm: result; least forward speed m/s; worst one-tick speed change m/s; deepest hull-to-standing overlap m; cells that left standing (of them by the contact law); share left in place; contact-law cells freed on ticks when only pieces parted a block; bodies at end; tick p50 / p95 ms / ticks over 15.6 ms.

| pass | TODAY | | | | | | | | | WORK FORM | | | | | | | | |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| wall 2 km/h | held | -0.01 | -0.09 | 0.100 | 1 (1) | 1.00 | 0 | 1 | 6.61 / 8.07 / 2 | crossed 15.22 s | 0.02 | +0.08 | 0.045 | 194 (194) | 0.54 | 102 (53 %) | 191 | 4.29 / 6.21 / 3 |
| wall, standing start | held | -0.09 | -0.10 | 0.097 | 2 (2) | 1.00 | 0 | 2 | 6.27 / 7.04 / 2 | crossed 3.36 s | 0.36 | -0.08 | 0.129 | 203 (203) | 0.52 | 84 (41 %) | 200 | 3.41 / 5.59 / 1 |
| wall 10 km/h a | held | -0.13 | -0.12 | 0.373 | 136 (135) | 0.67 | 17 (13 %) | 133 | 5.20 / 6.88 / 2 | crossed 3.19 s | 1.57 | -0.14 | 0.113 | 220 (220) | 0.50 | 103 (47 %) | 217 | 4.64 / 7.06 / 2 |
| wall 10 km/h b | crossed 3.06 s | 1.98 | -0.07 | 0.250 | 177 (177) | 0.57 | 15 (8 %) | 177 | 3.87 / 7.28 / 2 | crossed 3.27 s | 1.49 | -0.16 | 0.119 | 216 (215) | 0.48 | 98 (46 %) | 213 | 4.37 / 7.25 / 2 |
| wall 10 km/h c | | | | | | | | | | crossed 3.19 s | 1.59 | -0.15 | 0.120 | 195 (195) | 0.54 | 56 (29 %) | 193 | 4.40 / 7.24 / 2 |
| wall 10 km/h d | | | | | | | | | | crossed 2.92 s | 2.23 | -0.16 | 0.128 | 210 (210) | 0.49 | 78 (37 %) | 207 | 4.56 / 7.49 / 2 |
| wall 10 km/h e | | | | | | | | | | crossed 3.22 s | 1.64 | -0.22 | 0.340 | 216 (215) | 0.49 | 113 (53 %) | 214 | 3.90 / 10.26 / 3 |
| wall 10 km/h f | | | | | | | | | | crossed 3.27 s | 1.49 | -0.20 | 0.165 | 195 (195) | 0.53 | 59 (30 %) | 191 | 3.72 / 8.74 / 4 |
| wall 10 km/h g | | | | | | | | | | crossed 3.22 s | 1.57 | -0.15 | 0.113 | 214 (213) | 0.50 | 89 (42 %) | 211 | 3.82 / 6.99 / 2 |
| wall 10 km/h h | | | | | | | | | | crossed 3.25 s | 1.54 | -0.17 | 0.108 | 196 (196) | 0.53 | 72 (37 %) | 193 | 4.23 / 7.66 / 2 |
| wall 10 km/h, clip run | held | -0.21 | -0.14 | 0.164 | 116 (115) | 0.72 | 28 (24 %) | 114 | not timed | crossed 3.25 s | 1.65 | -0.17 | 0.119 | 201 (201) | 0.51 | 74 (37 %) | 199 | not timed |
| wall 20 km/h | crossed 1.70 s | 4.24 | -0.13 | 0.016 | 182 (182) | 0.58 | 17 (9 %) | 181 | 3.30 / 6.75 / 3 | crossed 1.75 s | 3.75 | -0.22 | 0.202 | 225 (225) | 0.47 | 78 (35 %) | 218 | 3.91 / 6.25 / 4 |
| house long wall 10 km/h | crossed 4.50 s | 2.19 | -0.05 | 0.077 | 379 (288) | 0.60 | 123 (43 %) | 321 | 5.83 / 10.23 / 1 | crossed 4.47 s | 2.11 | -0.12 | 0.115 | 454 (380) | 0.55 | 228 (60 %) | 408 | 8.20 / 11.51 / 3 |
| house 10 km/h, 75 s run | crossed 4.53 s | 2.10 | -0.06 | 0.074 | 386 (304) | 0.61 | 110 (36 %) | 334 | | crossed 4.47 s | 2.08 | -0.08 | 0.149 | 470 (401) | 0.53 | 249 (62 %) | 428 | |
| house 10 km/h, clip run | held | -0.11 | -0.24 | 0.409 | 83 (81) | 0.92 | 19 (23 %) | 81 | not timed | crossed 4.67 s | 1.80 | -0.10 | 0.111 | 428 (353) | 0.58 | 182 (52 %) | 367 | not timed |
| house long wall 20 km/h | crossed 2.78 s | 3.66 | -0.12 | 0.098 | 386 (295) | 0.59 | 116 (39 %) | 356 | 5.89 / 9.26 / 0 | crossed 2.72 s | 3.71 | -0.10 | 0.112 | 463 (382) | 0.53 | 191 (50 %) | 426 | 6.84 / 11.06 / 0 |
| house long wall governed | crossed 1.44 s | 9.18 | -0.13 | 0.000 | 448 (358) | 0.54 | 181 (51 %) | 420 | 7.23 / 10.24 / 6 | crossed 1.42 s | 8.98 | -0.17 | 0.108 | 547 (519) | 0.49 | 286 (55 %) | 505 | 7.27 / 11.22 / 2 |
| hall long wall 10 km/h | held | -0.18 | -0.21 | 0.021 | 61 (61) | 0.94 | 15 (25 %) | 54 | 3.15 / 6.82 / 1 | crossed 8.25 s | 1.54 | -0.44 | 0.000 | 360 (160) | 0.71 | 125 (78 %) | 273 | 7.59 / 10.26 / 12 |
| hall long wall 20 km/h | held | -0.51 | -0.36 | 0.239 | 97 (97) | 0.90 | 36 (37 %) | 85 | 3.38 / 8.01 / 2 | crossed 6.27 s | 1.42 | -0.88 | 0.278 | 334 (134) | 0.78 | 89 (66 %) | 222 | 7.55 / 10.84 / 16 |
| hall long wall governed | crossed 3.34 s | 4.15 | -0.37 | 0.047 | 297 (98) | 0.82 | 50 (51 %) | 200 | 7.23 / 12.72 / 13 | crossed 3.88 s | 1.22 | -0.44 | 0.103 | 505 (300) | 0.54 | 255 (85 %) | 451 | 10.78 / 12.48 / 14 |
| hall junction 10 km/h | held | -0.10 | -0.97 | 0.123 | 9 (9) | 0.99 | 5 (56 %) | 9 | 3.11 / 4.69 / 2 | held | -0.44 | -0.64 | 0.064 | 476 (299) | 0.58 | 287 (96 %) | 425 | 10.16 / 12.16 / 9 |
| hall junction 20 km/h | held | -0.40 | -0.61 | 0.000 | 320 (154) | 0.80 | 125 (81 %) | 205 | 7.56 / 8.48 / 8 | held | 0.01 | -0.83 | 0.000 | 476 (337) | 0.57 | 297 (88 %) | 423 | 11.14 / 12.74 / 6 |
| house end-on governed (`ram.json`) | crossed 1.66 s | 7.88 | -0.38 | 0.000 | 752 (372) | 0.24 | 230 (62 %) | 721 | 10.23 / 14.99 / 17 | crossed 1.84 s | 6.85 | -0.39 | 0.124 | 819 (550) | 0.18 | 387 (70 %) | 789 | 9.04 / 15.27 / 25 |

Against the earlier figures (`clips-brick/instrument/wall-stall/out-depth/passes.txt`): the earlier lane's `today` held the wall at 10 km/h 4 of 4 with overlap 0.40 to 0.49 m, crossed the hall wall at 10 and 20 km/h (8.95 s, 8.52 s) where this session's `today` held it, and its lever arm left 0.01 to 0.39 of the wall in place where the work form leaves 0.48 to 0.54.

### Why the held passes hold (last 2 s, MEASURED)

- **Work form, junction 10 and 20 km/h: an anchoring stall on rubble, not the wall.** The hull is 8.9 and 9.2 m past the contact point, still moving at 0.57 and 0.28 m/s, riding 0.16 and 0.29 m above its approach height. Belts' forward force 15 and 0 kN of a traction capacity of 20 and 0 kN (304 kN at contact); standing matter returns 0 kN along forward; loose pieces 31 kN.
- **Today, wall 10 km/h a:** belts 68 of 68 kN forward; standing matter -182 kN along forward, 494 kN of normal load across travel: the pinch the lane was opened for.
- **Today, wall 2 km/h and standing start:** belts 346 and 348 kN at capacity against -348 and -350 kN from standing matter; 1 and 2 cells freed.
- **Today, junction 10 km/h:** belts 339 kN against -349 kN from standing matter; 9 cells freed, travel 0.05 m.
- **Today, hall wall 10 km/h:** belts 144 of 174 kN; standing matter -197 kN, 375 kN across travel.

### Over-freeing and the cascade

- Parting rows by presser (work form): at the wall the hull parts 5 to 25 block rows and pieces 77 to 105; at the house 30 and 161; at the junction 3 and 208 (MEASURED). Today at the wall: hull 39 to 58, pieces 13 to 21.
- The ways pieces part are weak ones: `W` p10 / p50 / p90 of 4 / 79 / 701 J at the wall, 5 / 89 / 1238 J at the house, 14 / 426 / 3614 J at the junction (MEASURED).
- Cells that left standing rise from 177 to 195–220 at the wall (crossing passes), 379–386 to 454–470 at the house, 297 to 505 at the hall wall governed, 320 to 476 at the junction at 20 km/h, 752 to 819 end-on.
- The probe's "cells the hull never touched" reading is not reported as the measure: it samples contacts after the step, so a cell freed on the tick it is first touched reads untouched (today's governed house reads 1 touched cell). It gives 54 to 62 % at the wall and 84 to 99 % at the buildings in the work form, and 69 to 100 % today; treat it as an upper bound in both arms.

### 60 s after a crossing (house long wall, 10 km/h, 76 s logged)

Nothing parts after 9.4 s from contact in the work form (9.3 s today); 0 cells leave from then to the end. No body presses a standing cell that has a way open after 14.3 s (12.6 s today). 65 pieces are still awake at the end (66 today), in both arms: that is not this change's.

### Tick time

Two-thread p95 stays under 15.6 ms in every pass except the end-on house (15.27 against 14.99 ms). It rises with the body count: hall wall 10 km/h 6.82 to 10.26 ms with 54 to 273 bodies and 1 to 12 ticks over budget; hall wall 20 km/h 2 to 16 ticks over; end-on 17 to 25. The clip runs render on the same machine and are not timed.

## Tests

`construction::breakage::bounded` (5), `construction::fracture::bound`, `avian_ext::reaction_bound`, `tests::avian_reaction_bound`:

- (SUPERSEDED: cells also leave by the second clause.) work per cell never decreases, a cell leaves if and only if a row's taken work reached its way's, and the work counted is at most the kinetic energy the body lost (a double count would break that bound; there is no separate duplicated-give fixture);
- a body placed inside and pushed out adds exactly none; a held body adds 1.5e-3 J in 6 s (asserted small, not zero);
- a belt keeps what standing matter returns, and the cells it advances into leave when their work reaches their price;
- the read impulse equals the momentum the body lost.

Deleted with the old counting: the `crossed` assertion, the spent-report assertions, the tests' `spent` arguments. Worker-assignment independence has no test (SUPERSEDED: `owners_come_back_in_their_order_on_any_pool`).

## Deleted

`crossed`; `Bounds::spent`, the group's `room`, `spent` and `returned` fields and the spent test in `read_bounds`; the `Give` resource (share kept per cell, cleared on any step not read as spent); `Block::spent`, `root`, `arm`, `to_owner`; `BlockRow::yielded` and `spent`; the choice of one give by what it returns at the block's speed; the belt's free-on-over-push. Readouts: `BlockReadout.yielded` and `spent` became `taken_j`.

Line counts (`git diff --numstat`): `src` outside tests +349 / -286; tests +238 / -89; docs +22 / -16. The change adds lines net.

## Commits

- `e9751c45` the law, its tests, the readout and the doc sentences it made false.
- this report.

## Clips

Two panels, `TODAY` | `WORK FORM`, labels burned in, 8 frames per second of game time, stills at 3, 6 and 10 s beside each:

- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/wall-10kmh-today-vs-work-form.mp4`: does the hull come out the far side of the wall, or stop inside it.
- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/house-long-wall-10kmh-exit-side.mp4`: how much of the house still stands once the hull is through. The `TODAY` panel is a run that held.
- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/hall-junction-20kmh-raised.mp4`: how far the hull gets into the hall, and how much wall falls around it.

Probe patch, run scripts, analysis scripts and their outputs: `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/instrument/`.

## Not verified

- One run per building pass and arm; today's wall, house and hall wall outcomes vary run to run (wall 10 km/h held 2 of 3 here, house 10 km/h held 1 of 3).
- The work on cells with no open way, and the total work on a building over time, were not logged.
- The residue lag and the with-no-press parting were seen in fixtures only; their share of the passes' overlap is not isolated.
- (SUPERSEDED for the order, as above.) Summation order independent of worker assignment, and `Bounds` agreeing with `ContactTransfers`: derived, untested.
- No hosted run, no play by a person, no two-core machine: the pool cap is not a two-core allocation.
- The six lenses were not run by this lane.

# Work form: checkpoint 2

Subject: the game's building model (game state only). Lane merged with main `6a443e8f` (`484541d0`); every number here is on the merged lane, checkpoint-1 law, release build, pool capped at two threads, the per-point probe logging in every run (so tick times carry its cost).

The depth-gained advance was **not built**: the zero-build replay shows it cannot see a fast hull. The stall stays gone under the law as it stands: 11 of 11 plain-wall passes cross (2 km/h, standing start, 8 at 10 km/h, 20 km/h; MEASURED).

## A1: the replay, and why the build stopped

Method (MEASURED from logged rows). The probe logged, per contact point on a standing cell and per step: impulse returned, the narrow phase's overlap at the step's start, the point's travel into the cell from the body's pose before and after the step, and the body's speed before and after. `replay2.py` recomputes the work under each reading and compares it with the built one, per presser and at each parting row.

How depth is read within a step: a contact is (body, cell, belt or solver), which names no manifold point. Its number is how far the body stands back from its deepest in the cell. A contact first seen this step starts at the gap still open at the step's start (zero when already overlapped), so a first-step entry counts everything past the cell's face. The step's advance is the impulse-weighted mean of its points' pose travel into the cell; the gain is the advance less the stand-back.

| reading, J over the run | fixture: 57 t box entering at 4.5 m/s | wall 2 km/h hull | wall 10 km/h hull (a, b) | wall 20 km/h hull | house 10 hull | hall 10 hull | junction 20 hull |
|---|---|---|---|---|---|---|---|
| built: impulse x end speed | 9081 | 28517 | 28310, 27058 | 24045 | 27919 | 31638 | 49608 |
| depth gained, from poses | 10976 | 27092 | 15665, 13532 | 8200 | 12543 | 25677 | 7747 |
| depth gained, from the overlap | 11243 | 26870 | 15300, 13454 | 8001 | 12463 | 25422 | 7746 |

- **Fast bodies: seen in the fixture, not in the game.** The fixture's first-step entry counts in full (first-step pairs 9640 J against the built 7565 J). In the passes the hull's work falls to 55 %, 50 % (10 km/h), 34 % (20 km/h) and 16 % (junction 20 km/h) of the built reading. At 2 km/h it is 95 %.
- **Why (MEASURED).** On the steps where the standing cells return most, the hull's pose barely advances while its velocity is kept: at 20 km/h the hull moves 84 mm, then 7 mm, then 84 mm in three consecutive steps with its speed reading 5.39, 5.38, 5.38 m/s throughout, and the 7 mm step is the one that returned 760 N.s. Impulse-weighted, the hull's pose travel is 0.38 of end speed x dt at 20 km/h, 0.61 at 10 km/h, 0.18 at the junction, 1.04 at 2 km/h; 76 % of the hull's impulse at 20 km/h arrives on steps whose pose travel is under half of speed x dt. A depth reading is honest about the pose and so counts a fraction of the kinetic energy the hull lost (that step: 355 J by pose, 4090 J by speed, about 2940 J of kinetic energy lost, DERIVED from 57 t and the speed change). Which part of the solve holds the pose back while the velocity stays was not traced; the vendored fork is not this lane's to change.
- **Stall.** At the tick the built law parts a hull's block, the depth reading holds under half of the built work in 4 of 19 and 6 of 22 hull partings (10 km/h), 7 of 26 (20 km/h), 9 of 32 (house), 2 of 4 (junction); medians 0.69 to 0.91. A replay cannot run the changed trajectory, so "stall returns" is not shown, only that a quarter to a half of the hull's partings would wait for more than twice the travel.
- **Pieces gain.** Loose pieces' work rises under the depth reading wherever they are fast (wall 20: 30137 to 43757 J; junction: 231969 to 338576 J), because a slowing body's end speed under-counts its step. The hull is cut and the cascade is fed: the opposite of the purpose.
- **Rest.** Fixture, 57 t box stopped in a leaf: built 46.19 J/s over 2 to 4 s, 0.001 J/s over 4 to 6 s, zero after; depth 0 from 2 s on. Fixture, 1 t box held by 5 kN: built 0.14 J/s in the first 2 s then zero; depth 0.0001. In the game a parked hull does no work under any reading (below).
- A fourth reading (impulse x mean of the speeds before and after, beyond the body's own deepest travel) keeps the hull (20 km/h: 22792 J) but is not silent at rest (40 J/s in the same fixture window) and halves the pieces; replayed only, `replay2.py`.

Decision: stop on A2. The unbuilt implementation (a `Marks` resource keyed by owner then contact, rewritten whole each `release`) passed the law's 14 tests before the replay finished; it is kept as a patch beside the instruments, not in the tree.

## B: whose work a parting block cashes (MEASURED, report only)

Own share: of the work on the block's cells when it parts, the part the row's own body added (impulse x end speed, summed since the log began). Per parting row under a loose piece:

| pass | piece rows | own share p10 / p25 / p50 / p75 / p90 | rows under 10 % | rows with none at all | piece speed change over the step p50 / p90 / max, m/s | cells parted | of them through a way with own share under 10 % |
|---|---|---|---|---|---|---|---|
| wall 2 km/h | 90 | 0 / 0.001 / 0.24 / 0.69 / 0.95 | 38 (42 %) | 17 | +0.07 / +0.37 / +2.20 | 209 | 48 (23 %) |
| wall 10 km/h a | 81 | 0 / 0 / 0.04 / 0.38 / 0.71 | 48 (59 %) | 21 | +0.12 / +0.73 / +3.34 | 199 | 59 (30 %) |
| wall 10 km/h b | 85 | 0 / 0 / 0.004 / 0.27 / 0.84 | 57 (67 %) | 29 | +0.06 / +1.41 / +4.01 | 202 | 70 (35 %) |
| wall 20 km/h | 86 | 0 / 0 / 0.03 / 0.38 / 0.69 | 50 (58 %) | 29 | +0.14 / +2.14 / +6.82 | 196 | 58 (30 %) |
| house long wall 10 | 162 | 0 / 0 / 0.02 / 0.42 / 0.93 | 97 (60 %) | 46 | +0.10 / +1.84 / +2.94 | 337 | 108 (32 %) |
| house long wall 20 | 155 | 0 / 0 / 0.10 / 0.52 / 0.96 | 78 (50 %) | 36 | | 346 | 100 (29 %) |
| hall long wall 10 | 91 | 0 / 0 / 0.08 / 0.40 / 0.93 | 51 (56 %) | 21 | +0.14 / +1.36 / +3.27 | 149 | 59 (40 %) |
| hall long wall 20 | | | | | | 225 | 98 (44 %) |
| hall junction 20 | 218 | 0 / 0 / 0.10 / 0.64 / 0.97 | 108 (50 %) | 51 | +0.20 / +2.11 / +5.82 | 357 | 142 (40 %) |

Half to two thirds of the blocks that part under a loose piece part on work that piece did not add; a quarter to a third never had work from it at all. The other work is other pieces', not the hull's (the hull's share of those rows' work: median 0, over half in 5 to 20 rows per pass). Pieces mostly gain speed over the parting step. A cell counted once per tick, by its row of largest own share.

The 51.65 m/s piece: nothing pressed it. A piece that ends under the terrain falls freely to the play-area floor and is retired there: server log `retiring piece below the play-area floor piece=8408v0 speed_mps=52.02 height_m=-100.66 floor_m=-100` (house 10 km/h; junction 20 km/h retires one at 49.82 m/s), its speed rising 0.153 m/s per tick, which is gravity over one step. DERIVED: a 132 m fall from the house's ground gives 50.9 m/s. It appears in both of checkpoint 1's arms and still on merged main in 2 of 17 runs here; the 60 s house rerun had none (fastest 7.65 m/s). How the piece got under the terrain was not traced: the census logs speed, not position.

## C

Commit `844fb673`: a piece's `Live` carries an empty work vector; the module doc names "one physics step between two `release`s" and what breaks without it; `BlockRow`'s doc says why the readout cannot name a way that took the work (work is per cell, a step's impulse belongs to points several ways share) and that it reports the way the block parts by.

## D: the merged lane, checkpoint-1 law

Columns as checkpoint 1's table.

| pass | result | least fwd m/s | worst tick dv | deepest overlap m | left standing (contact law) | in place | freed on piece-only ticks | bodies | tick p50 / p95 ms / over 15.6 |
|---|---|---|---|---|---|---|---|---|---|
| wall 2 km/h | crossed 14.92 s | -0.01 | -0.08 | 0.056 | 209 (209) | 0.49 | 115 (55 %) | 208 | 4.42 / 6.42 / 3 |
| wall, standing start | crossed 3.41 s | 0.36 | -0.08 | 0.090 | 231 (230) | 0.44 | 116 (50 %) | 231 | 3.30 / 5.84 / 2 |
| wall 10 km/h a | crossed 3.28 s | 1.43 | -0.15 | 0.119 | 199 (199) | 0.51 | 77 (39 %) | 198 | 3.83 / 8.16 / 2 |
| wall 10 km/h b | crossed 3.23 s | 1.51 | -0.17 | 0.106 | 202 (202) | 0.52 | 79 (39 %) | 202 | 3.69 / 6.66 / 2 |
| wall 10 km/h c | crossed 4.16 s | 0.57 | -0.23 | 0.234 | 192 (192) | 0.52 | 48 (25 %) | 192 | 3.81 / 8.53 / 2 |
| wall 10 km/h d | crossed 3.27 s | 1.50 | -0.16 | 0.119 | 193 (193) | 0.53 | 73 (38 %) | 193 | 3.61 / 7.17 / 1 |
| wall 10 km/h e | crossed 4.14 s | 0.52 | -0.17 | 0.234 | 199 (199) | 0.52 | 54 (27 %) | 199 | 3.92 / 8.94 / 2 |
| wall 10 km/h f | crossed 3.11 s | 2.06 | -0.16 | 0.253 | 216 (198) | 0.48 | 66 (33 %) | 216 | 3.88 / 9.46 / 3 |
| wall 10 km/h g | crossed 3.11 s | 1.96 | -0.15 | 0.253 | 204 (204) | 0.51 | 92 (45 %) | 204 | 3.94 / 8.73 / 2 |
| wall 10 km/h h | crossed 3.31 s | 1.53 | -0.18 | 0.116 | 206 (206) | 0.50 | 92 (45 %) | 204 | 4.04 / 7.14 / 2 |
| wall 20 km/h | crossed 1.72 s | 3.81 | -0.18 | 0.188 | 197 (197) | 0.52 | 63 (32 %) | 197 | 3.72 / 6.24 / 2 |
| house long wall 10 | crossed 4.38 s | 2.15 | -0.11 | 0.152 | 431 (338) | 0.56 | 195 (58 %) | 379 | 7.38 / 12.47 / 1 |
| house long wall 20 | crossed 2.48 s | 4.43 | -0.13 | 0.127 | 418 (346) | 0.58 | 189 (55 %) | 393 | 6.45 / 10.09 / 0 |
| hall long wall 10 | crossed 8.67 s | 1.15 | -0.23 | 0.017 | 343 (149) | 0.73 | 114 (77 %) | 267 | 8.41 / 10.17 / 12 |
| hall long wall 20 | crossed 6.47 s | 1.32 | -0.94 | 0.011 | 342 (225) | 0.75 | 195 (87 %) | 298 | 9.12 / 11.92 / 15 |
| hall junction 20 | held | 0.09 | -0.65 | 0.000 | 484 (357) | 0.55 | 310 (87 %) | 468 | 11.37 / 13.02 / 9 |
| house end-on governed | crossed 1.83 s | 6.90 | -0.41 | 0.144 | 786 (497) | 0.18 | 348 (70 %) | 756 | 7.52 / 13.16 / 5 |

Two of the eight 10 km/h passes (c, e) slow to 0.5 m/s and take 4.15 s; both are the deep-overlap kind below.

### Parked hulls (MEASURED)

| run | while the hull moved | after its last motion |
|---|---|---|
| hull nudged 0.1 m into the wall, demand off | 31 cells left (4 parting rows: 3 hull, 1 piece), deepest overlap 0.033 m | 65.6 s: 0 cells, 0 contact rows, 0 J/s |
| hull driven half through, demand off at 0.56 m/s, stops in the breach | 147 cells while driven, 11 more in the 0.25 s after demand ended (2 hull rows, 8 piece rows); pieces settle for 9.4 s more (1.74 J/s built reading over that window) | 54.8 s: 0 cells, 0 contact rows, 0 J/s |

Nothing parts with nobody moving: once the hull and the pieces sleep, no contact row exists, so no reading adds work. Before sleep the 11 cells are the over-freeing of section B, not a rest effect.

### Deepest overlap and the residue lag (MEASURED)

Deepest hull-to-standing overlap 0.056 to 0.253 m at the wall (checkpoint 1: 0.045 to 0.340 m). A cell the hull overlaps by more than 2 cm stands a median 8 steps at 10 km/h and 2 at 20 km/h; of those cell-steps the cell returns under 1 N.s (the hull's weight over a step is about 8700 N.s) in 51 % (10 km/h a), 67 % and 72 % (c and f, the 0.23 to 0.25 m passes), 38 % (2 km/h), 11 % (20 km/h), 24 % (house), 17 % (hall). The deepest cells of pass a returned 2, 0, 0, 0, 0, 0 N.s over six steps while the overlap grew 0.108 to 0.122 m. The advisor's guess holds at 10 km/h: most of the deep overlap is a block that has returned what it has and stands, returning nothing, until its work reaches the way's.

### What carries the hull at the junction hold (MEASURED, last 2 s)

- Belt law: 510 N on soil, nothing else (0.1 % of the weight); over the whole log it loads only soil and standing cells, never a loose piece. Traction capacity 200 N, belts' forward force 172 N, demand 1.0.
- Solver, vertical impulse on the hull per step: 6247 N.s from pieces on the belt-face colliders, 2458 N.s from pieces on the hull body, together 8705 N.s of a weight of about 8737 N.s.
- The hull stands 0.30 m above where it met the junction and still creeps forward at 0.43 m/s (0.09 to 0.93).

So the hull is lifted clear of the soil onto loose pieces: 72 % of its weight on pieces under the belts, 28 % on its belly. The belts are not spinning on pieces in the belt law's terms: the law finds no ground under them and so has no load, no slip and no drive. Belt slip and piece speed under the belts have no reading for that reason.

## Clips

None. The second panel (`DEPTH-GAINED ADVANCE`) needs a build that A1 stopped.

## Commits

- `484541d0` merge of main `6a443e8f`.
- `844fb673` section C.
- this report.

Scripts, plans, outputs and the unbuilt patch: `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/instrument/c2/`.

## Not verified

- A replay holds the trajectory fixed: what a depth-gained build would do in play is inferred, not run.
- Why the hull's pose stalls on a step where its velocity does not: not traced into the solver.
- How a piece ends under the terrain: not traced.
- One run per pass apart from the wall at 10 km/h; tick times include the probe's per-point logging.
- The junction's hull mass is taken as 57 t; the piece speed under the hull was not logged.
- B's own share uses the built reading, summed from the log's start, per body; work added before a body was renumbered by a split is another body's.
- Gates for `844fb673`: fmt check, CI clippy, 671 targeted tests (`construction::`, `avian_ext`, `avian_reaction_bound`, `net::ram`, `net::protocol`, `net::construction`, `track::`, `contact_transfer`, `debris::`), the server check. `PROTOCOL_REV` unchanged at 108. The six lenses were not run by this lane.

# Work form: checkpoint 3

Subject: the game's building model (game state only). Lane `lane/work-form` on main `6a443e8f`; law commit `a0839b2d`, commitment pass `325c6083`. Passes: release build of `a0839b2d` with the per-point probe, pool capped at two threads, one tank. The commitment pass changed no arithmetic after that build and was not re-measured.

Stall: gone, 11 of 11 plain-wall passes cross; deepest hull-to-standing overlap 0.001 to 0.066 m at the wall and 0.000 to 0.101 m over all passes (MEASURED; checkpoint 2: 0.056 to 0.253 m at the wall).
Lag (cell-steps overlapped by more than 2 cm while returning under 1 N.s): 51 to 72 % at 10 km/h before, 0 % in seven of eight passes and 4 % in the eighth after (MEASURED).
In-place shares against checkpoint 2: no pass falls by a third; the largest fall is the hall long wall at 20 km/h, 0.75 to 0.55 (27 %), and the house at 10 km/h rises, 0.56 to 0.65 (MEASURED).

The re-measure is clean on the three stop conditions, so the commitment pass was done.

## The two fixes as built

The law is in the design doc, "As built"; what this checkpoint found on the way to it:

**The lag, verified in the code before building.** The advisor's reading holds, with one correction: the game does not size the solver's ration with `reaction`. It hands the solver the block's strength for the step and the work its way has left (`W − taken`); the fork clamps each point at `max_work / v` with `v` the closing speed the solve finds at that iteration. A fully delivered ration therefore satisfies `Σ p·v_solve ≤ left`, and the count reads another speed, so the count closes on `W` from below. The mean-speed read does not make them agree.

One thing the first form got wrong, found by the law's own test: a point sits in several ways' groups and takes the least of their bounds, so "a point of the way ended clamped" also fired for a way whose points a cheaper way was holding (a block of 2384 J parted after returning 34 N.s of 543). `Bounds` now records which group dealt each point its impulse bound and its work bound, and a group is limited only by a point held at its own.

## Passes

Columns as checkpoint 2, plus that checkpoint's in-place share and the lag measure. Tick times carry the probe's logging.

| pass | result | least fwd m/s | worst tick dv | deepest overlap m | left standing (contact law) | in place | in place, checkpoint 2 | freed on piece-only ticks | bodies | tick p50 / p95 ms / over 15.6 | overlapped cell-steps returning under 1 N.s |
|---|---|---|---|---|---|---|---|---|---|---|---|
| wall 2 km/h | crossed 15.14 s | 0.00 | -0.12 | 0.066 | 243 (240) | 0.42 | 0.49 | 95 (40 %) | 238 | 4.72 / 6.36 / 2 | 8 of 113 (7 %) |
| wall, standing start | crossed 3.44 s | 0.38 | -0.08 | 0.029 | 213 (211) | 0.49 | 0.44 | 49 (23 %) | 212 | 3.51 / 5.67 / 0 | 3 of 69 (4 %) |
| wall 10 km/h a | crossed 3.14 s | 1.74 | -0.06 | 0.012 | 193 (192) | 0.55 | 0.51 | 94 (49 %) | 190 | 3.48 / 6.64 / 2 | 0 of 23 |
| wall 10 km/h b | crossed 3.17 s | 1.60 | -0.10 | 0.010 | 225 (225) | 0.45 | 0.52 | 102 (45 %) | 223 | 4.35 / 6.95 / 2 | 0 of 25 |
| wall 10 km/h c | crossed 3.12 s | 1.66 | -0.06 | 0.011 | 209 (209) | 0.50 | 0.52 | 116 (56 %) | 204 | 3.95 / 6.68 / 2 | 0 of 24 |
| wall 10 km/h d | crossed 3.08 s | 1.81 | -0.06 | 0.003 | 204 (203) | 0.52 | 0.53 | 92 (45 %) | 202 | 3.74 / 7.06 / 2 | 0 of 24 |
| wall 10 km/h e | crossed 3.14 s | 1.60 | -0.07 | 0.050 | 244 (241) | 0.42 | 0.52 | 89 (37 %) | 239 | 3.80 / 7.05 / 2 | 1 of 27 (4 %) |
| wall 10 km/h f | crossed 3.16 s | 1.66 | -0.10 | 0.003 | 225 (222) | 0.47 | 0.48 | 72 (32 %) | 222 | 3.94 / 6.64 / 2 | 0 of 23 |
| wall 10 km/h g | crossed 3.09 s | 1.78 | -0.06 | 0.020 | 199 (198) | 0.52 | 0.51 | 59 (30 %) | 196 | 3.58 / 6.78 / 2 | 0 of 25 |
| wall 10 km/h h | crossed 3.16 s | 1.70 | -0.07 | 0.009 | 231 (228) | 0.45 | 0.50 | 90 (39 %) | 225 | 4.36 / 7.03 / 2 | 0 of 22 |
| wall 20 km/h | crossed 1.69 s | 4.36 | -0.14 | 0.001 | 217 (217) | 0.48 | 0.52 | 92 (42 %) | 215 | 3.61 / 6.35 / 1 | 6 of 30 (20 %) |
| house long wall 10 | crossed 4.30 s | 2.22 | -0.08 | 0.000 | 455 (382) | 0.65 | 0.56 | 154 (40 %) | 329 | 4.85 / 12.50 / 2 | 0 of 11 |
| house long wall 20 | crossed 2.36 s | 4.52 | -0.13 | 0.101 | 509 (438) | 0.55 | 0.58 | 202 (46 %) | 438 | 8.45 / 12.98 / 2 | 3 of 15 (20 %) |
| hall long wall 10 | crossed 9.06 s | 0.51 | -0.28 | 0.010 | 414 (201) | 0.67 | 0.73 | 127 (63 %) | 329 | 9.58 / 11.64 / 8 | 0 of 8 |
| hall long wall 20 | crossed 5.44 s | 2.03 | -0.57 | 0.001 | 510 (304) | 0.55 | 0.75 | 134 (44 %) | 447 | 10.66 / 12.47 / 14 | 0 of 7 |
| hall junction 20 | held | -0.17 | -0.52 | 0.000 | 714 (617) | 0.46 | 0.55 | 327 (53 %) | 674 | 13.86 / 17.39 / 186 | 0 of 5 |
| house end-on governed | crossed 1.80 s | 6.06 | -0.49 | 0.000 | 855 (542) | 0.19 | 0.18 | 321 (59 %) | 807 | 11.13 / 14.61 / 21 | 0 of 17 |

Lag at checkpoint 2, same measure: 2 km/h 38 %, 10 km/h 51, 67 and 72 % (three passes read), 20 km/h 11 %, house 24 %, hall 17 %. What is left at 20 km/h is 6 cell-steps on the wall and 3 on the house.

Of the parting rows, about half are by the second clause (wall 10 km/h a: 58 held, 48 by work; house 10: 92 and 76; hall 20: 86 and 99; junction: 149 and 195).

Tick time without logging (same binary, one run each): junction 20 km/h mean 11.6 ms, p99 15.0, 6 ticks over; hall wall 20 mean 9.6, p99 17.2, 11 over; house 10 mean 7.8, p99 19.2, 26 over; wall 10 mean 4.1, p99 9.3, 2 over; end-on mean 11.1, p99 15.4, 2 over. The junction's 186 ticks over in the table are the probe's logging. The house's 26 against 2 in the logged run is run-to-run spread.

### Parked hulls (MEASURED)

| run | while driven | after demand ended | all at rest |
|---|---|---|---|
| hull nudged into the wall, demand off | 42 cells left (3 hull rows, 6 piece rows), deepest overlap 0.026 m | pieces settle for 0.5 s, 0 cells | 64.8 s: 0 cells, 0 contact rows |
| hull driven half through, stops in the breach | 177 cells (13 hull rows, 56 piece rows), deepest overlap 0.036 m | hull and pieces come to rest over 8.8 s: 6 cells (1 hull row, 3 piece rows) | 55.6 s: 0 cells, 0 contact rows |

No parked hull parts a cell.

### Hull pose travel against speed x dt (MEASURED)

The stall of the pose survives. Impulse-weighted over the hull's points on standing cells, pose travel over end speed x dt: 2 km/h 0.93, 10 km/h 0.65 and 0.60, 20 km/h 0.28 (checkpoint 2: 0.38), house 0.57 and 0.54, hall wall 1.01 and 0.85, junction 0.35, end-on 0.10. The step of highest return at 20 km/h moved the pressing points 3.9 mm where their end speed times the step is 37 mm. The advisor's guess, that it was the cells standing while returning nothing, is refuted: those are gone and the ratio did not move. Unexplained; backlog row destruction-11.

### Own share (MEASURED)

| pass | piece rows | own share p10 / p25 / p50 / p75 / p90 | rows under 10 % | cells parted | through a way with own share under 10 % | at checkpoint 2 |
|---|---|---|---|---|---|---|
| wall 2 km/h | 83 | 0 / 0.004 / 0.47 / 0.87 / 0.97 | 31 (37 %) | 240 | 53 (22 %) | 23 % |
| wall 10 km/h a | 62 | 0 / 0.001 / 0.43 / 0.86 / 0.96 | 20 (32 %) | 192 | 23 (12 %) | 30 % |
| wall 10 km/h b | 91 | 0 / 0.04 / 0.36 / 0.70 / 0.96 | 32 (35 %) | 225 | 51 (23 %) | 35 % |
| wall 20 km/h | 85 | 0 / 0.01 / 0.26 / 0.66 / 0.96 | 32 (38 %) | 217 | 49 (23 %) | 30 % |
| house long wall 10 | 131 | 0 / 0.02 / 0.28 / 0.88 / 1.00 | 52 (40 %) | 382 | 66 (17 %) | 32 % |
| house long wall 20 | 177 | 0 / 0 / 0.17 / 0.58 / 0.92 | 81 (46 %) | 438 | 124 (28 %) | 29 % |
| hall long wall 10 | 148 | 0 / 0.02 / 0.12 / 0.60 / 0.97 | 70 (47 %) | 201 | 40 (20 %) | 40 % |
| hall long wall 20 | 173 | 0 / 0.05 / 0.36 / 0.90 / 1.00 | 53 (31 %) | 304 | 64 (21 %) | 44 % |
| hall junction 20 | 339 | 0 / 0.04 / 0.34 / 0.76 / 0.99 | 119 (35 %) | 617 | 188 (30 %) | 40 % |

A piece's median own share rises from 0.00 to 0.10 to 0.12 to 0.47; rows under a tenth fall from 42 to 67 % to 31 to 47 %. A piece's speed change over its parting step is now about zero at the median (-0.02 to +0.11 m/s; checkpoint 2: +0.06 to +0.20).

### The junction

Still held at 20 km/h. Last 2 s: the belt law loads nothing and has no traction; the solver carries the hull through the belt-face colliders (8101 N.s a step) and the belly (721 N.s); the hull stands 0.49 m above where it met the wall and creeps at 1.07 m/s. More cells leave than at checkpoint 2 (714 against 484; in place 0.46 against 0.55). Backlog row driving-22.

## Commitment pass

Done, in `325c6083`:

- **One path.** The end-speed count is gone with the belt test's six-tick allowance, which existed for the lag. No flag, lever or probe is in the tree.
- **Deleted.** The work-only bound (`Bounds::group` accepted an infinite impulse for one point; `shares`' arm for it); `shares`' three vectors per group per substep (one buffer, reused); the `usize::MAX` sentinel never written; the unbuilt depth patch; checkpoint 1's wall and house clips.
- **Narrative.** `.agents/scratch/strength-limited-contact-design-2026-11-21.md`, "As built", states the law once: the mean-speed count, the spent clause with its before and after, the scalar-work simplification with its cost, and three rejected alternatives in prose with the measurement that refuted each (end-speed count, depth read from the contact and depth gained, spent when clamped). `construction-destruction.md` and `scripts/perf/collapse.md` had one sentence each made false; fixed.
- **Scalar work.** Stated where it is defined, `fracture::bound::taken`: one scalar per cell, a way may part on work done along another or by another body; 12 to 30 % of parted cells leave through a way whose own body added under a tenth of the work on it (MEASURED above). The module doc of `bounded` points there.
- **Tests** (each names its invariant, its reliers and its mutants): the work of a stopped body is the kinetic energy it lost, between the two mutants (end speed alone: 14 J of 186 J; start speed alone: twice); a body pushed out, held under a force or resting under gravity does none; a cell does not stand in a body past its give plus two steps' travel; a belt's cell leaves on tick `ceil(W / (F v dt))` exactly; cells leave only on a step that spent a way; a group is limited by its own bound alone (fork guard); owners come back in their order on pools of one, two and eight threads. The three mutants (both clauses off, end speed alone) were run once: three tests fail.
- **Backlog.** Closed: destruction-06 (the deal says why groups go in order, and reuses its buffer), destruction-08 (`per_owner` takes its pool; the witness above; an owner's work reads nothing another writes, so result order is all of the dependence), destruction-09. Added: destruction-11 (pose stall), destruction-12 (pieces under the terrain), driving-22 (the junction hold).
- **Not closed, and why.** destruction-07: its fix is a price on a hosted step, which this lane cannot run; the row now carries the local price of the whole stage (0.26 ms a tick at a wall, 1.8 to 2.3 ms at a hall) and says the gather's share is not isolated. The other fix, walking only construction colliders' pairs, would change the gather for an unmeasured gain. destruction-05 also names `bounded.rs` (bonded cells in a moving group answer rigidly): pricing them means parting a moving body inside the solve, which is the landing law's job and a design of its own.

## Clips

Two panels, `MAIN` (`6a443e8f`) | `WORK FORM` (`a0839b2d`), labels burned in, 8 frames per second of game time, stills at 3, 6 and 10 s beside each:

- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/wall-10kmh-main-vs-work-form.mp4`: does the hull come out the far side, or stop inside the wall. `MAIN` holds; `WORK FORM` crosses in 3.13 s.
- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/house-long-wall-10kmh-exit-side.mp4`: how much of the house still stands once the hull is through. Both cross, 4.48 and 4.50 s.
- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/hall-long-wall-20kmh-raised.mp4`: does the hull get through the hall, and how much wall and roof falls around it. Both cross, 5.20 and 5.03 s; the `WORK FORM` hall loses most of its roof over the path.

The wall clip has a new name (its label changed); the old wall and house files are deleted. Checkpoint 1's `hall-junction-20kmh-raised.mp4` is left and is stale.

## Commits

- `a0839b2d` the two fixes and their tests.
- `325c6083` the commitment pass.
- this report.

Probe patch, scripts, plans and outputs: `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-work/instrument/c3/` and `work-probe-c3.patch` beside it.

## Not verified

- The commitment pass's code (the deal's buffer, the optional dealer, `per_owner`'s pool argument) ran the tests, not the passes.
- One run per pass apart from the wall at 10 km/h; `MAIN` was run for the clips only, not timed.
- Why the hull's pose stalls; how a piece ends under the terrain. One piece was retired below the play-area floor, at 50 m/s, in 4 of this checkpoint's 27 runs: the end-on house (both runs), the hall long wall at 20 km/h and the junction (the unlogged run of each) (MEASURED from the server logs).
- The second clause's lag of one substep: a point held at its share while its group still has room reads as limited. Half the partings go by that clause and over-freeing did not rise (freed on piece-only ticks 23 to 63 %, checkpoint 2: 25 to 87 %), but the two are not separated.
- No hosted run, no play by a person, no two-core machine. The six lenses were read and applied by this lane to what it edited; no independent reviewer ran them.
- Gates for both commits: fmt check, CI clippy, 675 targeted tests (`construction::`, `avian_ext`, `avian_reaction_bound`, `net::ram`, `net::protocol`, `net::construction`, `track::`, `contact_transfer`, `debris::`), the server check. `PROTOCOL_REV` unchanged at 108.

# Work form: fix pass

Subject: the game's building model (game state only). The lens review's sections A to E (`lens-review-work-form.md`), on `5c0181ab`. Commits `9c2f3d10` (behaviour), `b0abda6a` (structure), `7b87d19a` (tests), `8ea98694` (prose), and this report's commit. Passes: release build of `8ea98694` with the per-point probe and stage timers, pool capped at two threads, one tank; the last commit changes comments and docs only.

**Stopped after the re-measure on one of its conditions.** The house met end-on at governed speed keeps 0.09 in place against checkpoint 3's 0.19, a fall of 54 % (MEASURED, two runs, the same 1.187 m³ in both). Nothing was built to counter it, and the clips were not refilmed.

The lag is also back where the hull is slow: at 2 km/h 124 of 263 overlapped cell-steps return under 1 N.s (47 %; checkpoint 3: 7 %; checkpoint 2: 38 %). At 10 km/h it is 10 of 211 over the eight passes (4.7 %), 14 % in two of them and 0 to 4 % in six. The stall has not returned: 11 of 11 wall passes cross, deepest overlap 0.005 to 0.082 m at the wall.

Which of A1 and A2 moved the end-on house is not isolated: both shipped in one build. A2's sign is known (fewer partings by the second clause at low speed: the 2 km/h lag); A1 adds work the old code dropped on steps with no group. The contact law freed as many cells as before end-on (537 against 542); the difference is in what fell with them (914 gone against 855).

## A2, read from the code

A point held at its way's strength has not delivered the work left. The fork clamps a point at the lesser of its impulse bound and `max_work / v`, `v` the closing speed the solve finds (`solve_impulse`). Both bounds are dealt by the same shares, so the impulse bound binds exactly when `v < left / (F dt)`: the body is slower in the solve than the speed that crosses the give left within the step, and the step's work through the way is `F v dt`, under the work left (DERIVED). So the ruling stands as given and was built: `Bounds::limited` reads a hold at the work bound alone.

## Passes (MEASURED)

In place is against checkpoint 3 in brackets. Tick times carry the probe's logging.

| pass | result | deepest overlap m | in place (checkpoint 3) | overlapped cell-steps returning under 1 N.s | cells through a way with own share under 10 % | left standing (contact law) | freed on piece-only ticks | tick mean / p95 ms / over 15.6 |
|---|---|---|---|---|---|---|---|---|
| wall 2 km/h | crossed 15.30 s | 0.042 | 0.51 (0.42) | 124 of 263 | 30 (15 %) of 206 | 207 (206) | 72 (35 %) | 4.19 / 6.30 / 2 |
| wall, standing start | crossed 3.38 s | 0.024 | 0.45 (0.49) | 0 of 65 | 50 (22 %) of 230 | 230 (230) | 95 (41 %) | 3.44 / 5.83 / 2 |
| wall 10 km/h a | crossed 2.88 s | 0.015 | 0.50 (0.55) | 0 of 23 | 41 (20 %) of 203 | 203 (203) | 105 (52 %) | 3.81 / 7.08 / 2 |
| wall 10 km/h b | crossed 3.20 s | 0.036 | 0.46 (0.45) | 1 of 27 | 49 (21 %) of 231 | 231 (231) | 101 (44 %) | 4.22 / 6.96 / 2 |
| wall 10 km/h c | crossed 3.17 s | 0.082 | 0.51 (0.50) | 5 of 36 | 24 (12 %) of 201 | 201 (201) | 81 (40 %) | 3.98 / 6.98 / 2 |
| wall 10 km/h d | crossed 3.16 s | 0.006 | 0.47 (0.52) | 0 of 24 | 42 (19 %) of 219 | 220 (220) | 100 (45 %) | 4.03 / 6.85 / 0 |
| wall 10 km/h e | crossed 3.11 s | 0.005 | 0.48 (0.42) | 0 of 23 | 40 (18 %) of 218 | 220 (219) | 82 (37 %) | 4.01 / 7.05 / 2 |
| wall 10 km/h f | crossed 3.11 s | 0.009 | 0.52 (0.47) | 0 of 25 | 27 (14 %) of 192 | 202 (192) | 88 (46 %) | 3.98 / 6.52 / 2 |
| wall 10 km/h g | crossed 3.16 s | 0.043 | 0.52 (0.52) | 4 of 29 | 42 (22 %) of 192 | 193 (192) | 78 (41 %) | 3.88 / 6.68 / 2 |
| wall 10 km/h h | crossed 3.09 s | 0.006 | 0.55 (0.45) | 0 of 24 | 29 (16 %) of 186 | 187 (186) | 70 (38 %) | 3.78 / 6.78 / 2 |
| wall 20 km/h | crossed 1.70 s | 0.005 | 0.48 (0.48) | 2 of 29 | 42 (19 %) of 222 | 222 (222) | 89 (40 %) | 3.70 / 6.38 / 1 |
| house long wall 10 | crossed 4.33 s | 0.030 | 0.66 (0.65) | 4 of 17 | 70 (20 %) of 353 | 432 (355) | 149 (42 %) | 6.16 / 12.22 / 0 |
| house long wall 20 | crossed 2.47 s | 0.098 | 0.58 (0.55) | 3 of 16 | 108 (26 %) of 418 | 492 (418) | 238 (57 %) | 5.64 / 12.88 / 2 |
| hall long wall 10 | crossed 9.03 s | 0.000 | 0.53 (0.67) | 0 of 6 | 71 (19 %) of 368 | 532 (368) | 243 (66 %) | 11.12 / 13.98 / 14 |
| hall long wall 20 | crossed 6.73 s | 0.002 | 0.50 (0.55) | 0 of 8 | 120 (25 %) of 487 | 571 (487) | 295 (61 %) | 10.92 / 13.39 / 21 |
| hall junction 20 | held | 0.000 | 0.35 (0.46) | 0 of 5 | 163 (29 %) of 559 | 686 (560) | 409 (73 %) | 13.55 / 15.93 / 56 |
| house end-on governed | crossed 1.81 s | 0.000 | 0.09 (0.19) | 0 of 23 | 166 (31 %) of 537 | 914 (537) | 306 (57 %) | 11.20 / 14.26 / 5 |

In place falls against checkpoint 3 by: end-on 54 %, junction 24 %, hall long wall at 10 km/h 21 %, hall at 20 km/h 9 %; the others move by a tenth or less either way. The halls' run-to-run spread is wide: the unlogged runs of the same binary kept 0.64 (hall wall 20) and 0.62 (junction) where the logged runs kept 0.50 and 0.35; the house at 10 km/h kept 0.53 against 0.66. The end-on house is the same in both.

Tick time without logging (same binary, one run each): wall 10 km/h mean 3.9 ms, p99 9.4, 2 ticks over; house 10 mean 7.1, p99 13.0, 0 over; hall wall 20 mean 8.5, p99 18.9, 17 over; junction mean 10.8, p99 17.1, 9 over; end-on mean 10.9, p99 14.8, 1 over.

Partings by clause (work reached / second clause): wall 2 km/h 56 / 55, wall 10 km/h 57 / 62, wall 20 km/h 79 / 45, house 10 92 / 84, hall 20 120 / 115, junction 208 / 102.

Own share: a piece's median 0.13 to 0.37; rows under a tenth 30 to 45 %; cells through such a way 12 to 31 % (checkpoint 3: 12 to 30 %).

Hull pose travel over end speed x dt, impulse-weighted: 2 km/h 1.03, standing start 1.02, 10 km/h 0.58 to 0.63, 20 km/h 0.30, house 0.63 and 0.41, hall wall 1.01 and 0.91, junction 0.33, end-on 0.33. Unchanged in kind (backlog destruction-11).

Parked hulls: nudged into the wall, 37 cells while driven, none after; 65.0 s at rest with 0 cells and 0 contact rows. Stopped in a breach, 163 cells while driven, 1 in the 7.1 s the hull took to come to rest, then 57.2 s at rest with 0 cells (24 contact rows, no parting). No parked hull parts a cell.

One piece was retired below the play-area floor in 5 of the 24 runs (hall wall 20 and junction, logged and unlogged; house 10 unlogged).

## C4: what stays serial (MEASURED)

Per tick, ms, from first contact to the end of the run (about 700 ticks), pool of two threads on a ten-core machine, after C1 to C3:

| part | hall long wall 20 km/h: mean | p95 | junction 20 km/h: mean | p95 |
|---|---|---|---|---|
| `bound_contacts` gather (serial) | 0.112 | 0.151 | 0.155 | 0.187 |
| `bound_contacts` pricing (tasks) | 2.429 | 5.519 | 1.734 | 2.666 |
| `bound_contacts` commit (serial) | 0.029 | 0.042 | 0.033 | 0.041 |
| `locate` (tasks) | 0.010 | 0.015 | 0.011 | 0.017 |
| `deal`, all six substeps (serial) | 0.218 | 0.348 | 0.221 | 0.277 |
| `read_bounds` (serial) | 0.004 | 0.006 | 0.005 | 0.006 |

The serial parts together are about 0.4 ms of a 15.6 ms tick; the pricing, already in tasks, is five to twenty times the gather. Pricing has one tick of 64 to 72 ms in each run, not traced.

What was done with it: the gather and commit do not dominate, so the walk is unchanged; destruction-07 carries this split, is P2, and leaves only the hosted price open. `deal` stays serial: its doc claims only the order-free least and states the measured size, and destruction-13 is its row. No parallel `deal` was built, so there is no measurement that dispatch loses on two cores; the row says so.

## Items

- **A1** done: `locate` and `deal` run whenever a point is read. Witnessed at the seam (`a_point_read_in_no_group_reads_what_it_returned`), not with two owners in the building fixture: a block with no open way is exactly a read point in no group.
- **A2** done as ruled (above). **A3** done: a group is limited when its own deal to a held point is that point's work bound, so equal deals read alike; the dealer record is gone. **A4** done: `bound_belts` bounds a block's whole staged force, so the whole is counted, stated at `Returned::Staged`. **A5** done: one assert at the write.
- **B1** done (assert in `bound_contacts`; the seam's lookups name the invariant). **B2**, **B3** done. **B5** row destruction-14.
- **B4 not done; the finding is wrong.** The offline composition (`headless_test::headless_app_on`: `SimPlugin` with the single-player spawn) is an authority without a shot clock, and 49 tests run on it; the net protocol alone inserts one. With `Res<ShotClock>` all 49 panic in `release`. Giving that composition a clock would change its shooting law (`shooting.rs` branches on the clock's absence), outside this lane. The `Option` stays, with the composition named beside it; `breakage.rs` and `ballistics.rs` read it the same way.
- **C1**, **C2**, **C3** done. **C4** above. **C5** done.
- **D1** done, with A3's tie arm and a force-held arm. **D2** done: exact per step, friction and turning off. **D4** done: the crossing is read from the row's speed, price and work. **D5**, **D6** done. **D7** done by deletion: with the work-bound reading the `impulse > 0` guard has no case. **D8** row destruction-15.
- **D3 not done as written; the fixture is right.** The belt presses under every way's strength and the way spent first is the least in work, which is not the one that holds least; taking both from one way fails (10 ticks against 627). The test doc now says why they differ.
- **E1** to **E11** done. **E12**: the readout's doc is corrected; destruction-16 is the row for its name; destruction-07 is rewritten.
- Mutants run once against the new tests: the seam gated on groups, force holds read as limited, the dealt work ignored, end speed alone, a speed offset on the entering read. Each fails the test that names it.

## Not verified

- Which change moved the end-on house; whether the unlogged halls or the logged ones are the typical run.
- The commitment of comments in the last commit ran the gates, not the passes.
- No hosted run, no two-core machine, no play by a person. No independent reviewer has read the fix pass.
- The clips in `clips-work/` are checkpoint 3's (`a0839b2d`) and do not show this code.

# Work form: end-on and lag measurement

Subject: the game's building model (game state only). Measurement only: no law changed, the
lane's code is `f3193ea8`. Every result is an observation; a crossed or held pass and a larger or
smaller standing share are not findings. Builds, each with the census probe and a two-thread pool:
**c3** = `5c0181ab`; **fix** = `f3193ea8`; **a2r** = `f3193ea8` with a scratch patch (never
committed) under which a way also reads limited through a point held at the impulse bound this
group dealt it, the checkpoint-3 reading. Checkpoint 3 gave a tie to the first group only; the
patch gives it to every equal dealer, as the fix pass does. Passes vary the pass file's
`lateral_m`. All numbers MEASURED from the probe log or `result.json` unless marked DERIVED.

## What the law as built does not explain

One thing, and it is not in the brief's list: **blocks part by the second clause (`crossing` and
`limited`) on steps in which the way's group returned almost nothing.** On the fix build the
group's return over that step, as a share of its impulse bound `F·dt`, is p50 0.8 % (end-on,
417 such blocks, 1068 cells), 0.8 % (long wall, 508 blocks, 1480 cells) and 1.7 % (wall at
2 km/h, 660 blocks, 1783 cells); the work counted on the block when it parts is p50 0.56, 0.52
and 0.45 of the way's work, p10 0.04, 0.01 and 0.06; in 74, 127 and 94 of them the step added
under 1 % of the work the way had left. 5 % to 24 % of these blocks are the hull's, the rest
loose pieces'. `limited` reads true from one member point held at its ration; it does not read
what the group returned. Mechanism not isolated (the bodies' masses are not logged, so whether
the body could have paid the work left is not known). Pointer, not built: read the second
clause from what the way's group returned or the solve rationed over the step, not from one
point's last hold.

Not seen: work on a cell from no contact (every solved point sat in a block with an open way,
and every block that parted with its way's work already reached at the step's start was either
a new block over cells that kept their work, 500, or a block whose way's work had fallen, 130);
a way approaching its work in ever smaller steps; a point returning nothing while its block
neither answered elsewhere nor was being left.

## End-on house, governed speed, five entry lines, three builds

In place = `structure.in_place_share`. Contact = cells parted by the contact law; support =
cells gone less those (DERIVED), parted by lost support. The hull crossed in every run, contact
speed 10.87 m/s.

| line, m | c3 in place | c3 contact / support / gone | a2r in place | a2r contact / support / gone | fix in place | fix contact / support / gone |
|---|---|---|---|---|---|---|
| -0.2 | 0.049 | 603 / 313 / 916 | 0.039 | 665 / 255 / 920 | 0.043 | 599 / 317 / 916 |
| -0.1 | 0.128 | 546 / 318 / 864 | 0.063 | 603 / 317 / 920 | 0.137 | 551 / 317 / 868 |
| 0 | 0.194 | 542 / 312 / 854 | 0.101 | 576 / 338 / 914 | 0.087 | 537 / 377 / 914 |
| +0.1 | 0.147 | 611 / 249 / 860 | 0.159 | 601 / 248 / 849 | 0.129 | 590 / 277 / 867 |
| +0.2 | 0.106 | 473 / 425 / 898 | 0.167 | 435 / 385 / 820 | 0.176 | 443 / 386 / 829 |
| range | 0.049 to 0.194 | | 0.039 to 0.167 | | 0.043 to 0.176 | |

The three ranges overlap. A 0.1 m change of line moves the share by up to 0.09 within one
build, as much as the two builds differ on the pass file's line; the line is repeatable (the
two filmed runs of each build reproduce 0.194 and 0.087 to three figures). Neither A1 nor A2
carries a direction: the path A1 opened carried 0 J in all ten probed runs (no solved point sat
in a block with no open way), and a2r against fix moves single lines both ways (-0.1 m: 0.063
against 0.137; 0: 0.101 against 0.087).

House long wall at 10 km/h, same five lines, hull crossed in 4.31 to 4.48 s in every run:

| line, m | c3 in place | c3 contact / gone | fix in place | fix contact / gone |
|---|---|---|---|---|
| -0.2 | 0.455 | 515 / 596 | 0.570 | 391 / 481 |
| -0.1 | 0.511 | 469 / 642 | 0.534 | 432 / 515 |
| 0 | 0.529 | 428 / 529 | 0.498 | 449 / 526 |
| +0.1 | 0.484 | 454 / 547 | 0.555 | 403 / 484 |
| +0.2 | 0.522 | 417 / 513 | 0.544 | 402 / 485 |
| range | 0.455 to 0.529 | | 0.498 to 0.570 | |

## The 2 km/h wall pass

The 124 of 263 lag cell-steps of the fix pass was one run and did not recur: eleven runs of
the same pass on the fix build give 16, 3, 0, 0, 0, 0, 19, 0, 6, 3, 3 lag cell-steps of 134,
145, 31, 138, 73, 38, 133, 46, 119, 112, 132 overlapped (0 % to 14 %; 50 of 1101, 5 %). That
run's log was deleted with the fix pass, so its steps' state is not known. The hull crossed in
all eleven, in 15.30 to 16.08 s, in place 0.42 to 0.51. Deepest overlap 0.055 m. Longest run of
consecutive lag steps on one cell: 14 steps, 0.22 s; the next longest 6 steps, 0.09 s.

A lag cell-step: a standing cell the hull's solver points overlap by over 2 cm while returning
under 1 N·s between them. Bounds are the last substep's deal times six.

| state | cell-steps | bound dealt | impulse bound, N·s a step, p50 | work dealt, J a step, p50 | group returned over `F·dt`, p50 | least work left on the cell's ways, J, p50 | hull normal speed, start / end, m/s, p50 | open way |
|---|---|---|---|---|---|---|---|---|
| point dealt nothing in a group at its impulse bound | 22 | force | 0.00 | 0.000 | 1.00 | 893 | +0.307 / +0.307 | yes |
| hull leaving the cell | 19 | none binding | 355 | 94 | 0.00 | 263 | -0.047 / -0.049 | yes |
| way's work already reached at the step's start, block parts this step | 8 | work | 262 | 0.000 | 0.00 | 0.0 | +0.342 / +0.367 | yes |
| point dealt nothing, group at 0.55 of its bound | 1 | force | 0.00 | 0.000 | 0.55 | 246 | -0.036 / +0.017 | yes |

The steps are three explained states. In 22 the block is returning its whole impulse bound
through its other points and a point that carries nothing is dealt nothing (`shares`, the
saturated branch): the wall is answering at full strength. In 19 the hull is moving away from
the cell: in the 14-step run a four-cell block with 254 J left of 2769 stopped the hull, which
rolled back about 1.6 cm on the 3 degree upslope and came again at its traction limit; a
contact returns nothing to a body that is leaving it. In 8 the way's work was already reached
when the step began (a new block over cells that kept their work, or a way whose work fell),
so the way dealt nothing and the block parted at that step's end. No cell had no open way.

## The attack's three guesses

| reading | measured | holds |
|---|---|---|
| 1. Lag points starved in a saturated group | 22 of 50 lag cell-steps: point's bound 0, group at 1.00 of `F·dt`. Over all hull point-steps of the eleven runs, 6456 of 38911 end the step dealt a zero impulse bound, 4958 of them in a group at 0.9 of `F·dt` or more | Holds for 22 of 50; the other 28 are a leaving hull or a block's last step |
| 2. Work through points at no bound (A1's path), end-on | fix: 0 J against 821 kJ through bounded points; a2r: 0 J against 764 kJ (DERIVED: impulse times mean normal speed per logged point, five lines each) | Does not hold: the path was never taken |
| 3. Cells parted on the first step a way opens, its work already reached, end-on | fix: 23 of 2720 cells parted by the contact law; a2r: 21 of 2879. With the work already reached on any step: 648 and 642, of which 543 and 512 on a way whose work fell | Does not hold as a path of any size |

Also read for the attack's Z1 and Z2 (fix build, wall at 2 km/h, 124989 block-steps): a block
stands with a way limited and not crossing in 3701 (40 the hull's), 1639 of them with no
closing at the step's start, 207 with another way crossing; the longest such run is 10 steps
there and 15 on the end-on house, where a loose piece resting on a ten-cell block read limited
in 332 steps over 9 s with 3415 to 3418 J left of 3445 and the group returning nothing.

## Clips, saved state, not verified

Two-panel clips of the end-on pass on the pass file's line, c3 left and fix right, filmed runs
reading 0.194 and 0.087 in place: `clips-work/house-end-on-before-vs-after-raised.mp4` and
`clips-work/house-end-on-before-vs-after-exit-side.mp4`. Probe patches, scripts, pass files and
tables: `clips-work/instrument/measure/`.

Not verified: the state of the unreproduced 124-step run; why the second clause's blocks return
so little (masses not logged); checkpoint 3's tie rule under the a2r patch; the 2 km/h pass on
c3 or a2r; any pass at more than five lines or on another machine load.

# Second clause: which mechanism

Subject: the game's building model (game state only). Measurement only; code is `f3193ea8`. The
build measured is `f3193ea8` with a census probe (per substep and member point of every group on
a crossing way: share, dealt work, impulse, clamped, held, lean, and the point's closing speed
from the solver bodies before the substep's forces and after its relax solve; per block: the
pressing body's mass, its effective mass along the push at the mean pressed point, and the
dynamic bodies touching it at the step's start), two-thread pool. Passes: house end-on at five
lines, house long wall at 10 km/h at five lines, wall at 2 km/h five times. It tests the three
derivations of `second-clause-premise.md` section 1. All numbers MEASURED unless marked DERIVED.
`L` is the work the way had left at the step's start. "True work" is the sum over the parting
way's member points and the six substeps of impulse × ½(closing before the substep's forces +
closing after its last solve). "Can pay alone" is ½ × effective mass × start closing² ≥ `L`
(DERIVED from logged values). "Touched" is contact with another dynamic body at the step's start;
whether that body pushed is not measured.

## By the report's reading key

| | end-on | long wall 10 km/h | wall 2 km/h |
|---|---|---|---|
| blocks parted by the second clause (cells) | 417 (1068) | 505 (1480) | 280 (707) |
| could not pay, untouched (1.3 by the key) | 3 | 1 | 0 |
| could pay or touched, held fraction under 0.5 (1.2 by the key) | 306 | 358 | 221 |
| could pay or touched, true work ≥ 0.9 `L`, built count under half of it (1.1 by the key) | 0 | 0 | 0 |
| none of the three | 108 | 146 | 59 |

The three readings as keyed account for 74 %, 71 % and 79 % of the blocks. The rest are not a
fourth mechanism: their held fraction is p50 0.80, 0.78, 0.87 and their true work p50 0.85,
0.84, 0.89 of `L` with the built count equal to it (p50 1.00): the same parting, short of the
work left by a sixth, by bodies that could pay.

The key's first two rows do not separate 1.2 from 1.3, because "touched" is nearly universal.
Split by what the body could pay alone:

| pressing body | end-on | long wall | wall 2 km/h | held fraction, p50 | true work over `L`, p50 | own energy over `L`, p50 |
|---|---|---|---|---|---|---|
| can pay alone (the hull's: 21, 125, 76) | 77 | 163 | 79 | 0.80, 0.72, 0.78 | 0.86, 0.78, 0.77 | 5.3, 138, 33 |
| cannot, touching the hull | 116 | 274 | 169 | 0.042, 0.003, 0.002 | 0.17, 0.054, 0.032 | 0.009, 0.006, 0.005 |
| cannot, touching loose pieces only | 221 | 67 | 32 | 0.006, 0.000003, 0.008 | 0.078, 0.0001, 0.097 | 0.005, 0.003, 0.002 |
| cannot, untouched | 3 | 1 | 0 | | | |

So 82 %, 68 % and 72 % of the blocks are parted by a loose piece of p50 30 kg whose own energy
is under 1 % of the work left, at a held fraction under 5 %, having done p50 0.01 % to 17 % of the
work left across the rows above. Those pieces did 15 to 20 times their own energy, so something drove them; the log does
not say what did beyond the touch.

The tail the report asks about: of the blocks whose built count is under a tenth of `L` (138,
186, 102), every held member's share is under 0.05 in 131, 177 and 100, and a held member's lean
is under 0.5 in 114, 150 and 92. Held members hold p50 4 %, 8 % and 27 % of their dealt impulse.
That is 1.3 as derived: a point at a collapsed share, cut by the work term, reads the group
limited.

1.1 is seen and carries nothing: true work over the sum of impulse × end speed is p50 1.09,
1.07, 1.14 (p90 2.7, 2.7, 2.2), so the ration under-reads the work as derived, by a tenth at the
median; no block has true work near `L` and a count far under it. The built count over the
group's true work is p50 1.10, 1.07, 1.26; the count covers every point on the block's cells
and the true work only the parting way's members, so the two are not the same sum.

No block in these tables parted by the belt path: they are the solver-point loop's. The belt
loop parted 7 blocks in the fifteen runs, 6 by the first clause and 1 by its second.

## Replay of the hull's crossing steps (trajectory fixed, nothing built)

Hull block-steps with a crossing way, and how many would part under each reading of the second
clause:

| reading | end-on | long wall | wall 2 km/h |
|---|---|---|---|
| hull block-steps with a crossing way | 128 | 269 | 472 |
| as built: a crossing way limited | 83 | 186 | 112 |
| first clause alone on a crossing way | 62 | 61 | 38 |
| (b) equality: every substep's whole deal on held points | 36 | 26 | 15 |
| (b) equality within 1 % | 37 | 27 | 16 |
| (b) impulse: returned × closing ≥ `L` | 45 | 42 | 45 |
| true work over the step ≥ `L` | 46 | 39 | 36 |

## Code check

Confirmed. The fork's substep order is integrate velocities, warm start, biased solve, integrate
positions, relax (`SubstepSolverSystems`, chained in `schedule.rs`); `warm_start` applies the
whole accumulated impulse to the bodies; `solve` computes `relative_velocity` from the bodies as
they then are and `solve_impulse` takes `normal_speed` from it for `max_work / -normal_speed`.
The ration bounds impulse times the speed left, after the warm start and every earlier row.

## Saved, and not verified

Probe patch, scripts and tables:
`.agents/scratch/rubble-explore/clips-work/instrument/second-clause/` in the main checkout.

Not verified: whether a touching body pushed (no impulse between dynamic bodies is logged), so
"touched" over-reads "backed"; what drove the pieces that did many times their own energy;
whether any reading of the replay would cross or stall, since a replay keeps the trajectory; the
0.5 and 0.9 lines of the key are this pass's choices; the probe's closing speeds are the solver
bodies' at two points of the substep and miss what happens between.

# One budget: checkpoint 1

Subject: the game's building model (game state only). Build `b6b776d5` on `lane/work-form`: one
work budget per body and block, spent inside the solve by true work. The law is stated once in
`.agents/scratch/strength-limited-contact-design-2026-11-21.md`, "As built"; this section holds
the fixtures' readings, the fork's size and the seam's cost. No building pass was run.

## One departure from the premise's clamp

The premise (section 2(c)) cuts a point whenever some impulse under the solve's answer does the
work left (the lesser root of `½ p (a − pK) = e`). Built that way first as a mutant, it failed
two fixtures (MEASURED): a 2 kg box a 2,000 kg box holds still on the slab had its 100 J budget
spent at rest on step 19; and the pair moving at 4 m/s lost 1,055 J against a budget of 1,000 J
with an allowance of 48 J over what its points did. Cause: `K` is the point's own response. A
pressed light body holds a large impulse, and at a lesser impulse the formula has it fly into
the slab alone, though the body behind it would follow at once. The build judges the work at
the impulse the solve found: `½ p (c0 + u − pK)` at that `p`. If it is more than the budget has
left, the impulse is cut to the lesser root; if not, it stands. The law's statement holds as
briefed (true work bounded by the budget; a body that cannot pay stops and the way stands), and
the "cannot pay" branch needs no test of its own: the stopping impulse of a free body does
exactly its kinetic energy.

## Decisions

- **Way per push** for solver points: the way that holds least (`price.force`), where the old
  form took the least `reaction` at the start speed. The budget is the least work left among
  the block's ways, and the block parts by its least-work way.
- **Belts keep their whole-block form**, unchanged in arithmetic: a staged force has no solver
  point, so nothing is solved for it and the limiter cannot reach it. `Standing::give_to_belt`
  holds `reaction` and `crosses` at the belt's start speed; its selection (least `reaction`,
  first of equals) is the old one.
- **`BlockRow::bound` / `bound_ns` deleted** (it was the start-speed prediction); `spent` added:
  whether a body did the last of the block's work inside the step. `scripts/perf/collapse.md`
  follows.
- **Hook shape.** One call after each normal solve of a point that names a limiter, before the
  increment moves the bodies: `fn(&mut ContactNormalPart, &mut SolverBody, &mut SolverBody,
  closing, increment) -> increment`. The fork knows a token, three floats on the part and a
  `Vec<f32>` on the solver body; work, budgets and the cut are in
  `src/avian_ext/reaction_bound.rs`.
- **Parallel safety** checked in `constraint_graph.rs`: a colour's `body_set` holds every
  non-static body of its constraints, a constraint takes a colour only when neither of its
  non-static bodies is in it, and the overflow colour is solved on one thread. A budget's cell
  is on its one dynamic body, so it has one writer at a time.

## Fixtures

Seam level, `src/tests/avian_reaction_bound.rs` (a static slab, no friction, isotropic
inertia, energy as `½ m v² + ½ I ω²`); building level, `src/construction/breakage/bounded/tests.rs`
(the brick leaf). All MEASURED on the committed build unless marked.

| Fixture | Reading | Allowance (DERIVED) |
|---|---|---|
| 2,000 kg at 4 m/s (16,000 J), budget 400 J, 1 point | lost 400.003 J, counted 400, spent step 1 | f32: 0.023 J |
| same, 4 points | lost 399.9998 J, counted 400, spent step 1 | 0.09 J |
| same, 16 points | lost 399.9998 J, counted 400, spent step 1 | 0.37 J |
| 2 kg at 1 m/s (1 J), budget 400 J, 1 point | stopped, budget stands, counted 1 J | f32 |
| same, 4 points | stopped, stands, counted 0.999986 J; keeps 0.000014 J backing out of the fixture's 1 mm overlap | f32 |
| held at 100 N·s a step, budget 1,000 J, 1 / 4 / 16 points | lost 1,000.01 / 999.99 / 1,000.21 J, spent step 3 | one substep at the bound: 66.7 J |
| 2 kg under 2,000 kg, both at 4 m/s, same bound and budget | light box's points did 1,015 J; the pair lost 1,036 J; spent step 3 | points: 66.7 J; pair: up to 48 J more, rejoining |
| 2,000 kg at rest under gravity, budget 100 J, 64 steps after settling | 0.00006 J, budget stands | 0.046 J |
| 2 kg held still under 2,000 kg, same | 0 J, budget stands | 0.046 J |
| leaf: one 57 t box at 4.5 m/s on a block of 3,110.94 J | block took 3,111.24 J | one substep at strength: 1,692 J |
| leaf: two such boxes on that block in one step | block took 6,222.48 J (2.0002 times) | up to twice, plus the same |

The loss does not move with point count beyond the residual: 400.003 / 399.9998 / 399.9998 J,
and 1,000.01 / 999.99 / 1,000.21 J. A resting body adds no work past f32. The settle before the
rest reading did 1.9 J (one box) and 40 J (the stack), bodies sinking from the fixture's
overlap onto the contact: real travel under weight, and under the budget.

Mutants run (MEASURED): no cut, the budget read between substeps alone (16,000 J lost on the
free fixture; 1,178 J on the held one; 1,122 J done by the pressed box's points); the count at
the end speed alone (397.5 of 400 J; 0 J for a stopped body); the limiter without the speed
before the substep's forces (held-still budget spent on step 4); the premise's lesser-root cut
(above). The release ignoring `Bounds::spent` survived every building fixture until the
`spent` row was read by `bonds_take_the_work_done_on_them_once_and_part_only_when_a_way_is_spent`:
a block parted on its budget with the count at 6,213.12 of 6,213.36 J.

Two older building fixtures moved. `a_cell_does_not_stand_in_a_body_past_its_give` sent 57 t at
0.5 m/s (7,125 J) and asserted the cells in its path leave; under the budget the box frees two
cells, does 4,350 J of 8,218 on the next block and stops, out of energy with no drive
(MEASURED trace). The fixture is now 1.5 m/s (64 kJ) and keeps its depth assertion. The
"crossed" reading of the old second clause is gone from the work fixture.

Named residuals, each in the design doc: solve lag (within one substep's work at the block's
strength), several bodies (up to bodies × work left), the count's rounding against the signed
budget, restitution (in the impulse, not in the work; standing matter's pairs have none).

## Fork

`OVERMATCH_PATCH.md`, "Bounded contact": **hook**, a divergence with its seam at
`avian_ext::reaction_bound`; no upstream draft. Fork lines by `diff --minimal -ruN` of `src/`
against the published crate (MEASURED): 1,008 before, 1,032 now. The bounded-contact patch is
53 lines: 22 in the normal part (was 24: `max_work` and its clamp gone, `limit` and
`limit_state` in), 27 in the contact constraint (5 restitution as before, 22 the limiter's type,
static and call), 4 in the solver body (`limit_scratch`). The hook is generic: the fork states
nothing about work.

## Cost

One run: a 57 t hull through the hall long wall at 20 km/h, two-thread pool, 699 ticks from
first contact, release build with timers around the seam's systems and a counter in the
limiter. Per tick, ms:

| Part | mean | p95 | max |
|---|---|---|---|
| `deal` (six substeps) | 0.202 | 0.231 | 0.882 |
| `account` (seven calls) | 0.055 | 0.069 | 0.255 |
| `locate` (with the budgets' cells) | 0.018 | 0.030 | 0.315 |
| `read_bounds` | 0.002 | 0.003 | 0.017 |
| the four systems | 0.277 | 0.316 | |
| limiter, in the solve (DERIVED) | 0.055 to 0.112 | 0.062 to 0.127 | 0.137 to 0.278 |

The limiter ran 7,513 times a tick at the mean (p95 8,472, max 18,600; MEASURED); a call costs
7.4 ns uncut and 15.0 ns cut (MEASURED, 4M calls each in the same binary at start-up, warm
cache, so a floor for the solve's own calls); the row is the count times each. Against `deal`
alone at 0.22 ms before: `deal` 0.20, and the budget adds 0.07 ms of systems and about 0.06 to
0.11 ms in the solve. The run crossed in 8.0 s with no stall (an observation from the one
probe run, not a pass).

## Not verified

- No pass of the checkpoint-2 set, no clip; the one hall run above was for cost.
- The cost is one run; the hosted price is not measured.
- The limiter's cost inside the solve is derived from a count and a start-up loop, not timed in
  place.
- Belts: no belt fixture was added; the two existing belt tests pass unchanged.
- Backlog destruction-11 (pose advance against velocity) was measured under the old law and is
  not re-measured.
- `net::protocol` passes; `WIRE_TYPE_DEFS` is untouched.


# One budget: checkpoint 2

Subject: the game's building model (game state only). The pass set on the build of `b6b776d5`
(the law) with a temporary probe, never committed (kept at
`.agents/scratch/rubble-explore/clips-work/instrument/budget/`). Every number is MEASURED unless
marked. Release build, ordinary input path (`scripts/perf/collapse.sh --ram`), one tank, two
compute threads. The machine was shared throughout (load average 5 to 15, other sessions'
processes), so tick times are upper readings and are not comparable with earlier checkpoints.

## Not explained by the law as built

One thing. A piece's budget is marked spent by a cut whose substep then does far less than the
budget, and the block parts under its work by more than the solve-lag allowance.
- Read at the cutting substep (8 passes rerun with the reading): of 123 partings by a piece's
  spent budget, 4 had the budget left before that substep, less the work the substep did, over
  one substep's work at the way's strength at the substep's fastest closing (`F dt/6 v`): by
  1.15, 1.28, 2.4 and 7.1 times. Largest: hall long wall 20 km/h, a way of 9,557 J and
  3.85 MN, 8,761 J left, the substep did 437 J at 0.35 m/s, the block parted at about 12 % of
  its work. In two rows the substep's work was negative (−1.6 J and −4.9 J).
- The bodies have many points on the block (8, 16, 56, 83). DERIVED, not verified: each
  point's cut assumes its own response only, the body's other points on the same block slow it
  further, and the sum of their predictions passes the budget when the true work does not.
- The hull does not show it: 33 partings by the hull's spent budget, the same reading at most
  0.0015.
- In the 23-pass set, with the allowance read at the step's start and end speeds, 5 of 374
  spent partings were outside it, all pieces. 7 of 3,031 piece partings were a piece with less
  energy than the budget, not driven from behind, parting a block by its own spent budget; the
  budgets were 0.2 to 52 J (blocks already near their work).

Everything else below is explained by the law or by a named residual.

## Passes

Columns: result; speed at contact m/s; least forward speed m/s; worst one-tick speed change m/s;
deepest hull-to-standing overlap m; cells that left standing (by contact; by lost support);
share in place; cells of blocks only pieces parted (share of contact-law cells); bodies at end;
tick mean / p95 ms / ticks over 15.6 ms, with the probe's logging.

| pass | result | contact | least fwd | worst dv | overlap | left (contact; support) | in place | piece-parted | bodies | tick |
|---|---|---|---|---|---|---|---|---|---|---|
| wall 2 km/h | crossed 15.16 s | 0.45 | -0.03 | +0.09 | 0.046 | 191 (190; 1) | 0.53 | 110 (58 %) | 190 | 5.10 / 9.98 / 22 of 1921 |
| wall, standing start | crossed 3.86 s | 0.36 | 0.38 | -0.08 | 0.027 | 215 (214; 1) | 0.49 | 132 (62 %) | 214 | 3.90 / 9.56 / 29 of 1921 |
| wall 10 km/h a | crossed 3.31 s | 2.65 | 1.39 | -0.08 | 0.027 | 205 (204; 1) | 0.49 | 108 (53 %) | 205 | 4.54 / 9.54 / 5 of 641 |
| wall 10 km/h b | crossed 3.27 s | 2.65 | 1.58 | -0.09 | 0.017 | 202 (200; 2) | 0.51 | 96 (48 %) | 201 | 4.45 / 9.65 / 2 of 641 |
| wall 10 km/h c | crossed 3.23 s | 2.65 | 1.50 | -0.08 | 0.024 | 218 (217; 1) | 0.50 | 120 (56 %) | 214 | 4.85 / 11.75 / 9 of 641 |
| wall 10 km/h d | crossed 3.23 s | 2.65 | 1.53 | -0.07 | 0.036 | 202 (190; 12) | 0.50 | 103 (54 %) | 202 | 4.30 / 9.52 / 5 of 641 |
| wall 10 km/h e | crossed 3.23 s | 2.65 | 1.61 | -0.09 | 0.015 | 207 (206; 1) | 0.52 | 111 (54 %) | 206 | 4.75 / 10.00 / 2 of 641 |
| wall 10 km/h f | crossed 3.20 s | 2.65 | 1.61 | -0.10 | 0.032 | 197 (197; 0) | 0.53 | 101 (51 %) | 196 | 4.11 / 9.01 / 2 of 641 |
| wall 10 km/h g | crossed 3.27 s | 2.65 | 1.48 | -0.08 | 0.061 | 226 (223; 3) | 0.45 | 135 (61 %) | 225 | 3.86 / 8.11 / 2 of 641 |
| wall 10 km/h h | crossed 3.33 s | 2.65 | 1.34 | -0.10 | 0.063 | 205 (196; 9) | 0.50 | 113 (58 %) | 205 | 4.65 / 10.56 / 5 of 641 |
| wall 20 km/h | crossed 1.73 s | 5.39 | 3.98 | -0.16 | 0.014 | 205 (205; 0) | 0.50 | 134 (65 %) | 205 | 3.56 / 8.88 / 7 of 641 |
| house long wall 10 | crossed 4.70 s | 2.67 | 1.45 | -0.25 | 0.045 | 414 (341; 73) | 0.66 | 238 (70 %) | 327 | 7.08 / 13.82 / 7 of 641 |
| house long wall 20 | crossed 2.44 s | 5.46 | 4.35 | -0.15 | 0.093 | 403 (327; 76) | 0.62 | 243 (74 %) | 348 | 8.17 / 14.22 / 28 of 641 |
| hall long wall 10 | held (run invalid) | 2.67 | -0.16 | -0.53 | 0.058 | 414 (209; 205) | 0.69 | 190 (91 %) | 337 | 38.76 / 81.11 / 633 of 641 |
| hall long wall 20 | crossed 7.16 s | 5.43 | 0.85 | -0.90 | 0.000 | 317 (107; 210) | 0.74 | 80 (75 %) | 219 | 12.92 / 30.67 / 125 of 641 |
| hall junction 20 | held | 5.42 | -0.26 | -0.53 | 0.000 | 415 (273; 142) | 0.62 | 260 (95 %) | 376 | 11.10 / 13.12 / 17 of 641 |
| end-on, line -2 | crossed 2.77 s | 10.87 | 4.07 | -1.20 | 0.000 | 875 (728; 147) | 0.10 | 699 (96 %) | 856 | 14.79 / 19.21 / 232 of 641 |
| end-on, line -1 | crossed 2.08 s | 10.87 | 5.73 | -1.33 | 0.000 | 791 (648; 143) | 0.19 | 619 (96 %) | 765 | 9.25 / 15.77 / 39 of 641 |
| end-on, line 0 | crossed 1.92 s | 10.87 | 6.32 | -0.32 | 0.000 | 882 (546; 336) | 0.10 | 511 (94 %) | 845 | 13.77 / 19.49 / 167 of 641 |
| end-on, line +1 | crossed 1.98 s | 10.87 | 6.51 | -0.47 | 0.000 | 919 (478; 441) | 0.05 | 441 (92 %) | 886 | 14.42 / 21.03 / 195 of 641 |
| end-on, line +2 | crossed 2.03 s | 10.87 | 5.32 | -0.47 | 0.000 | 782 (551; 231) | 0.20 | 519 (94 %) | 751 | 17.14 / 30.18 / 293 of 641 |

Mechanisms.
- **Walls**: all eleven cross. The hull parts 21 to 43 blocks a pass by its own work (its own
  work over its budget 1.000 at the median); pieces part 73 to 95.
- **House long wall**: crosses at both speeds; a fifth of the cells leave by lost support.
- **Hall long wall 10 km/h**: the logged run is invalid (the server ran at 37.6 Hz under the
  machine's load, mean tick 38.8 ms) and held; two unlogged runs of the same binary crossed in
  9.89 s and 9.75 s. In the logged run the hull was slowed with the belts at 59 kN of a
  122 kN capacity and standing cells returning under 10 kN along travel: pieces ahead and
  under the belts, not standing masonry.
- **Hall junction 20 km/h**: held, in the logged run and the unlogged one (stall 0.80 to
  0.91 s). The hull parts 5 blocks and is on 5 overlapped cell-steps in the whole pass; in the
  last 2 s the belts carry no load (capacity 0 kN) and standing cells return nothing along
  travel: the hull rides on pieces. Lost traction and mass ahead.
- **End-on**: all five lines cross with 5 to 20 % in place; pieces part 92 to 96 % of the
  contact-law cells.
- **Parked hulls**: both sleep. Touching the wall: the hull parts 4 blocks arriving, then no
  point on a standing cell and no work for 60 s. Stopped in the breach: 16 J over the 5 s it
  takes to stop, then nothing for 60 s; no parting after the hull stops. No work at rest.

## True count over W at every parting (23 passes, 3,558 partings)

| who, cause | partings | counted / W p10, p50, p90 | under W | outside the lag allowance | over W |
|---|---|---|---|---|---|
| hull, work reached | 407 | 1.000, 1.00 to 1.03, up to 1.8 | 0 | 0 | 245, all by other bodies' work on the same cells |
| hull, budget spent | 120 | 1.000, 1.000, 1.000 (least 0.999) | 94 | 0; shortfall over allowance at most 0.007 | 0 |
| piece, work reached | 2,777 | 1.00, 1.1 to 1.9, 3 to 9 | 0 | 0 | 2,724; 3 not covered by the same step's other bodies (work counted on the cells in earlier steps) |
| piece, budget spent | 254 | 0.7 to 0.99, 0.89 to 0.99, 0.99 to 1.00 (least 0.14) | 254 | 5 (above) | 0 |

The hull's partings sit on W: a real pass uses under 1 % of the solve-lag allowance. Pieces'
spent-budget partings use a median 0.3 to 5 % of it and reach it on fast pieces at small
blocks (least counted over W 0.14, inside the allowance). The count over W by pieces is the
several-bodies residual: up to tens of bodies press one block and each may lose the work left.

## Overlapped cell-steps returning nothing

817 hull cell-steps overlapped over 2 cm across the set; 66 returned under 1 N·s: 48 on a
block that parts that step, 10 on a way whose points are at its strength between them (the
deal gives the strength to its other points), 8 not closing at the step's start or end. None
closing with budget left and its way under its strength.

## The advisor's four readings

| reading | holds? | size |
|---|---|---|
| 1. floored work under a rocking body | Not for the hull. Yes for pieces. | Hull: signed over counted work per cell p10 1.000, least 0.81, on every pass, holds included. Pieces: at 348 of 3,558 partings (9.8 %) the signed work on the block's cells was under half the counted; at 272 (7.6 %) it was under half of W. Example: W 533 J, counted 549 J, signed −453 J. |
| 2. starved points seeded by the budget | No. | 0 of the 66 idle overlapped cell-steps had a point cut by the budget in an earlier substep with the budget standing at the step's end. The deal's own starvation is the 10 above (1.2 % of overlapped cell-steps). |
| 3. way parted against way pushed | Yes. | Hull: 71 of 527 partings (13 %) along a way whose group returned nothing on any step of the block, 89 (17 %) nothing that step; the parted way's W under 0.9 of the most loaded way's in 136 (26 %), p10 0.72 to 0.85. Pieces: 917 of 3,031 (30 %) never, 1,167 (39 %) not that step, cheaper in 811 (27 %); a piece's row also reads nothing when other bodies did the work. |
| 4. belt partings | Not observed. | 4 belt partings in 23 passes, all by the work clause, none by the start-speed crossing clause; counted over W 1.009, 1.042, 1.185, 2.556. |

## Cost

Seam stages in place, ms a tick, mean / p95, from first contact (probe timers, logging on):

| pass | pricing | deal | account | locate | limiter calls a tick | limiter, timed in place |
|---|---|---|---|---|---|---|
| wall 10 km/h (8 runs) | 0.23 to 0.33 / 0.5 to 2.2 | 0.010 / 0.05 | 0.005 / 0.02 | 0.005 / 0.015 | 420 to 630 / 2,300 to 2,800 | 0.003 / 0.013 |
| house long wall 10 | 0.90 / 2.69 | 0.128 / 0.565 | 0.020 / 0.065 | 0.011 / 0.023 | 2,202 / 6,624 | 0.012 / 0.037 |
| hall long wall 20 | 2.29 / 5.62 | 0.258 / 0.576 | 0.075 / 0.184 | 0.023 / 0.057 | 6,608 / 7,992 | 0.023 / 0.002 |
| hall junction 20 | 1.55 / 1.90 | 0.145 / 0.180 | 0.047 / 0.056 | 0.015 / 0.020 | 5,913 / 7,164 | 0.025 / 0.042 |
| end-on, line 0 | 0.52 / 2.52 | 0.040 / 0.154 | 0.019 / 0.055 | 0.015 / 0.033 | 1,617 / 5,652 | 0.014 / 0.028 |

The limiter is timed on one call in 32 with a clock pair, less the pair's own cost (15 to
25 ns, calibrated at start): 4 to 9 ns a call, 0.003 to 0.03 ms a tick.

Tick time without logging (same binary, one run each, machine shared): wall 10 km/h mean
4.4 ms, p95 9.3, 6 ticks over 15.6 of 641; house 10 mean 10.5, p95 18.3, 81 over; hall wall 10
mean 13.5 and 8.7 (two runs), p95 20.9 and 10.4, 77 and 13 over; hall wall 20 mean 12.4, p95
18.7, 91 over; junction mean 13.1, p95 19.3, 92 over; end-on mean 13.1, p95 17.4, 98 over. The
two hall runs differ by 1.6 times on one binary, which is the machine's load.

## Clips

Left `f3193ea8`, right the new build, labels burned in:
`.agents/scratch/rubble-explore/clips-budget/wall-10kmh.mp4`,
`house-long-wall-10kmh-exit.mp4`, `house-end-on-raised.mp4`,
`hall-junction-20kmh-raised.mp4`, each with three stills.

## Not verified

- The mechanism of the spent-budget finding (many points' predictions summed) is DERIVED.
- Tick times against `f3193ea8`: the machine was shared, and the earlier build's binary has no
  thread cap without its own probe.
- One run a pass except the wall at 10 km/h; the halls vary run to run.
- Friction work on standing cells (the attack's 1c) was not read.
