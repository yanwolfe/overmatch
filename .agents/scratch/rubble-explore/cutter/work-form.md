# Work form: checkpoint 1

Subject: the game's building model (game state only). Lane `lane/work-form`, law commit `e9751c45` on `4edd1852`.

The stall is gone at the plain wall: 9 of 9 passes at 10 km/h cross, and 2 km/h and the standing start cross too (MEASURED).
Over-freeing is real: with the work form 29 to 96 % of the cells the contact law frees leave on ticks when only loose pieces, not the hull, parted a block (today 8 to 81 %), and every building keeps less in place (MEASURED).

## The law as built

Every step, each contact point on a standing cell adds to that cell the normal impulse the solver returned there times the speed its body advances into the cell there as the solve left it (negatives add nothing). Impulse (N s) times speed (m/s) is the step's work (J): the step length is inside the impulse. Nothing removes it. A block is charged with all the work on its pressed cells; when that reaches the work `W` of a way the block gives, that way's `leaves` part.

## Design points settled

- **Where the work lives.** `Live::work`, one `f64` per law chunk, on the owner's `Breakage` (`src/construction/breakage.rs`). Zero at spawn, built from the structure, never from an asset.
- **Written** only in `release` (`src/construction/breakage/bounded.rs`), after the physics step, by `settle`, once per owner. Owners settle in parallel (`per_owner`, compute pool): each writes only its own `Live::work` and reads the solver's impulses (`Bounds`) and the bodies' post-step velocities, both immutable there. Cuts and departures then commit serially in owner order.
- **Read** in `bound_contacts` and `bound_belts` before the step (`Standing::work`, `give_along` takes `taken(work, pressed)`), to size the bound's work left, and in `settle` to decide parting.
- **Ordering relied on.** `bound_belts` and `bound_contacts` before the solve; `read_bounds` after the last substep; `release` after that and before the support check. Within an owner the sum runs belts first, then contacts in body-entity then narrow-phase order, so it does not depend on worker assignment (DERIVED from the code; no test).
- **Impulse source.** `Bounds` now keeps the returned impulse per contact point (`Bounds::read`, `Bounds::impulse`), summed over substeps. `ContactTransfers` was not used: it has no contact-id key. The two agree at warm-start coefficient 1 (DERIVED, not tested).
- **A give's sum** is over the block's pressed cells, whatever body or direction did the work. Of a block's ways the least `W` is reached first (work only rises), so that one parts. Parting frees the give's `leaves`.
- **No double count.** Work is added once per point to its cell; gives only read it. `Bounds::spent`, the group's `room` and the per-group `returned` state are gone.
- **Belts: same form.** A staged belt force on standing matter is capped at the cells' bound as before; the kept impulse times the belt body's advance into the cell is added to the cell; the cells leave when their work reaches `W`. The belt no longer frees a cell merely for pushing harder than `F`.
- **Rollback and wire.** The work is server state only. It is not in a replicated type and not in a rollback-registered component, so `net::protocol` is unchanged and `PROTOCOL_REV` stays 108 (DERIVED from the diff: no replicated type edited; the ADR texts were not re-read in full).
- **Cells that leave take their work.** `Live::group` clones the vector into the piece; nothing reads it there. Remaining cells keep theirs.
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

- work per cell never decreases, a cell leaves if and only if a row's taken work reached its way's, and the work counted is at most the kinetic energy the body lost (a double count would break that bound; there is no separate duplicated-give fixture);
- a body placed inside and pushed out adds exactly none; a held body adds 1.5e-3 J in 6 s (asserted small, not zero);
- a belt keeps what standing matter returns, and the cells it advances into leave when their work reaches their price;
- the read impulse equals the momentum the body lost.

Deleted with the old counting: the `crossed` assertion, the spent-report assertions, the tests' `spent` arguments. Worker-assignment independence has no test.

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
- Summation order independent of worker assignment, and `Bounds` agreeing with `ContactTransfers`: derived, untested.
- No hosted run, no play by a person, no two-core machine: the pool cap is not a two-core allocation.
- The six lenses were not run by this lane.
