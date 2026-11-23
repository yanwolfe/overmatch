# Hull on loose pieces: plan (2026-11-23, read-only planner; claims DERIVED unless marked)

**Decision: build no new contact law yet.** The two-way law the owner asked for is already in the tree, and the "no traction" reading comes from a probe that only reads the other channel. I recommend candidate A below, and the first step is one measurement.

All paths are under `/Users/Yan/github/yanwolfe/overmatch/.claude/worktrees/bay-1` unless given in full.

## 1. Mechanism (DERIVED from code unless marked)

- **What the sampled law reads.** `RubbleQuery::supports` (`src/track/rubble.rs`) rejects any collider whose layers interact with `soft_run::run_layers()`, which is every `Layer::Debris` collider. So the sampled support sees the terrain field, standing cells and other hulls, and never a loose piece. This is ADR-0041's "one channel per solid", not an accident.
- **What holds the hull up.** Per side there are three solver colliders, all carrying `BeltFace`, plus the hull's own colliders (the belly):
  - the soft run (`src/track/soft_run.rs`, `TrackBelt` against `Debris`, compliant at `k'` per link, datum at the rest outer face);
  - the bump stop (`src/track/bump_stop.rs`, `Layer::Vehicle` against everything, rigid, at full bump travel);
  - the side plates, which never meet a piece.
- **MEASURED** (`work-form.md`, "The junction"): 8101 N·s per step on belt-face colliders and 721 on the belly, of about 8740. The earlier section "What carries the hull at the junction hold" reads 6247 and 2458 of 8737.
- **The drive path exists and is two-way.** `prepare_belt_surfaces` in `src/track/stop_contacts.rs` binds every belt-face contact to its side's `SurfaceCoordinate`. `src/avian_ext/surface_motion.rs` solves Coulomb friction against the belt's finite drivetrain mobility (`src/track/surface_motion.rs`), inside Avian's contact rows. The piece takes the equal and opposite impulse, and the transmission takes the belt's. `by_owner` and `reactions` return friction and `mu × N` to the Driver as `HardReaction`.
- **Why it read "0 N".** The probe's `H` row logs `effect.tangential_force` and `effect.traction_capacity_n`. `src/track/sim.rs` (about line 1705) sums both over sampled contacts only. The solver channel's friction was never logged; the `K` row sums only `impulse.y`.
- **The probe cannot split run from stop.** `part()` in `/Users/Yan/github/yanwolfe/overmatch/.agents/scratch/rubble-explore/clips-work/instrument/fix-pass/work_log-fix-pass.rs` tests `Has<BeltFace>` first, so run, stop and plate all print `F`, and the `B` and `L` arms are unreachable.
- **So "loses all drive" is not shown.** What is shown is that the sampled channel is empty. The hull still creeps at 1.07 m/s under full demand (MEASURED), which suggests some drive is arriving.
- **A real account defect, whatever the measurement says.** `TrackGripEffect::traction_capacity_n` omits the solver channel, and `src/tank/battle.rs` (about line 660) feeds it to `braking_deceleration`. A tank on pieces therefore plans its stop with zero grip.
- **Standing cells are a separate path.** `bound_belts` (`src/construction/breakage/bounded.rs`) with `Standing::give_to_belt` (`src/construction/fracture/bound.rs`) prices sampled forces on fixed structure and plays no part in the hold.

## 2. Candidate laws

**A. The existing law, fully accounted (two-way; recommended).**
- **Law:** a piece under the belt supports through the run's per-link spring, then the rigid stop. The belt drives it by Coulomb friction at belt speed against drivetrain inertia, and the piece feels the opposite impulse. The belly is a rigid plate with static-surface friction.
- **Change:** fold `HardReaction` friction and grip into `TrackGripEffect`, so capacity, forward force, braking and probes read both channels.
- **Cost:** a few adds per tank per tick (DERIVED).
- **Soil and standing cells:** unchanged; no belt-face contact exists there except at the stop.
- **Bans:** no new rule, number or limit.
- **Refuted if:** the belt faces carry normal load `N`, the belt slips against the pieces, and delivered friction is well below `mu × N` with the piece not accelerating to match.

**B. One sampled channel that also reads pieces (two-way by explicit impulse).**
- **Law:** `supports` admits `Debris`. Pieces enter the convex slab check, and `rubble::reactions` applies the opposite impulse, as it does for hulls.
- **Cost:** the island holding the stalled hull has 339 bodies (MEASURED, `brick-account-checkpoint-3.md`). Each candidate costs six support-map queries per surviving slab across 48 stations (DERIVED), on a pass already at 10.8 to 13 ms.
- **Bans:** the explicit spring is stable only while `dt²k/m + 2dt·c/m < 4` (stated in `rubble.rs`). Light pieces break that, and a mass floor or a damping number is banned.
- **Prior evidence:** ADR-0041 records this imitation as measuring "both worse and larger".
- **Refuted if:** a brick-mass piece under one link goes unstable.
- **Verdict:** reject.

**C. The piece's lower face takes the ground's own grip and soil law (two-way, through the stack).**
- **Law:** belt friction on a piece is returned only by what holds the piece. If pieces shear out along the soil, the piece-to-soil contact should carry the field's grip, which the hook currently sets only for a belt face on a `FieldSurface`.
- **Cost:** a hook call per piece-to-terrain manifold, hundreds per step (DERIVED).
- **Soil and standing cells:** belt driving unchanged; only piece-on-soil contact changes.
- **Bans:** generic, not specific to one kind of piece.
- **Refuted if:** the pieces under the belts are not moving rearward.
- **Verdict:** only if the measurement points there.

## 3. Recommendation and first measurement

Take A. Before building anything, extend the probe; do not change the law.

Per step, per side, log:
1. Vertical impulse split by `Has<SoftRun>`, `Has<BumpStop>` and the belly. Test `SoftRun` before `BeltFace`.
2. Friction impulse along hull-forward on each of the three, and `mu × N` for each.
3. Belt surface speed, hull speed, and the mass-weighted velocity of the pieces touching the run.
4. The run's contact count and deepest overlap against bump travel.
5. Forward impulse on the hull from pieces on non-belt colliders (drag and mass ahead; 31 kN at checkpoint 2, MEASURED).

Decision table (belly force = belly drag plus mass ahead):

| Reading | Meaning | Next |
|---|---|---|
| Friction ≈ `mu × N`, belly force ≥ friction | High-centred; the law explains the hold | Ship A's account fix; close the row as explained |
| Friction ≈ `mu × N`, pieces streaming rearward | Heap shears under the belt | Believable, or C if the piece-to-soil grip is wrong |
| Friction ≪ `mu × N` while slipping | The existing law is not answering | Trace the compliant row's friction; stop |

Builder checkpoints, each with the behaviour the law cannot explain:
1. **Probe on the junction and on `park_on_pieces`.** Stop if hull forward momentum change differs from the sum of logged forward impulses by more than the gravity-along-slope term.
2. **Account fold.** Stop if `traction_capacity_n` on plain soil changes at all, or a tank on pieces reports capacity above `mu × weight`.
3. **Fixture: one belt on one free piece on a frictionless floor.** Stop if piece momentum gained differs from belt momentum lost, or the piece ends faster than the belt.
4. **Junction rerun.** Stop if a belt carries normal load, slips, and delivers friction below its Coulomb limit with no moving piece absorbing it; or if the hull rises while the run's overlap is under bump travel and the stop is loaded.

## 4. Not determined from reading

- Whether the compliant (run) row's friction limit uses the compliant normal impulse or an inert point's. I did not open `compliance.rs` or the vendored row.
- Whether the weight sits on the run or the stop at 0.49 m.
- Piece-to-terrain friction values and their combine rule.
- Whether the Driver's gear choice behaves once the sampled grip is zero and the solver grip arrives a tick late.
- The run's cost per tick at the junction.
- The cause of driving-21's 0.22 m jitter, which may share a cause with the hold.
