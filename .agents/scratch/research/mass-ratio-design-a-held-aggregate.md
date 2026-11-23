<!-- Design A, paper only, 2026-11-23; claims DERIVED unless marked -->

# Aggregate design: "a held piece is one body with what holds it"

Decision: I reshaped the direction in three ways. The pusher (hull) joins the aggregate, because otherwise case (a) has one piece and nothing to aggregate. Static bodies never join, so their rows stay and their bounds still act. "Moving as one" becomes "held", a Coulomb fact, because velocity agreement needs a tolerance.

By the fork recurrence, the 3 kg piece of case (a) passes about 2,000 N·s a step, so a 100 N·s bound is already on time today. The design only changes blocks priced above about 128 kN, or lighter pieces.

## 1. The law

**L1, bond (re-derived every step).** A contact point is *stuck* if its stored normal impulse is positive and its stored tangent impulse is strictly inside `μ·λn`. DERIVED: the clamp is at `vendor/avian3d-0.8.0-overmatch/src/dynamics/solver/contact/tangent_part.rs:189-238` and the store at `dynamics/solver/plugin.rs:1058-1091`.

- A dynamic body is *held* when two of its stuck, touching points (separation ≤ 0) on different neighbours join by a line inside both friction cones (a strut). A load of any size along that line cannot move it, and its own mass does not enter.
- A held body is locked to the dynamic bodies at its strut ends.
- Compliant rows and rows with non-zero surface speed never lock, because their law permits relative motion.

**L2, the aggregate.** A connected set of locks is one solver body for the step.
- Mass is the sum; inertia is by parallel axes about the common centre.
- Velocity is the fit that conserves linear and angular momentum. This is the restitution-0 law taken exactly.
- All colliders stay. Rows between members are skipped; rows to anything else use the aggregate's mass, with anchors shifted to its centre.

**L3, split.** After each substep, for every cut of the lock forest (a tree bond plus the bonds crossing it):
- The impulse the cut must carry is Newton's law for one side: its momentum change minus its external impulses.
- That impulse must lie in the sum of the crossed bonds' Coulomb cones (compressive, tangent ≤ μ·normal).
- A failing cut unlocks, the substep is restored and re-solved once (`Resolution`: bounds cost), and its stored tangent impulse is set to the cone boundary so L1 reads "slipping" until an ordinary solve says otherwise.

**State between steps:** none beyond the engine's stored contact impulses.

## 2. Cases

Assumptions: μ = 0.6 (cone half-angle 31°), dt = 15.625 ms, rigid hull surfaces unless stated.

**(a) 57 t hull at 3 m/s, 3 kg piece, block bound 100 N·s a step, work 1,000 J.**
- Step 1 is ordinary (no stored impulses). The ceiling is 1.1·3 kg·12,920·3 = 128 kN, so the bound is reached and about 300 J is counted.
- Step 2: the hull–piece and piece–wall points are stuck and the strut runs along the push, so hull and piece are one 57,003 kg body.
- The wall row is clamped at 100 N·s, costing the hull 0.00175 m/s and 300 J a step, which is the counted work.
- The block parts on step 4, at a hull speed of about 2.994 m/s.
- Same arithmetic on the R1b 30,000 arm (2,000 kg, 0.067 kg, 4 m/s): 11.8 N·s in step 1, then 100 N·s and about 399 J a step. The count lands on step 4 and the driver loses about 1,050 J (MEASURED today: never, 9,445 J).

**(b) Heap of 300 pieces, 10 t assumed.**
- At rest, pieces with a neighbour on opposite sides form clusters; top pieces are not held.
- At first contact the hull meets a multi-tonne cluster, so `em` is about 8.5 t and the ordinary row is well conditioned.
- The common speed is 57·3/67 = 2.55 m/s.
- The cut under a 20 kg top piece needs 51 N·s forward but its bonds supply only μ·m·g·dt = 1.8 N·s a step, so it unlocks and is left behind at +0.09 m/s a step.
- The core is pushed against soil friction of 59 kN, or 920 N·s a step.

**(c) Hull at rest on a layer, soft run.**
- Soft-run rows never lock, so the hull stays out.
- Side-by-side pieces strut into a mat. Its compliant row converges at `em/(em+a)` with the mat's mass: at 120 kg and `a` = 13.4 kg, the gap after 12 solves is about 1e-12 (0.2 kg alone: 84 %).
- The final force is the spring's, as today. A lone piece is unchanged.
- A rigid stop at rest (zero surface speed) locks: the piece stands on soil with the hull's `em` and does not sink.

**(d) Glancing blow on a 5 kg piece in a cluster.**
- The new contact has no stored impulse, so it stays an external row on the cluster (say 40 kg), and the trial solve passes about 120 N·s.
- The piece's cut needs that through vertical-normal bonds, which is outside the cone. The substep is restored and re-solved with the piece alone.
- It leaves at about 3 m/s and the hull loses 15 N·s.

**(e) Two hulls at ±2 m/s on one heap.**
- Through-chains lock both hulls into one 124 t body with fitted speed 0; 228 kJ is dissipated, as in a hull-on-hull contact.
- Each chain cut carries the drive (300 kN is 4,690 N·s a step) in compression. Today a 2 kg link caps at 50 kN.
- At 400 kN against 300 kN the whole body accelerates at 0.81 m/s², so one hull shoves the other through the heap.
- A chain whose line leaves a cone unlocks and squirts out.

## 3. Guarantees and breaks

- **Momentum:** exact at merge, solve and unlock.
- **Energy:** merge only dissipates; the hull no longer loses `λ·c` in a pressed row.
- **Energy break:** overlap present at lock is frozen and released at unlock by the existing push-out. For 6 cm that is about 0.9 m/s on the light piece.
- **Two-way:** the holder's bounded reaction reaches the hull whole.
- **Pass-through:** internal overlap cannot grow, and no collider is dropped.
- **Rigidity break:** a strut is a one-direction fact but the lock is rigid in six. A locked piece cannot pivot or slip until a cut fails, and cuts test force, not moment.
- **Lateness:** the lock is always one step late, so the first-step sink is up to `c·dt` (4.7 cm at 3 m/s).
- **Determinism:** the server derives; lock components do not depend on union order. Any derived index must sort by pose bits, as `src/debris/freeze.rs` does.
- **What a player sees:** held rubble stops or slows a hull as much as its backing can; pinned pieces ride with the hull; heap cores move as clumps; a small pop on release.

## 4. Cost and seams

**Heap case** (about 900 pairs, 1,500 manifolds, 6,000 points):
- L1 is about 400 dot products a body, 1.2e5 a step. It is parallel by body over its own contacts; unions commit serially.
- L3 is about 1,200 cone tests a substep.
- Internal rows are skipped for 12 sweeps.
- Worst case is one extra substep solve per failing substep.

**Where:** `src/avian_ext/aggregate.rs`, after `PrepareSolverBodies` and before `PrepareContactConstraints`; the cut test runs after Relax.

**Fork divergences (not upstream-adoptable):**
- An alias table read at `plugin.rs:726-747` (index, inertia, anchor shift, skip equal aliases).
- Aggregate solver bodies, force summing and member write-back (`solver_body/plugin.rs:331-368`).
- Aliased rows break the colouring's one-body-per-colour invariant, which is keyed by entity (`constraint_graph.rs:224-264`). They go to a serial per-aggregate list, static rows last.
- Substep snapshot and restore.

**Game side:**
- `Pricing::price` reads the piece's own velocity and forces (`src/construction/breakage/bounded.rs:275-289`); it must read the aggregate's.
- `limit_scratch` budgets must follow the alias.

## 5. Owner's rules

- **Physical truth:** Coulomb self-locking plus the restitution-0 law; it applies to any dynamic body.
- **Tuned numbers:** none; μ is material data and the single retry is a `Resolution`.
- **Cut-offs:** none; a hull can be the held body.
- **Contacts answering:** skipped rows have zero relative velocity by construction.
- **Pass-through:** covered in section 3.
- **Honest two-way:** the hull feels the holder's row, bounded or not.
- **Outcomes:** nothing is pinned.
- **Simplest:** no. The retry and the alias are heavy, and the strut-only form is the least I could make true.

## 6. Refuting fixture

- **Arm A:** the R1b rig at 0.067 kg. Prediction: count on step 4, driver loss about 1,050 J, driver-side and slab-side impulses equal.
- **Arm B:** the same driver glancing a 5 kg piece atop a three-piece column. The design is wrong if the piece exceeds the driver's speed or the column moves.
- **Measure first (read-only probe on existing passes):**
  - Block `price.force × dt` against the 128 kN ceiling. If no block is priced above it, the law is unneeded.
  - The share of hull–piece rigid manifolds whose piece has a strut.
  - Bonds flipping per step.
  - Which hull collider kind presses pieces.

## 7. Strongest objection, and unknowns

**Objection:** the law's correctness rests on the restore-and-retry, the deepest fork change, and one retry is argued, not proven, to suffice. In a flowing heap the partition changes every step, and each change is a mass discontinuity and a warm-start loss. If play's failure sits mostly in the first-contact step or under a running belt stop, the law never engages.

**Not determined by reading:**
- How a running stop's surface speed can be read at lock time.
- Shipped piece friction.
- Manifold point counts on real rubble, and so how often struts exist.
- Sleep, island and CCD behaviour of aliased bodies.
- Whether XPBD joints or gyroscopic terms need the alias.
