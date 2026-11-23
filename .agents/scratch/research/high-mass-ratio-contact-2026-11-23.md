# High mass-ratio contact: how engines and games handle a heavy body against light pieces

Research only, 2026-11-23. Subject: rigid-body contact between a vehicle hull and loose building
rubble in the game's server physics (Avian 0.8.0 fork, 64 Hz, six substeps, two cores). Nothing
here was built or run. Every number is labelled MEASURED (by the cited source or by us), DERIVED
(by this note, from stated inputs) or STATED (a source's claim we did not check). "Inference"
marks a conclusion of this note, not of a source.

## Answer

The failure is a property of the solver family, not of our settings: sequential impulses and
soft-step (substepped soft constraints) are *dual* iterative methods, and their conditioning
degrades with mass ratio (Macklin 2020; Havok 2026; Avian's own maintainer on issue #593). Each
visit to a contact can move at most the lighter body's effective mass times the closing speed, so
force passes through a light piece in increments sized by the light piece, and a soft contact's
stiffness is likewise proportional to the lighter body's mass, so the heavy body sinks in
proportion to the ratio. Substepping reduces the error (STATED: quadratically in step size) but
does not remove the dependence; the number of substeps needed still grows with the ratio. Nobody
ships a pure-iteration fix at ratios of 10,000 and more. Engines and games do one of four things:
(1) keep a small direct solve where the heavy body is (AGX hybrid, Havok per-group direct, Roblox
LDL for joints, Bullet Dantzig blocks); (2) remove the ratio from the solve by treating the heavy
body as immovable in that pair (dominance in PhysX, Rapier and Avian; inverse-mass scaling in
PhysX, Jolt; Chaos one-way debris; Source debris); (3) remove the light bodies from the problem
(clusters, aggregates, merged or lumped bodies, client-only debris, removal on sleep); or (4)
change solver family to a primal one (VBD, AVBD), which trades the mass-ratio problem for a
stiffness-ratio one. Only (1) and (3) keep two-way reaction and momentum; (2) is the industry
default for debris and is exactly the one-way rule the owner wants to avoid; (4) is a solver
replacement with no CPU evidence at our scale. Shortlist for a design discussion: aggregate the
jammed pieces, block-solve the hull's contact neighbourhood, and (as the baseline to beat) one-way
contact below a perceptibility bound with the heap's summed reaction returned to the hull.

## Comparison table

Columns: **Keeps** = what the technique guarantees. **Breaks** = energy (E), momentum (P), two-way
reaction on the heavy body (2W), determinism (D). **Scope** = pair-local or island-level.
**Coloured** = fits a parallel graph-coloured solver. **Rules** = the game's constraints: T = from
physical truth, not a rule for one kind of object; N = no number that exists only to make a case
work; O = no permanent pass-through; W = two-way where feasible; B = fits 15.6 ms on two cores
with hundreds of bodies. A letter means "meets", a struck letter in brackets means "does not",
"?" means not established.

| # | Technique | What it is | Keeps | Breaks | Cost order | Scope | Coloured | Rules |
|---|---|---|---|---|---|---|---|---|
| 1 | More iterations | More sweeps per substep | E, P, 2W, D | nothing; just slow | sweeps needed grow about linearly with ratio (DERIVED, section 1) | pair | yes | T N O W [B] |
| 2 | More substeps (TGS, soft step, XPBD small steps) | Smaller step, one sweep each | E better than iterations, P, 2W, D | nothing; ratio dependence remains | linear in substeps; error falls about quadratically in step (STATED, Macklin 2019) | pair | yes | T N O W [B at 10,000+] |
| 3 | Soft-contact stiffness (contact hertz, damping ratio) | Spring-like bias on overlap | stability | static sink grows with ratio (DERIVED) | free | pair | yes | T [N if tuned per case] O W B |
| 4 | Warm starting and relaxation | Reuse last impulses; unbiased pass | convergence over time, less overshoot | nothing | one extra sweep | pair | yes | already in Avian |
| 5 | Contact ordering (static last, priority groups) | Solve the constraints that must win last | no pass-through of ground | order-dependent result | free | graph order | partly: order is by colour | T N O W B |
| 6 | Shock propagation (Guendelman 2003; Chaos) | Sweep up from the ground; processed levels get infinite or scaled mass | no sinking of stacks | 2W and P inside the sweep: lower bodies stop feeling weight unless normal sweeps run first | one extra ordered sweep plus a level sort | island (contact graph) | poor: levels are sequential | [T] N O [W] B |
| 7 | Dominance (PhysX, Rapier, Avian) | The dominant body has infinite mass in that pair | dominant body never yields | P, 2W, E in that pair | free | pair | yes | [T] N O [W] B |
| 8 | Inverse-mass or inertia scaling per contact (PhysX, Jolt, Chaos, Havok tools) | Scale a body's inverse mass in one pair | conditioning | P and 2W in proportion to the scale; reported impulses no longer true | free | pair | yes | [T] [N] O [W partly] B |
| 9 | Mass splitting (Tonge 2012) | Divide a body's mass by its contact count so contacts solve in parallel | jitter-free parallel Jacobi; converges to the LCP (STATED) | nothing at convergence | Jacobi-rate convergence | pair | yes, replaces colouring | T N O W ? (does not target ratio) |
| 10 | Block or direct solve of a small group (Dantzig, Lemke, LDL, Schur complement) | Solve a set of rows together, exactly | E, P, 2W; ratio-insensitive | D only if pivot order varies; friction usually left iterative | about cubic in rows; Dantzig STATED 1 ms at 200 rows, 30 ms at 600 (2014 hardware) | group or island | the block is one serial unit between coloured passes | T N O W B if the block is bounded |
| 11 | Reduced coordinates (Featherstone) | Joint chains as one articulated body | exact joints, O(n) | applies to joints, not contact heaps | O(n), plus O(n) per contact row | chain | separate pass | not applicable to heaps |
| 12 | XPBD rigid bodies | Position-level dual solve with substeps | E fairly well, P, 2W | still dual: error at high ratio (STATED, AVBD authors) | as 2 | pair | yes | T N O W [B at 10,000+] |
| 13 | Primal solvers (VBD, AVBD) | Solve each body's position against all its constraints | insensitive to mass ratio (STATED) | sensitive to stiffness ratio; AVBD adds dual stiffness ramp | per-body 6x6 solve per iteration; GPU results only | body-local | yes, colours bodies (fewer colours) | T N O W ? |
| 14 | Long-range constraints (Mueller 2017) | Distance bounds from each body to the ground through the contact graph | instant support through a pile | built for resting support, rebuilt each step; push-through not shown | graph analysis per step | island | constraints are far-reaching, so colours collide | T N O W ? |
| 15 | Speculative contacts | Contact rows before touching | less tunnelling | can fail when the solver itself accelerates a body through (STATED, PhysX) | free | pair | yes | already in Avian; not a ratio fix |
| 16 | One-way debris (Chaos, Source, WoT) | Light pieces are pushed, never push back | heavy body unaffected; cheap | P, 2W | cheaper than two-way | pair or class | yes | [T] N O [W] B |
| 17 | Kinematic or velocity-driven heavy body | The vehicle is not dynamic against debris | as 16 | P, 2W; the vehicle ignores every contact force | cheap | body | yes | [T] N O [W] B |
| 18 | Aggregates: clusters, adaptive merging, lumped bodies (Chaos clusters, Coevoet 2020, AGX terrain) | Pieces that move as one are one body until they stop doing so | P, 2W; ratio falls to hull over aggregate; chain length falls | internal contact forces are estimated, not solved; merge and split thresholds | cheaper: fewer bodies and rows | island (graph) | yes: fewer constraints | T N? O W B |
| 19 | Heap as a medium (AGX terrain; heightfield or particles) | The heap is a continuum with lumped bodies coupled to the machine | P, 2W through the lumped body | individual pieces lose identity | cheap at runtime; a second model | region | separate system | T N? O W B, but a new system |
| 20 | Removal and sleeping (Chaos remove-on-sleep, crumbling) | Settled or small pieces leave the solver | budget | the piece itself | negative | island | yes | T N O W B; does not help a live push |

## 1. Why it happens

**What sources state.**

- PGS and sequential impulses with 4 to 10 iterations "often fail to converge"; the standing
  advice from Havok was to avoid large mass ratios, shorten the step or raise inertia, and Halo 2
  pinned masses to at most 16:1 (Coumans, GDC 2014 notes,
  https://archive.org/stream/GDC2014Coumans/GDC2014-Coumans_djvu.txt). His chain test (unit-mass
  links, end mass 1 to 1000, 60 Hz): at 1:1000 the velocity error "stays around 0.02" however many
  iterations are added and reaches zero "only after 10000 iterations or so"; at 240 Hz the error
  is about 0.005 (MEASURED by the source).
- "Solver convergence is fundamentally limited by the condition number of the constraint system";
  "the effective mass for the row is dominated by the lighter body, which limits how much
  correction the solver can apply per iteration"; mass ratios are a case "where more iterations
  alone cannot cost-effectively close the gap" (Havok, 3 April 2026,
  https://www.havok.com/blog/how-havoks-constraint-solver-works-pgs-baumgarte/).
- Dual methods (PGS, XPBD: unknowns are constraint multipliers) are ill-conditioned by mass ratio;
  primal methods (unknowns are positions) are ill-conditioned by stiffness ratio, and the two
  condition numbers mirror each other on a 1D chain (Macklin, Erleben, Mueller, Chentanez,
  Jeschke, Kim, "Primal/Dual Descent Methods for Dynamics", 2020, sections 5 and 7.1, figures 5,
  6 and 8, https://mmacklin.com/primaldual.pdf). Their contact stack at 4096:1 holds with a primal
  solver in 20 iterations and fails with the dual one at 500 (MEASURED by the source, figure 8).
- Substeps beat iterations at equal cost: a chain with a 1:100,000 end mass has error 322.1 m
  with 100 iterations and 3.2 m with 100 substeps, "in line with our prediction of quadratic error
  reduction with respect to time step size" (Macklin, Storey, Lu, Terdiman, Chentanez, Jeschke,
  Mueller, "Small Steps in Physics Simulation", 2019, section 6,
  https://mmacklin.com/smallsteps.pdf). The error at 100 substeps is still 3.2 m: substepping
  shrinks the problem, it does not end it.
- "The number of sub-steps required depends on the specific mass ratios present, and it is not
  easy to predict how many are required in the worst case" (Giles, Diaz, Yuksel, "Augmented Vertex
  Block Descent", SIGGRAPH 2025, section 2,
  https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25.pdf).
- "TGS refers to preferring sub-stepping over iteration"; "It has long been known that smaller
  time steps are more effective than more iterations"; relaxation "is expensive, but it improves
  the simulation quality dramatically" (Catto, "Solver2D", 2024,
  https://box2d.org/posts/2024/02/solver2d/). The post tests large mass ratios but states no
  ratio limit; its results are in video.
- PhysX lists "Improved handling of high-mass ratios" among TGS's advantages over PGS without a
  bound (PhysX 5.4.1, Rigid Body Dynamics,
  https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html).
- Bepu: "One notable pathological case for the solver is high mass ratios"; "A tiny box can sit on
  top of a tank without any issues at all, but a tank would squish a tiny box"; "Using higher
  update rates can enable the simulation of otherwise impossible mass ratios, like 1000:1"
  (https://github.com/bepu/bepuphysics2/blob/master/Documentation/StabilityTips.md).
- Avian's maintainer, on a 1:1000 report: "a known limitation ... which the vast majority of
  physics engines and constraint solvers struggle with (specifically dual solvers)"; "the contact
  between the two dynamic boxes only 'sees' their masses ... Each individual contact has no real
  awareness of any other interactions" (https://github.com/avianphysics/avian/issues/593).

**DERIVED here (idealised; trends, not predictions of our rig).**

- *Convergence.* Heavy body `M` pressing a light piece `m` against something fixed is two rows
  sharing the light piece: `K11 = 1/M + 1/m`, `K22 = 1/m`, `K12 = 1/m`. Gauss-Seidel's error
  contracts by `K12^2 / (K11 K22) = M / (M + m)` per sweep, so one e-fold of error takes about
  `R = M/m` sweeps. Avian runs 12 normal sweeps a tick at six substeps (one biased and one relax
  sweep each). With warm starting carrying the impulses across substeps and ticks, an e-fold takes
  about `R/12` ticks: 1.3 s at 1,000, 13 s at 10,000, 39 s at 30,000. The MEASURED rig (on time to
  1,000, late at 10,000, never at 30,000) has the same linear trend; the soft bias and the block's
  strength bound are outside this model, so the absolute times are not comparable.
- *Per-visit bound.* One visit to a row changes its impulse by at most `m_eff * |v_rel|` with
  `m_eff <= m`. Force through the light piece therefore grows by at most about `m * v_rel` per
  sweep, whatever the heavy body could supply. Treating the heavy body as immovable (dominance)
  does not lift this bound: the increment is still sized by the light piece.
- *Sink.* In Avian's biased solve
  (`src/dynamics/solver/contact/normal_part.rs:144-148` at v0.8.0) the fixed point under a steady
  load `F` is an overlap of `F / (m_eff * omega^2)`: the soft contact is a spring of stiffness
  `m_eff * omega^2`, and `m_eff` is about the lighter body's mass. At our settings the contact
  frequency is `1.5 * min(1/(2 dt), 0.25/h) = 48 Hz` between dynamic bodies and 96 Hz against
  static or kinematic ones (`src/dynamics/solver/plugin.rs:625-649`), so a heavy body whose weight
  bears on a piece `R` times lighter sinks about `R * g / omega^2 = R * 0.108 mm`: 1.1 cm at 100,
  10.8 cm at 1,000. The bias speed cap (`max_overlap_solve_speed`, 4 m/s) binds only beyond 27.6
  cm. This matches "sinks centimetres" in kind and explains why substepping alone leaves it: the
  frequency is capped by the tick, not the substep, once `1/(2 dt) < 0.25/h`.

**What substepping fixes and what it does not.** It fixes non-linearity (contact geometry updated
within the tick), energy behaviour and joint stretch at moderate ratios (sources above). It does
not change the per-visit bound or the mass scaling of soft-contact stiffness (DERIVED above), and
the count needed grows with the ratio (STATED, AVBD).

## 2. Solver-side techniques

### Ordering and shock propagation

- Guendelman, Bridson, Fedkiw, "Nonconvex Rigid Bodies with Stacking" (SIGGRAPH 2003), sections
  8.1 and 8.2 (https://graphics.stanford.edu/papers/rigid_bodies-sig03/rigid_bodies.pdf): bodies
  are sorted into levels of a contact graph from the ground up; on the last sweep "all the objects
  in that level are assigned infinite mass". The authors state the cost: applied alone, "the
  see-saw either sits still or tips very slowly since it does not feel the weight of the heavy
  block", so ordinary sweeps run first "to get a sense of weight". Level order is defined against
  gravity; a horizontal push by a vehicle has no such order (inference).
- Erleben, "Velocity-based shock propagation for multibody dynamics animation" (ACM TOG 26(2),
  2007): shock propagation combined with a velocity-level complementarity solver. Read only
  through secondary summaries.
- Unreal Chaos has collision shock propagation as mass scaling per phase
  (`EnablePositionShockPropagation`, `EnableVelocityShockPropagation`, cvars
  `p.Chaos.Solver.Collision.PositionShockPropagationIterations` and
  `...VelocityShockPropagationIterations`). Source: Epic's API index as returned by search; the
  pages did not render for us, so the scale values and level rule are not established.
- Havok orders by priority: "Contact constraints are prioritized above joint constraints, and
  static-to-dynamic contacts receive the highest priority"; "Higher-priority groups are solved
  last" (Havok 2026, above). Avian does the same by colour: constraints with a static body take
  the last colours and overflow is solved first (section 4).
- Guarantees: no sinking of resting stacks in one sweep. Breaks: two-way reaction and momentum
  within the sweep. Cost: a graph sort and one ordered sweep. Island-level. Poor fit to colouring:
  levels are a sequence, colours are not.

### Dominance and inverse-mass scaling

- PhysX: "Dominance effectively imbues the dominant body in a pair with infinite mass"; "a form of
  local mass modification within the constraint solver"; 32 groups, pair values only 0 or 1;
  "Dominance can produce some very peculiar results if misused" (Rigid Body Dynamics, above).
  Contact modification allows any scale: "inverse mass/inertia scales of 0.5 result in the body
  appearing to have double the mass", and reported impulses then "may no longer accurately
  reflect the change in momentum" (Advanced Collision Detection,
  https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/AdvancedCollisionDetection.html).
- Rapier: "Dominance is a non-realistic, but sometimes useful, feature"; groups in [-127, 127];
  the higher group "will act as if it has an infinite mass"
  (https://rapier.rs/docs/user_guides/rust/rigid_bodies/). The same page says a stack "involving
  large mass ratios, might require more solver iterations".
- Jolt: `ContactSettings::mInvMassScale1/2`, `mInvInertiaScale1/2`: "0 = infinite mass, 1 = use
  original mass, 2 = body has half the mass"; "you should strive to keep the value the same over
  time" (https://jrouwe.github.io/JoltPhysics/class_contact_settings.html). Jolt's architecture
  page describes a sequential-impulse solver with warm starting and a separate position pass; we
  found no statement on mass ratios there.
- Box2D v3: no dominance and no mass scaling in `src/contact_solver.c` (main, read 2026-11-23);
  the only pair distinction is stiffer softness against static bodies (lines 41-43, 113-117).
- Bullet: we found no dominance feature; a forum thread suggests clamping the ratio during solver
  setup (secondary, https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=650).
- Havok: authoring-time "inertia optimization" and "mass optimization" for ragdolls, and a
  per-skeleton "Stabilize Inertias" tool (Havok 2026). These rebalance joints, not contacts.
- Guarantees: the dominant body is never pushed by that pair; conditioning of that row no longer
  depends on the ratio. Breaks: momentum, two-way reaction, and energy bookkeeping. Does not fix
  push-through by itself: the per-visit bound is still the light piece's (DERIVED, section 1).
  Pair-local, free, fits colouring.

### Mass splitting

Tonge, Benevolenski, Voroshilov, "Mass Splitting for Jitter-Free Parallel Rigid Body Simulation"
(SIGGRAPH 2012, https://dl.acm.org/doi/10.1145/2185520.2185601). A body's mass in the effective
mass is divided by its contact count so contacts can be solved Jacobi-style in parallel; the
abstract claims convergence to the LCP solution and a 5,000-body pile above 60 fps on a GPU.
It targets parallelism and jitter, not mass ratio; splitting a light body's mass further lowers
the per-visit bound (inference). Abstract read only.

### Direct and block solvers

- Bullet: `btMLCPSolver` with Dantzig, Lemke and NNCG. Dantzig "shows very good convergence for
  problems with large mass ratios"; "doesn't scale well for larger problems. It takes about 1
  millisecond to solve 200 constraint rows", "at 600 constraint rows it takes about 30
  milliseconds"; "Occasionally fails, needs fallback (PGS)"; conclusion: "Big matrix direct MLCP
  solvers can be useful as a block solver within a PGS/SI loop" (Coumans 2014, above; MEASURED by
  the source on 2014 hardware). NNCG reaches at about 400 iterations what PGS reaches at 10,000.
- Havok: "The direct solver operates on a constraint group and solves all constraints in the
  group simultaneously", "enabled per constraint group at runtime", and "constraints can be spread
  across both the direct solver and iterative solver" (Havok 2026). Its examples are joint
  systems; whether contacts can join a direct group is not stated.
- Roblox: LDL-PGS hybrid since 2018; the sparse LDL factorisation "ensures that the linear system
  coming from equality constraints is solved exactly"; contacts stay in PGS ("we are investigating
  direct methods applied to inequality constraints and collisions"); "only slightly slower
  compared to pure PGS"
  (https://about.roblox.com/newsroom/2020/08/improving-simulation-performance-advanced-physics-solver).
- AGX Dynamics: by default "all constraints including ordinary constraints and contacts are
  handled with the direct solver, except for the friction of contacts"; "The hybrid solver is able
  to handle large mass ratios in stacking and constrained systems". Its own recommended setup for
  a machine in a rock pile is the closest published analogue to our case: "ITERATIVE contact
  solver is used between rocks (for scalability)", "SPLIT contact solver is used between rocks and
  ground" and between rocks and the shovel, "DIRECT contact solver is used between the
  lifter/wheels and the ground"
  (https://www.algoryx.se/documentation/complete/agx/tags/latest/doc/UserManual/source/creating_stable_simulations.html).
  No cost figures are given.
- Peiret, Andrews, Koevecses, Kry, Teichmann, "Schur Complement-based Substructuring of Stiff
  Multibody Systems with Contact" (ACM TOG 38(5), 2019, https://dl.acm.org/doi/10.1145/3355621):
  subsystems solved directly and coupled through interface impulses, for real-time rates with
  large mass ratios; speed-up is "considerable" for structured scenes and "moderate" for piles.
  Abstract read only.
- Featherstone: O(n) for trees of joints; "cannot deal with closed loops, contacts, joint limits
  [and] motors by itself", and "each contact constraint requires O(n) time" (Coumans 2014). Not a
  tool for heaps.
- Guarantees: exact normal forces for the rows in the block, so the ratio stops mattering there;
  momentum, energy and two-way reaction kept. Breaks: nothing physical; determinism needs a fixed
  pivot order; friction is normally left to the iterative pass. Cost: about cubic in rows, so the
  block must be bounded. Group-level. In a coloured solver the block is one serial unit.

### Position-based and primal methods

- Mueller, Macklin, Chentanez, Jeschke, Kim, "Detailed Rigid Body Simulation with Extended
  Position Based Dynamics" (2020), sections 4 and 5
  (https://matthias-research.github.io/pages/publications/PBDBodies.pdf): substepping "allows the
  handling of large mass ratios"; shown at 1:1000 (two hanging boxes) and 1:760 (a steering
  linkage). Joint cases, not contact heaps.
- The AVBD authors report the opposite at higher ratios: "being a dual method, XPBD can result in
  significant error with high mass ratios"; in a 50-link pendulum with a 50,000:1 end mass, XPBD
  and sequential impulses stretch and AVBD does not (section 5.2, figure 7; MEASURED by the
  source). AVBD colours bodies, not constraints, "resulting in significantly fewer colors". All
  results are from a GPU implementation; a CPU budget at hundreds of bodies is not reported. VBD's
  own weakness is stated: with high absolute stiffness, light bodies "appear heavier than they
  really are".
- Avian's joints are XPBD today (`src/dynamics/solver/plugin.rs:45-46` at v0.8.0), so they share
  the dual limit.

### Long-range constraints

Mueller, Chentanez, Macklin, Jeschke, "Long Range Constraints for Rigid Body Simulations" (SCA
2017, https://matthias-research.github.io/pages/publications/LongRangeConstraintsSCA.pdf): the
contact or joint graph is analysed every step to add upper and lower distance bounds between
far-apart bodies, so error propagates through a pile at once. Abstract read only. Whether it helps
a lateral push through a heap, as opposed to resting support, is not established.

### Speculative contacts

Not a mass-ratio technique. PhysX states the limit that matters here: "there are cases where it
can fail", for example when the solver accelerates a body so that it passes through in one step
(Advanced Collision Detection, above). Avian uses the same row
(`normal_part.rs:127` at v0.8.0).

## 3. Game-side practice

| Game or engine | What it does | Source and strength |
|---|---|---|
| Unreal Chaos destruction (UE 5.4+) | "One Way Interaction" from a set cluster level: such objects "do not further the destruction", "will not affect two-way objects", and collide with each other as cheap spheres; small debris moves to GPU particles; "Remove on Sleep" and crumbling delete settled pieces | Epic docs, primary: https://dev.epicgames.com/documentation/unreal-engine/chaos-destruction-optimization?application_version=5.6 ; Epic tutorial (via search): https://dev.epicgames.com/community/learning/tutorials/KWeX/unreal-engine-a-tech-artist-s-playbook-for-chaos-performance |
| Unreal Chaos | Clusters: fractured pieces are one rigid body until strain releases them, with release throttled per step (`p.Chaos.Clustering.PerAdvanceBreaksAllowed`) | Same Epic doc, primary |
| The Finals (Embark) | All physics and destruction on the server: "We don't run physics on the clients at all"; clients receive "the correct transforms for the moving platforms, the debris piles" | Developer podcast transcript, 2023: https://www.thefinals.wiki/wiki/Meet_The_Makers%CB%90_Movement . Nothing on mass ratios |
| Source engine (Valve) | `prop_physics` "Debris" flag: no collision with players or other debris | Valve Developer Community (via search): https://developer.valvesoftware.com/wiki/Prop_physics |
| World of Tanks (Havok destruction) | Debris simulated on each client only; the server sends the break event; tank contact with debris is local | Leak plus comments, secondary and dated 2015: http://ftr.wot-news.com/2015/01/23/the-insider-returns-havok-chernobyl-and-more/ |
| Halo 2 | Masses pinned "so that there is never more than a 16:1 ratio" | Coumans 2014 quoting Havok's Oliver Strunk, primary talk, second-hand fact |
| AGX Dynamics (training simulators for loaders and excavators) | Soil as particles under an iterative solver; disturbed soil lumped into aggregate bodies that couple to the machine through the direct solver; force within 10 to 25 % of a fine reference at over 1,000 times the speed (STATED) | Servin, Berglund, Nystedt, "A multiscale model of terrain dynamics for real-time earthmoving simulation", 2021: https://arxiv.org/abs/2011.00459 ; AGX manual, above |
| Research, not shipped | Adaptive merging: bodies sharing a velocity become one body; a one-iteration PGS on internal contacts decides when to split | Coevoet, Benchekroun, Kry, SIGGRAPH 2020: https://eulaliecoevoet.github.io/AdaptiveMerging/ (abstract) |
| Bepu | Advice only: "avoid making heavy objects depend on light objects" | Bepu docs, above |

Reading across the row set (inference): every shipped game we could source handles a vehicle
against debris by removing the two-way coupling (one-way, no collision, client-only) or removing
the pieces (clusters until release, removal on sleep). The only sourced systems that keep a heavy
machine and a loose heap coupled two ways are engineering simulators, and they do it with
aggregates plus a direct solve at the machine.

## 4. What Avian 0.8.0 offers

Read from the tag `v0.8.0` (https://github.com/avianphysics/avian/tree/v0.8.0); line numbers are
upstream's. Our vendored copies of `plugin.rs`, `constraint_graph.rs`, `schedule.rs`,
`dynamics/mod.rs` and `rigid_body/mod.rs` are identical to upstream (MEASURED by diff).

- **Solver type.** "Avian uses an impulse-based solver with substepping and soft constraints.
  Warm starting is used to improve convergence, along with a relaxation pass to reduce
  overshooting" (`src/dynamics/solver/plugin.rs:39-40`). Per substep: integrate velocities, warm
  start, solve with bias, integrate positions, relax (`plugin.rs:67-71`). This is Box2D v3's soft
  step. There is no iteration count: one biased and one relax sweep per substep.
- **Substeps.** `SubstepCount`, default 6 (`src/dynamics/solver/schedule.rs:159-195`).
- **Soft-constraint parameters.** `SolverConfig` (`plugin.rs:508-601`):
  `contact_damping_ratio` 10.0, `contact_frequency_factor` 1.5, `max_overlap_solve_speed` 4.0,
  `warm_start_coefficient` 1.0, `restitution_threshold` 1.0, `restitution_iterations` 1. Contact
  frequency is `factor * min(1/(2 dt), 0.25/h)`, doubled against static and kinematic bodies and
  against a dominant body (`plugin.rs:625-649`; `src/dynamics/solver/contact/mod.rs:145-149`).
  The softness is global: no per-pair or per-body override.
- **Dominance.** `Dominance(i8)`, -127 to 127, default 0: "The body with a higher dominance acts
  as if it had infinite mass" (`src/dynamics/rigid_body/mod.rs:623-654`); applied by zeroing the
  dominant body's inverse mass and inertia in that constraint (`contact/mod.rs:119-143`). Contacts
  only; joints ignore it (open issue #180, "Dominance for joints?").
- **Colouring and order.** Greedy edge colouring, 24 colours; constraints between two non-static
  bodies take the first 20, constraints with a static body are assigned from the last colour
  backwards "giving higher priority to those constraints, reducing tunneling through static
  geometry"; what does not fit goes to overflow, which "can happen when a single body is
  interacting with many other bodies" and is solved serially and first: "They have lower priority,
  so they are solved first" (`src/dynamics/solver/constraint_graph.rs:53-64, 224-256`;
  `plugin.rs:867`). A colour under 64 constraints is solved serially (`plugin.rs:887`).
  Inference: a hull touching more than 20 dynamic manifolds puts its further contacts in overflow,
  so the rows that carry the push are solved first each sweep, at the lowest priority. Where our
  hull's rows land is worth measuring before any design.
- **No** block solver, direct solver, shock propagation, per-pair mass scaling, or per-contact
  softness upstream. Joints are XPBD.
- **Stated limitation.** "High mass ratios, such as when a very heavy object rests on top of a
  lighter one, can be difficult for the engine to deal with" (`src/dynamics/mod.rs:40-41`).
- **Issues.** #593 (closed 2025-11-24) is the only issue on mass ratio; the maintainer's answer is
  quoted in section 1 and proposes no fix beyond closer masses and softer contact frequency. #177
  and #178 added dominance. We found no open issue or discussion planning a mass-ratio feature.
  The 0.8 release post lists SIMD for the contact solver as the next solver priority and credits
  contact recycling with stack stability ("a 100 cube stack is easily stable with just 4
  substeps"; https://joonaa.dev/blog/14/avian-0-8).
- **Our fork** already adds two per-point hooks in the normal solve: a compliant spring and an
  upper impulse bound (`vendor/avian3d-0.8.0-overmatch/OVERMATCH_PATCH.md`, "Compliant contact",
  "Bounded contact"). Neither changes convergence.

## 5. The shortlist against the game's rules

1. **Aggregate the jammed pieces (table row 18).** Pieces that share a velocity with each other,
   with the ground or with the hull are one body for the solve until their internal contact
   forces say otherwise. The hull then meets `M` against the sum of the pieces, and a chain of `n`
   becomes one contact. A truth, not a rule: a jammed heap does move as one body. Keeps two-way
   reaction and momentum, lowers cost. Risks: the split criterion is an authored threshold (must
   be a law, not a tuned number), and a merged aggregate needs its own strength reading for our
   breakage laws. Graph-level work outside the solver; fewer constraints for the coloured solver.
2. **Block-solve the heavy body's neighbourhood (row 10).** The hull's normal rows and the rows of
   the pieces it presses on, solved together and exactly, between the coloured passes; everything
   else stays iterative. This is AGX's published layout for a machine in a rock pile and Bullet's
   and Havok's stated use of direct solvers. Keeps every conservation property. Cost is bounded by
   capping the block (STATED 1 ms per 200 rows, 2014); the serial overflow slot already exists.
   Risks: a pivoting solver's failure cases need a deliberate outcome; friction stays iterative;
   it is fork code inside the solve.
3. **One-way below a perceptibility bound, with the heap's reaction returned (rows 7, 16), as the
   baseline.** This is what shipped games do, and Avian's `Dominance` does it today at no cost.
   It breaks momentum and two-way reaction, so it meets the rules only if the hull receives the
   summed reaction through another channel, and the bound is stated as a perceptibility fact and
   not per object kind. It does not by itself fix push-through (per-visit bound, section 1). It is
   on the list as the cheap reference the other two must beat, not as a recommendation.

Not shortlisted: more iterations or substeps (owner's position; sources agree the count scales
with the ratio); shock propagation (its order is gravity's, and it removes weight); mass
splitting (targets parallelism); long-range constraints (resting support only, as far as
established); a primal solver (AVBD) is the one result that claims to remove the problem at its
root, but it replaces the solver and has no CPU evidence at our scale: a watch item.

Two cheap measurements would sharpen any design: where the hull's rows land in colour order
(overflow or not), and whether the MEASURED sink matches `F / (m_eff * omega^2)`.

## What we could not establish

- Any primary source for how Battlefield (Frostbite), Red Faction: Guerrilla, Teardown, Rainbow
  Six Siege, GTA or War Thunder treat a vehicle against debris. Searches returned press and
  forums only. The Red Faction GDC talk and Teardown's solver talk were not found in text form.
- A shipped game that represents rubble as a heightfield or granular medium against vehicles. The
  only sourced instance is AGX's terrain model, a simulator.
- Chaos shock propagation's rule and scale values: Epic's API pages did not render; names come
  from a search index. Unreal's source was not read (login required).
- Whether Havok's direct solver accepts contact constraints, and its cost.
- AGX direct-solver cost and scaling; the manual gives none.
- Catto's April 2025 statement that soft step handles "high mass ratios and long chains" without
  a direct solver (https://x.com/erin_catto/status/1908939691321344426) was seen only as a search
  snippet; no ratio or scene numbers were found in text. The Solver2D post's per-solver results
  are in video.
- Full texts not read (abstract or secondary summary only): Tonge 2012, Erleben 2007, Mueller
  2017 (long-range constraints), Coevoet 2020, Peiret 2019. Claims from them above are their
  abstracts'.
- Unity Physics per-contact mass factors and the Rapier and Jolt solvers' internals were not
  read; Jolt's docs contain no statement on mass ratios that we found.
- AVBD on CPU, at hundreds of bodies, in a 15.6 ms budget: no data. Its published results are
  GPU.
- The DERIVED convergence and sink figures in section 1 are from an idealised two-row model and a
  single-row fixed point; neither was checked against our rig.
