<!-- Design C, paper only, 2026-11-23; claims DERIVED unless marked -->

# Design: the backed tree pass

**Decision.** I did not replace the solver's rows on loose pieces with a price, because a loose piece has no single mover (fork facts §5, item 4). Instead, one exact evaluation per substep runs on the same rows: the contact graph is cut to a support tree, and the tree's normal rows are solved together with their bounds. The result is written into the rows' own accumulated impulses. The law never names a body kind.

## 1. The law

1. **When.** Each substep, after warm start and before the biased sweep. DERIVED: the sets are chained (`vendor/avian3d-0.8.0-overmatch/src/dynamics/solver/schedule.rs:62-70`), and `deal` already writes constraints in this schedule (`src/avian_ext/reaction_bound.rs:271-277`).
2. **Tree.** Per connected group of non-static bodies, breadth-first over body pairs with a touching manifold (separation ≤ 0 or carried impulse > 0), built once a step.
   - The root is the group's most massive body; neighbours are taken in descending mass, then `ContactId`.
   - Reason for mass order: a row's increment is sized by its lighter body (facts §3), so the shortfall runs from heavy to light.
   - Root choice only decides which loop edges are cut. Cut edges stay with the ordinary rows.
   - Static bodies are never nodes; each static contact is a leaf.
3. **Behind.** Body i carries a 3×3 inverse apparent mass `W_i`, starting at `I/m_i`. Each child edge j (normal `n_j`, child value `w_j`, compliance `γ_j` for a compliant row, else 0) applies `W ← W − W n nᵀ W / (nᵀWn + w_j + γ_j)`.
   - A static normal leaf has `w = 0`: it locks that direction.
   - A static contact also gives two tangent locks, capped at `μλ_n − |λ_t|` from the row's own `friction` and impulses.
   - Seen from the parent, `w_i = n_iᵀ W_i n_i`. This is the exact translational inertia of what must move (elimination of a tree). Rotation is omitted.
4. **Bounds.** An edge's cap is `Σ(max_impulse − impulse)` over its points: a bonded block's dealt bound arrives this way (`reaction_bound.rs:602-603`). A budgeted point also takes the seam's `cap`.
5. **Solve.** Root to leaf, each edge takes the impulse that zeroes its closing speed given `K_e = nᵀW_parent n + w + γ` (a compliant edge closes toward its spring target). A capped edge asked for more than its cap becomes a constant force at the cap and leaves `W`; the pass repeats. A released edge is never re-locked, so it terminates.
6. **Apply.** Each edge impulse is added to its points' `normal_part.impulse` and applied to both bodies as the row applies it (`contact/mod.rs:372-376`): equal and opposite at one point.
   - Nothing is counted twice: the next sweep, warm start, the friction clamp and `Bounds::impulse` all read the accumulator the pass wrote.
7. **State.** None beyond the rows' impulses and the step's tree.

## 2. Hand-worked cases

Substep `h` = 2.604 ms. Restitution 0.

**(a) One piece against a bonded block.** M = 57,000 kg, c = 3 m/s, m = 3 kg, head-on, piece touching the block. The dealt bound is 100/6 = 16.67 N·s a substep.
- Pass 1: the block locks, `K = 1/M`, asked 171,000 N·s, over the cap. Release.
- Pass 2: the block is a constant 16.67 N·s; `w = 1/3`. Hull–piece impulse = 16.67 + 9.0 = 25.67 N·s. Piece leaves at 3.0 m/s; hull at 2.99955 m/s.
- Later substeps: 16.67 N·s each. The piece closes on the block at 3 m/s, so the seam counts 25 J, then 50 J a substep: 275 J in step 1, 300 J a step after.
- 1,000 J is reached in step 4. The hull has lost about 1,027 J (1,000 counted, 13.5 piece kinetic energy, 13.5 plastic).
- Today this case is already on time: the facts' recurrence gives 19.2 N·s in the first substep (κ = 1.1), above 16.67. It fails today when the bound exceeds the ceiling of about 3,290 N·s a step for this piece. An illustrative 1 MN block (15,600 N·s a step) is never reached today; here it gets 2,600 N·s in substep 1.

**(b) Heap of 300 pieces on soil.** Assumed: 9 t total, μ = 0.6, all touching, all coupled through tree edges.
- Friction leaves can return 0.6 × 9,000 × 9.81 × h = 138 N·s a substep against 171,000 asked. All release in pass 2.
- Common speed = (171,000 − 138) / 66,000 = 2.59 m/s. The hull loses 23,400 N·s in substep 1; 35 kJ is lost as a plastic impact (½ × 7.77 t × 9).
- After that, 53 kN of friction slows the 66 t at 0.80 m/s². Pieces reached only through cut edges get the push from the rows.

**(c) Hull at rest on small pieces.**
- Soft run: the edge has `γ = 1/a`, and the soil locks the piece (`w = 0`). The row reaches the spring's `h·k·depth` in substep 1, and the same impulse lands on the piece–soil row. At the fixture law (a = 13.4 kg) a 0.2 kg piece today leaves 84 % of the gap after a step (facts §5). The fixed point is still the spring's deflection.
- Rigid stop or proxy: the pass gives M·g·h = 1,456 N·s a substep (559 kN) in substep 1. Today a 5 kg piece "does not settle".

**(d) Glancing blow at 20°.** Normal closing is 1.03 m/s. The friction leaf's cap is 0.046 N·s and releases. `K ≈ 1/m`, the same as the row's own, so the pass adds nothing: 3.1 N·s, and the piece flies as today.

**(e) Two hulls, heap between.** One tree; the second hull is a node of mass M. Head-on at 3 m/s each, a jammed path stops both in one substep: 171,000 N·s, 513 kJ plastic.
- That is 65.7 MN through 2–200 kg pieces, with nothing bounding it: a missing truth, the piece's own crush price.
- The second hull has one parent edge. Its other contacts are cut edges, so its load goes through one piece.

## 3. What it guarantees and what it breaks

- **Momentum:** kept exactly, linear and angular, edge by edge.
- **Energy:** an exact translational solve to zero closing never adds energy. Applied through real lever arms with rotation omitted, non-increase is not proven.
- **Two-way:** the heavy body receives exactly what the tree's far end returns, in the step it presses.
- **Pass-through:** the rows, bias and speculative margin are untouched. The pass removes the sink into held pieces. Overlap of a piece into a block that is giving remains, as for a hull on a cell today.
- **Determinism:** fixed order (mass, `ContactId`, then tree order). `ContactId` depends on contact history, as the colouring already does. I did not read ADR-0037.
- **Player sees:** rubble jammed against something that holds stops or slows a hull; a heap moves ahead as a mass instead of swallowing the hull; walls behind rubble take the push on time; a hull does not sink into rubble under a rigid surface.

## 4. Cost

- **Order:** O(bodies + leaves) per pass.
- **Heap case:** about 700 edges × 70 flops ≈ 50k flops a pass. An assumed 3–5 passes and 6 evaluations give about 1–1.5M flops a tick, which I estimate at 0.5–1.5 ms. The pass count is the unknown.
- **Tree build:** once a step, one walk over about 1,500 manifolds.
- **Where:** a game system at one seam (a new `src/avian_ext/backing.rs`) in `SubstepSchedule`.
- **Parallel:** by group. Each group writes only its own rows and bodies, statics take nothing, and `deal` has run. Within a tree the leaf-to-root dependency is serial.
- **Fork:** no new patch if a game system can read each body's inverse mass. `ConstraintGraph.colors` is public (`constraint_graph.rs:147`), and `reaction_bound` already takes `ResMut<SolverBodies>` (`reaction_bound.rs:303`, `:438`). The fields of `SolverBodyInertia` are private (`solver_body/mod.rs:268-280`); I did not confirm its accessor is public. If it is not, exposing it is an upstream-adoptable accessor.

## 5. The owner's rules

- **Physical truths:** the law is rigid transmission through touching contacts, with Coulomb caps and bond bounds.
- **No body kind:** the root is chosen by mass, for the stated reason, and every group gets a tree.
- **No tuned number:** none is authored. The pass count is structural.
- **No cut-off:** "touching" is the contact's own state.
- **No contact stops answering:** every row still solves.
- **No permanent pass-through:** unchanged, and improved for held pieces.
- **Honest:** a piece returns only what its subtree's inertia, friction and bounds return.
- **Outcomes not pinned:** nothing is asserted about stalling or crossing.
- **Simplest true form:** a tree, not a matrix. It is the tree-shaped limit of a block solve.

## 6. The refuting fixture

- **Before building:** extend the facts report's throwaway recurrence with the pass on the R1b arm at 0.067 kg. Expected: the bound is reached in step 1 and the driver's loss is near the count (about 1,000 J against the MEASURED 9,445 J).
- **Refuter:** in the seam fixture (`src/tests/avian_reaction_bound.rs:119-203`), a 2,000 kg driver presses a 0.2 kg cube off-centre at one corner against the slab, with a second cube touching both (a loop). The design is wrong if total kinetic energy rises in any substep, or a piece leaves faster than the driver.
- **Census in a real heap:** the share of touching edges that are cut, and passes per evaluation.

## 7. Strongest objection, and what I could not determine

**Objection.** A heap is not a tree. Each body is backed along one path, so loads concentrate on single pieces (case e), loop edges still converge at the light-body rate, and omitting rotation leaves the energy claim unproven. The release loop's worst case is one pass per capped leaf, which in a yielding heap could be tens of passes. If the census shows most heap edges cut, this reduces to a fix for chains only.

**Not determined by reading:**
- The shipped soft-run law.
- Real block bounds in play.
- Whether mobilising 9 t in one substep reads as right.
- The pass count.
- Whether `src/debris/crumble.rs` can supply a crush price for a pressed piece; it prices landings only.
