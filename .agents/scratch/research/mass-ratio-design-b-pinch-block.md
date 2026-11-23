<!-- Design B, paper only, 2026-11-23; claims DERIVED unless marked -->

# Pinch block: exact normal solve of a heavy body's light neighbourhood

**Decision.** I kept the direction but changed two things. The block's softness is taken on the block's own operator, not row by row. Heavy bodies enter by a 6-DOF Schur complement, so the dense part is only the light cluster. Nothing was built or run; all numbers are hand arithmetic.

**Why the first change is forced (DERIVED).** The row law `Δλ = −ms·em·(vn+b) − is·λ` (`vendor/avian3d-0.8.0-overmatch/src/dynamics/solver/contact/normal_part.rs:185-189`) is a soft constraint with compliance `γ_i = A_ii/a2`, which scales as 1/m of the light piece. Solving hull–piece–holder exactly with that compliance gives `λ ≈ 11·m·(c+b)`: still linear in the light mass. The facts file's ceiling is the fixed point of the system, not an iteration error. An exact solve of the unchanged system arrives sooner at the same wrong answer.

## 1. The law

- **Transmitter.** A body `b` is a transmitter when a body `a` pressing it through an active row has `m_a > N·m_b`, with N the normal sweeps per step (12), or when it is pressed through a chain of transmitters from such a body. Reason: a sweep passes at most about `m_b·c` per visit (facts §3), so N visits cannot deliver `a`'s push within the step. This is two engine masses and the engine's sweep count, not a bare cut-off.
- **Block.** A block is a connected cluster of transmitters plus every active normal row touching one (overlapping or speculative). Dynamic non-transmitters touching it (hulls, big pieces) are roots, each entering by its 6 DOF. Statics are holders.
- **Unknowns and conditions.** The unknowns are `Δλ` on the block's normal rows, with `0 ≤ λ ≤ max_impulse`: can push, cannot pull, and the building model's dealt bound (`/Users/Yan/github/yanwolfe/overmatch/src/avian_ext/reaction_bound.rs:601-604`). The work cut calls the existing `NORMAL_LIMIT` per bounded point at the block's solved closing speed. A cut point becomes a known impulse and the rest is re-solved. Cuts only lower, so this terminates.
- **Softness.**
  - Biased solve: `(A + WAW)·Δλ = −(Jv + b) − W·J·u`, where `A = JM⁻¹Jᵀ`, `W = diag(a2_i^-½)` and `u` is the bodies' velocity from all carried block impulses. An isolated row reduces exactly to today's formula.
  - Compliant soft-run rows keep their absolute diagonal (`/Users/Yan/github/yanwolfe/overmatch/src/avian_ext/compliance.rs:54-71`).
  - Relax solve: `A·Δλ = −Jv`.
  - Plainly: each independent way the block can be squeezed is a spring at the engine's contact frequency on the mass that squeeze must stop.
- **Outside the block:** today's row law, unchanged. I rejected applying the new softness to swept rows: a sweep sheds a hull-sized carried impulse only at the light piece's rate, so a load reversal would launch the hull.
- **Friction:** stays in the coloured sweep, clamped to the block's normal impulse (`contact/mod.rs:378-414`).
- **State:** none new. It uses the accumulated impulses already warm-started. `A` is assembled and factored once per step, because anchors and normal are fixed within a step (`contact/mod.rs:343-347`).
- **Method:** Cholesky on independent rows, with least-index principal pivoting for the box conditions. Dependent rows (four coplanar points) are dropped in `ContactId` order and keep their warm impulse. The reduced matrix is positive definite, so the solution is unique and pivoting terminates.

## 2. Cases

Assumptions throughout: one central point per row, restitution 0, h = 2.604 ms, a2 = 16.33.

**(a) 57 t at 3 m/s, 3 kg piece, wall bound 100 N·s a step, 1,000 J.**
1. The piece is a transmitter (57,000 > 36).
2. Unbounded, the solve wants `0.942 × 57,000 × 3 ≈ 161,000 N·s`. The wall row clamps at 16.67 N·s per substep.
3. The re-solve gives `λ₁ = 16.67 + 3×3 = 25.7`, then 16.67 per substep. The piece is at hull speed from substep 1.
4. Work at the wall is about 275 J in step 1 and 300 J a step after; the budget is reached in step 4.
5. The hull loses 100 N·s a step (1.75 mm/s) and about 1,020 J in total, against 1,000 J counted plus 13.5 J of piece kinetic energy. Hull overlap into the piece is about 0.

Scaling the fixture's first step (35.5 N·s at 0.2 kg and 4 m/s) gives 399 N·s wanted, so today's engine also reaches this bound in step 1 (DERIVED). The design changes outcomes only where the bound exceeds the ceiling: for 3 kg at 3 m/s that is about 130 to 210 kN, or 2,000 to 3,300 N·s a step.

**(b) Heap of 300 pieces on soil.** Every piece is a transmitter (limit 4,750 kg), so the cluster is the connected heap, of order 2,500 rows. That is over the cap (§4).
- Assume 25 touched pieces of mean 30 kg and 2 points per manifold. Shell 1 is then about 200 rows.
- Shell 1 reaches hull speed in substep 1: 2,250 N·s, hull −0.04 m/s.
- Deeper shells answer by sweep. The ratios there are at most 100, which MEASURED on time.
- Plastic limit if all 9 t moved: 2.59 m/s. Soil friction at μ 0.6 is 53 kN.

**(c) Hull at rest on small pieces.** Soft-run row at the fixture law, 50 kN/m per link, 5 cm proud: the block gives `λ = h·k·d = 6.5 N·s` in one solve. Today the gap after one step is 19 % at 2 kg and 84 % at 0.2 kg (facts §5). For a rigid stop or proxy on a 5 kg piece: `J·u = 0` at rest, so the upper row has no sag, the soil row sags `g/ω² ≈ 27 µm`, and it settles. Today it does not settle (facts §5).

**(d) Glancing blow.** One active row and no holder is a one-row block, identical to today. A 5 kg piece at 20° has normal closing 1.03 m/s and takes 5.1 N·s. Next step the gap is positive and the block is empty. λ ≥ 0 and no carried state means nothing sticks.

**(e) Two hulls on one heap.** Both are roots, giving a rank-12 coupling. A chain inside the cap carries `0.942 × (M/2) × 6 ≈ 161,000 N·s` in substep 1 and both hulls stop. Where the frontier falls mid-chain, the middle answers as today.

## 3. Guarantees and breaks

- **Momentum:** exact. Impulses are equal and opposite, applied with the bodies' own inverse masses.
- **Energy:** the pinch dissipation `λ·c` (facts §3) is gone inside the block; the loss is the plastic collision plus counted work.
- **Two-way:** the hull receives exactly what the holder returns, in the same substep.
- **Penetration:** hull-into-piece sink is removed inside the block. The piece-into-bounded-wall overlap of the building model is unchanged.
- **Determinism:** one thread, rows in `ContactId` order, fixed pivot rule. It also removes the hull's overflow-order dependence (facts §2). Cross-architecture f32 reproducibility is not determined.
- **Player sees:** walls part on time through shards; no sinking into rubble; a shard wedged against terrain stops the hull dead.

## 4. Cost

- **Structure:** dense cluster matrix plus rank-6 Woodbury per root.
- **Flops at n = 200 rows:** factor `n³/3 ≈ 2.7e6` once a step; 12 solves at `2n²` is about 1e6; about 50 pivot updates at `n²` is 2e6. Total is about 6e6 flops, roughly 1 to 3 ms on one core, against the 9.4 ms mean. Separate soft-run pieces are 8-row clusters, so 100 of them cost about 6e4.
- **Cap:** unavoidable. Cubic cost against a connected heap needs a fixed row cap, breadth-first from the roots. Beyond it rows answer by today's law. That is a behaviour change by count, confined to rows farthest from any heavy body. The frontier row count should be reported every step.
- **Placement:** a serial stage after the coloured sweep in both the biased and relax solves. Disjoint blocks could run in parallel by body ownership.
- **Fork:** one divergence. A flag on `ContactConstraint` makes `solve` skip its normal loop (`contact/mod.rs:335-377`) while friction still runs. The seam is a new `src/avian_ext/block_solve.rs`, ordered like `deal` and `account` (`reaction_bound.rs:274-277`), which already hold `ConstraintGraph` and `SolverBodies` mutably.

## 5. Owner's rules

| Rule | Verdict |
|---|---|
| Physical truth, not a rule per object kind | Met: one law for any body pair; a 200 kg piece on gravel forms a block too. |
| No tuned number | Not met: the cap is a cost constant. The rest uses only N, ω, ζ, h. |
| No cut-off | Not met beyond the cap, as stated in §4. The transmitter test is a ratio against N. |
| No contact stops answering | Met. |
| No permanent pass-through | Met in the block; unchanged outside it. |
| Two-way and honest | Met: clamped holder rows set what the piece returns. |
| Outcomes not pinned | Met. |
| Simplest true form | Not met: per-row softness on condensed mass alone is simpler but is late and launches on reversal. |

## 6. Refuting fixture

Driver 2,000 kg, piece 0.067 kg, slab unbounded. Run 8 steps at 4 m/s, then reverse the driver's force. The design is wrong if any of these happens:
- the driver gains kinetic energy;
- the piece leaves faster than the driver's speed;
- the bounded variant (100 N·s, 1,000 J) does not count on step 3, as the ratio-10 arm does.

Second rig: two equal pieces in the chain with the cap set to one shell. It shows the frontier's cost.

Before building:
- Add the block to the facts file's Python recurrence and reproduce R1b.
- Census in a real wall pass: block rows per hull by breadth-first depth, the hull's manifolds in overflow, and bound values against the ceiling. If play bounds sit under the ceiling for 2 to 6 kg pieces, the design buys timeliness only.

## 7. Strongest objection

The exact solve makes an indestructible shard a rigid stop. A 3 kg piece between hull and terrain carries 161,000 N·s in one substep, about 62 MN. Today's ceiling was acting as an accidental crush law. Without a strength bound on loose pieces (none exists, facts §5), hulls halt on pebbles. That is true to the model and false to the world.

Second objection: friction stays iterative, so tangential pinches (the hull shearing a piece along the ground) remain late.

Not determined by reading:
- stability of the block softness across the cap frontier;
- whether `/Users/Yan/github/yanwolfe/overmatch/src/debris/crumble.rs` reads pinch presses;
- points per hull–piece manifold;
- the shipped soft-run law;
- tick headroom.
