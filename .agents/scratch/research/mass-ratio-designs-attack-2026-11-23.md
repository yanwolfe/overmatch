# Heavy hull through light pieces: attack on designs A, B, C, D and the baseline E

Subject: the game's server physics, rigid-body contact between a hull and loose rubble pieces
(game state only). Paper review, 2026-11-23. Nothing was built or run in the engine; arithmetic
is a throwaway script. Read: the three design files, the facts and literature reports, R1b in
`work-form.md`, `bound.rs`, `crumble.rs`, `fracture.rs`, `materials.ron`, `freeze.rs`, and the
vendored row law.

Labels. **DERIVED** (file:line or formula), **MEASURED** (report named), **ASSUMED** (stated).
`F/` = `vendor/avian3d-0.8.0-overmatch/src/dynamics/solver/`. The ceiling used throughout is the
facts report's `F* = em·[12,920·c + 90,960·min(d, 0.276)]` N with `em = 1.1·m`.

## Verdict

**Measure first, build nothing in the solver.** The evidence in hand says ordinary play sits
under the ceiling (section 2), the agreed refuting fixture is a state play cannot contain
(section 2), and every design that transmits exactly needs a law none of them has (a piece's
own bearing strength, D). If the measurement of section 3 shows the failure window is populated,
the prototype is **C rebuilt in six degrees of freedom with its tree rows taken out of the soft
sweep, capped by D's force bound**; B second, A last. D alone changes nothing for brick pieces
under about 40 kg and is not worth building alone.

## 1. The strongest attack on each

### Shared arithmetic, case (a) (57 t at 3 m/s, 3 kg piece, bound 100 N·s a step, 1,000 J)

All three agree and I reproduce them (DERIVED): ceiling today 127.9 kN = 1,999 N·s a step at no
overlap, so the bound is met in step 1 today; first substep today 19.2 N·s against 16.67 dealt;
under each design the piece is at hull speed in substep 1 (16.67 + 9.0 = 25.67 N·s), the count is
275 J in step 1 and 300 J a step after, and the block parts in step 4. Differences are small:

| | A | B | C | recomputed |
|---|---|---|---|---|
| parts on step | 4 | 4 | 4 | 4 (substep 21) |
| hull loss at parting, J | not stated (speed 2.994 m/s) | 1,020 | 1,027 | 1,050 to 1,077 |
| ceiling quoted | 128 kN | 130 to 210 kN | 3,290 N·s a step | 128 kN at d = 0 |

- B and C set the wall's real work equal to the count. If the first substep is half-counted (25 J,
  as C says) the wall's rows dissipated 25 J more than counted, and the hull's loss is
  `(9 + 21 × 16.67) × 3 = 1,077 J` uncut, about 1,050 J with the last impulse cut. A 3 to 5 %
  slip; no conclusion moves. I did not read the count's convention in `reaction_bound.rs`.
- C's ceiling (3,290 N·s a step) and B's upper end (210 kN) need 27.6 cm of overlap. A 3 kg piece
  at brick density (1,845 kg/m³, MEASURED row, `assets/materials/materials.ron`) is 11.8 cm
  across (DERIVED). The reachable ceiling is 128 to 163 kN. A's figure is the right one.
- Case (d): B gives 5.1 N·s for a 5 kg piece closing at 1.03 m/s; C gives 3.1 N·s, which is
  3 kg × 1.03. C used case (a)'s mass. No conclusion moves.
- Cases (b), (c), (e): every figure I checked reproduces (common speeds 2.55 and 2.59 m/s; soil
  friction 59 and 53 kN; 1,456 N·s a substep = 559 kN; 171,000 N·s = 65.7 MN; 161,000 = 62 MN).

### A, held aggregate

**Attack 1: the lock is rigid in six directions, the evidence for it is one line, and the cut
test reads force only.** State: a 200 kg, 3 m timber piece with one end under a hull's rigid
surface carrying 100 kN, the rest lying free. The end is held by a vertical strut (L1), so the
piece is part of the hull's body (L2). Something pushes the free end sideways with 20 kN. The
cut carries 20 kN tangentially against a cone of `0.6 × 100 = 60 kN`: it passes (L3). The
aggregate is rigid, so 60 kN·m reaches the hull as yaw. A Coulomb point contact resists no such
moment: the piece pivots about its held end. The law (struts and cones) cannot explain a welded
3 m lever. A's own "rigidity break" names it. Fixing it means a moment test per cut over a
contact patch, which is a different and heavier design. **Fatal as written.**

**Attack 2: one retry leaves layers carried.** In A's own case (b) the top layer's cuts fail
(51 N·s needed, 1.8 supplied), the substep is restored and re-solved once. The layer beneath is
now the top, its cuts are not re-tested, and it rides that substep at the aggregate's 2.55 m/s on
bonds that supply 1.8 N·s a step. It unlocks one substep later, keeping the speed. A heap peels
one layer a substep, each layer leaving at the common speed instead of gaining 0.09 m/s a step.
DERIVED from L3's "re-solved once". Only fundamental cuts of one spanning forest are tested, so a
wedge that should slide along a plane crossing several tree bonds is never asked.

**Attack 3: it is one step late where the failure is fastest.** The lock reads stored impulses.
A 0.2 kg brick-density piece is 4.8 cm across; a hull at 3 m/s travels 4.7 cm a step (DERIVED).
The hull is through the piece before the lock exists. A does nothing for the pieces whose
ceilings are lowest.

**Cost.** "One extra substep solve per failing substep" in a flowing heap is every substep:
twice the solve on passes already at 8 to 13 ms (MEASURED, brief). Four fork changes inside the
solver core (alias table, aggregate bodies, a serial list that breaks the colouring's one body
per colour, substep snapshot and restore), none adoptable upstream and none at a game-side seam.

Tolerable: the frozen overlap's release (0.9 m/s on a light piece), the first-step sink.

### B, pinch block

**Attack 1: the cap is a bare count cut-off, and the design cannot work without it.** B says so
(§4, §5). Under a 57 t hull every piece under 4,750 kg is a transmitter, so the block is the
connected heap and the cap always binds in the case the bar names. Where the cap cuts a held
chain (hull, piece 1, piece 2, strong holder; cap after piece 1), the block must treat piece 2
either as immovable or by its own mass. Immovable: the exact solve puts up to 161,000 N·s into
the row between the pieces, and piece 2 (3 kg) receives it with only a swept row behind it.
By its own mass: both pieces reach hull speed and piece 2 meets the holder through today's row;
the failure has moved one shell out and the block bought nothing. B lists the frontier's
stability as not determined. **Fatal for heaps; tolerable for the single pinched piece**, where
the block is 8 rows and exact.

**Attack 2: the transmitter test contradicts the measurement.** B's reason for ratio 12 is that
12 sweeps cannot deliver the push within the step. MEASURED (R1b): ratios 10, 100 and 1,000 are
on time. The facts report shows why: the limit is the light body's mass against the bound, not
the ratio. The test builds blocks at ratios 12 to 1,000 where nothing is late, and it is a
discontinuity in treatment at a mass ratio.

**Attack 3: a new softness law.** `(A + WAW)` is B's own invention, forced by a true finding
(below). It has no source, and its behaviour at a pivot change is not shown.

B's own §7 objection (the piece that cannot give) is not B's flaw: it is every exact design's,
and D answers it. B's second (friction stays iterative) is tolerable under the relaxed bar.

**What B found that the others need.** The ceiling is the fixed point of the soft row law, not
an iteration error: the row sheds `impulse_scale × λ` each biased solve
(`F/contact/normal_part.rs:185-189`, read). Any design that writes a hull-sized impulse into a
row and leaves that row in the sweep inherits this. C does.

### C, backed tree

**Attack 1: the rows shed what the pass wrote, against the light body.** C writes the exact
impulse into the row's accumulator before the biased sweep and leaves the row in the sweep. The
biased row then subtracts 5.77 % of `λ` (dynamic pair) or 2.87 % (static pair), whatever the
masses. State: C's own case (c), rigid surface, 5 kg piece, `λ = 1,456 N·s` on both of the
piece's rows. Biased sweep, hull-to-piece row first (dynamic pairs are solved before a body's
static rows, facts §2): it sheds 84 N·s, the piece gains 16.8 m/s toward the hull; the soil row
then removes that and sheds its own 41.8 N·s, a net 39 N·s into the piece toward the soil:
7.8 m/s. Positions integrate before the relax sweep: the piece moves about 2 cm into the ground
in one substep, and the hull, short 84 N·s, sinks at `0.0577 × g × h = 1.5 mm/s` however long it
stands (DERIVED from the row law and the order; not run). In the bounded case (a) the same term
is 1 N·s and harmless, which is why C's worked case did not show it. C's case (c) claim (a hull
settles on a 5 kg piece) does not follow. **Fatal as written**; fixed by B's one fork flag (the
tree's rows skip the normal loop).

**Attack 2: translation only, with real lever arms.** `K_e` uses `I/m` per body. A hull meets a
held piece at lever arm `r` from its centre with true `K = 1/M + r²/I`. With `M` 57 t and
`I ≈ 207,000 kg·m²` (ASSUMED a 6.3 × 2.0 m box; radius of gyration 1.9 m) the pass's impulse is
`1 + r²M/I` times the one that stops the point: 1.27 at 1 m, 2.1 at 2 m, 3.5 at 3 m. Past the
radius of gyration the factor exceeds 2 and the pair gains energy: at `r` = 3 m and 0.5 m/s
closing, the hull's nose leaves at 1.2 m/s and 10.5 kJ appears in one substep (DERIVED). The row
cannot take it back: its increment is sized by the light body. If instead the pass reads the
closing speed at the centres, it is blind to a hull pitching onto a piece. C's §3 says
non-increase is "not proven"; it is violated. **Fatal as written**; fixed by 6 × 6 apparent
inertia, which the engine exposes (`effective_inv_mass`, `effective_inv_angular_inertia` are
public, `F/solver_body/mod.rs:479,507`, read; so C's "no new patch" claim holds).

**Attack 3: parallel supports.** A hull on N pieces on the ground is a star. Root to leaf, the
first edge takes the whole closing speed and the rest see none: 559 kN on the heaviest piece.
With rotation in the solve this becomes a proper sequential pass over the star; without, it is
one piece carrying a hull. Follows from attack 2.

Tolerable under the relaxed bar: loops answered at today's rate (cut edges), one path per body.
Unknown and decisive for cost: passes per evaluation in a yielding heap.

### D, the piece's own strength

**The bound.** The data holds it: `bulk.blast.compressive_strength` (brick 36.5 MPa, MEASURED
row; wood across the grain 5.6 MPa, along 38.2 MPa) already reaches the laws' material as
`Material::compressive_strength` and `Material::grain` (`src/construction/fracture.rs:44-48`).
No contact law reads it: outside tests only the blast compaction uses it, and the
`Material::bearing` those doc comments link to does not exist (grep, no `fn bearing`). The
bearing law itself is new code.
A piece bears `σ_c × A` over the section the push crosses. A brittle piece gives after the strain
`σ_c / E`, so its work is `σ_c² V / E`: 84 J per kg of brick (DERIVED; the design narrative's
"house cell crushed with a side free, 1.9 MN, 0.45 mm, 0.85 kJ" is the same law). The other
crush number in the tree, `Material::crush` (135 MJ/m³ for brick, DERIVED from
`src/substances.rs:117-121` at factor 36.5), is a round's cavity energy: 220 kJ for a 3 kg
piece, most of the hull's 256 kJ at 3 m/s. It is the wrong law for a pressed piece.

**Against the ceiling** (DERIVED; cube of the stated mass, full-face bearing, hull at 3 m/s;
ceiling from no overlap to overlap of the piece's own thickness):

| piece | brick: bound | ceiling | bound / ceiling | wood across grain: bound | bound / ceiling |
|---|---|---|---|---|---|
| 0.2 kg | 83 kN | 8.5 to 9.5 kN | 9 to 10 | 33 kN | 3 to 4 |
| 3 kg | 505 kN | 128 to 163 kN | 3 to 4 | 199 kN | 1.1 to 1.6 |
| 30 kg | 2.3 MN | 1.3 to 2.0 MN | 1.2 to 1.8 | 0.92 MN | 0.4 to 0.7 |
| 200 kg | 8.3 MN | 8.5 to 14 MN | 0.6 to 1.0 | 3.3 MN | 0.2 to 0.4 |

The bound grows as `m^(2/3)` and the ceiling as `m`; they cross at 41 to 184 kg for brick and
4 to 11 kg for wood across the grain at 3 m/s. At 0.5 m/s the ceiling falls sixfold and the
crossing moves to hundreds of kg. Real pieces bear on corners and edges, which lowers the bound
at first touch, but full-face is what a jammed piece reaches.

**Attack: where the failure is, D never acts.** For brick pieces under about 40 kg the solver
cannot deliver the force the bound tests. D with a force trigger is inert exactly for the small
pieces of R1b. It acts only on large pieces and on wood, where no failure is reported. D alone is
E plus an untriggered law.

**What a spent piece becomes; what the code does today.**
- Split at its seams: yes, for a piece of several chunks (`PendingSplit`, mass kept, every
  fragment a server body; `src/debris/crumble.rs:547-551`). Today only a landing does it, and its
  cracks are capped by the landing's own energy (`crumble.rs:196`), so a steady press cracks
  nothing that costs work. A split along the push keeps the bearing section, so it does not lower
  the bound unless fragments leave sideways.
- Removed as dust: yes (`wear`, `Fate::Dust`, `src/construction/fracture.rs:843-853`; recorded and
  derived by replicas). Mass leaves the simulation, as it does when a round spends a chunk's pool.
- Split one chunk into smaller bodies: **no**. A chunk is the bake's atom.

**A second form, and why I do not recommend it.** Trigger on work instead of force: count the
energy the hull loses in its row on a piece that does not move (`λ·c`, the facts' §3) against the
piece's 84 J/kg, and dust the piece when it is passed. A 3 kg piece at the ceiling takes 384 kW,
so it is spent inside the first substep, and the hull then meets the holder through a hull-sized
row, on time. It works alone, costs almost nothing and names where the energy went. But the
piece gives at a quarter of its strength, and whether it gives depends on the solver's softness
constants, not on its section: it is the accidental crush law made official. It breaks "behaviour
from physical truths". It is the cheapest thing that meets the relaxed bar and it is the owner's
call, not mine.

**What stays broken with D alone (force form):** all of R1b; the late or absent parting of a
block priced between a piece's ceiling and its bound; the hull's loss inside pinched pieces.

**D's true place.** It is the missing cap of every exact design (B §7, C case e): with it a 3 kg
brick between a hull and fixed matter returns at most 505 kN for 252 J, then is dust. And the
built block law has no bearing way either: a push into grounded matter has no way open and the
block is unbounded (`src/construction/fracture/bound.rs:239-240`, `:273-274`;
`breakage/bounded.rs:245`). One law of matter, bearing, would serve both; that keeps it from
being a rule for one kind of object.

### E, do nothing

**Attack.** The R1b mechanism is real and unmeasured in play. Blocks priced above a small
piece's ceiling exist: single-leaf brick face-on 250 to 470 kN, double-leaf 870 to 940 kN, timber
and trees 1.3 to 5 MN (DERIVED table, design narrative, "Losses"), against 128 kN for a 3 kg
piece at 3 m/s and 21 kN at 0.5 m/s. End-on passes lose 4.7 to 7.1 m/s from 10.87 (MEASURED,
fix-pass table) where the narrative's believable range for a 12 m wall is 1 to 4 m/s; my upper
bounds for honest losses (plastic pick-up of about 16 t, bonds, dragging all of it 12 m) sum to
about the 2.5 MJ lost, so the excess is not shown, and not excluded either.

## 2. Where the designs agree, and what follows

1. **Case (a) is on time today.** All three computed it independently and correctly. The
   failure needs a holder priced above the piece's ceiling.
2. **The limit is piece mass against the bound, not mass ratio.** The ceiling is linear in the
   light mass and independent of the heavy one (facts §3, and it reproduces R1b). R1b's sentence
   "a 57 t hull on a 2 to 6 kg piece is 10,000 to 30,000" carries a ratio measured under a
   2,000 kg driver to a hull 28.5 times heavier. The fixture's late and absent arms are 0.2 kg
   and 0.067 kg against a 6.4 kN bound; under a 57 t hull the same pieces fail the same way and
   a 2 to 6 kg piece does not.
3. **The agreed refuter is a state play cannot hold.** The fixture's pieces are 1 m cubes of 0.2
   and 0.067 kg. At brick density they are 4.8 and 3.3 cm. A hull at 4 m/s moves 6.25 cm a step:
   it is past such a piece in one step and on the holder itself, through a hull-sized row.
   "Sinks 36 cm into the piece over eight steps" has no counterpart. All three designs chose the
   0.067 kg arm as their refuting fixture.
4. **What play measured.** The impulse bound dealt on hull-overlapped cells is p50 262 to
   355 N·s a step, 17 to 23 kN (MEASURED, `work-form.md` lag table); the loose pieces that part
   blocks are p50 30 kg (MEASURED, same report), ceiling 213 kN at 0.5 m/s and 1.3 MN at 3 m/s.
   The median is 10 to 60 times under the ceiling. 52 to 94 % of contact-law cells are parted by
   pieces and every wall pass crosses (MEASURED): "a wall still breaks with rubble between" holds
   today. In the hall junction pass the hull rode on pieces with no load on its belts for 2 s
   (MEASURED): high-centring happens today, by which rows is not recorded.
5. **Resting on a heap is under the ceiling too.** A rigid row at rest is a spring of 91 kN/m per
   kg of effective mass; a hull's 559 kN sinks 2 cm on 307 kg of effective mass per layer
   (DERIVED). A belly on a few dozen 20 to 30 kg pieces has that. The "does not settle" row of
   the facts report is one 5 kg piece carrying a whole hull.
6. **The crush observation.** Every exact design makes a piece that cannot give carry 62 to
   66 MN. Today's ceiling costs the hull 15 kJ to pass a pinned 3 kg brick at 3 m/s
   (`128 kN × 11.8 cm`, DERIVED); a true brittle crush costs 0.25 kJ, full compaction 59 kJ. The
   accident sits inside the physical range for few-kg pieces and above it for large ones (324 kJ
   for 30 kg, more than the hull has). So exactness without D is worse than E, and E's error is
   in the piece's survival and in large pieces, not in the order of the small pieces' cost.

Implication: A, B and C are answers to a failure whose size in play nobody has read, tested
against a fixture that overstates it, and each needs D to be safe. The measurement comes first.

## 3. The measurement that decides

One read-only probe on the existing pass set (wall 2, 10, 20 km/h; house; hall long wall; hall
junction; end-on), plus a parked case. The seam exists: `reaction_bound::account` already reads
constraints and `SolverBodies` after the substeps (`src/avian_ext/reaction_bound.rs:435-503`). It
must also read rows between two dynamic bodies, which the fix-pass probe did not.

Per step, for every manifold between a hull collider and a loose piece, record:

1. Hull collider kind (soft run, bump stop, proxy); piece mass `m`; the piece's thickness along
   the normal; `Σem` over the manifold's points; closing speed `c` at step start; deepest overlap
   `d`; the step's normal impulse `J₁`; the row's work, `Σ λ × closing` over the substeps.
2. The piece's speed change along the normal over the step.
3. For each other manifold of that piece whose normal opposes the hull's push by more than
   cos 45° (the built law's own "behind", `bound.rs:18`): holder kind (terrain, standing block,
   loose piece, hull); impulse returned `J₂`; for a standing block its `price.force × dt`, the
   work left, whether a way is open, whether the point was clamped.
4. Derived per row: `s = J₁ / (F*(Σem, c, d) × dt)`, the share of the ceiling in use; and for a
   block holder `p = price.force / F*`; and `q = price.force / (σ_c × A)` with `A` from the
   piece's chunk shapes.

Per pass: the hull's kinetic energy lost, split into hull-to-piece rows with the piece not
moving (item 2 near zero), hull-to-piece rows with the piece moving, hull-to-standing rows,
belts, and the rest.

Parked case: the hull stopped on the heap left by the hall junction and end-on passes for 10 s:
vertical impulse by collider kind and by what is under it (ground through sampled support,
pieces through soft runs, pieces through rigid rows), and the overlap of each rigid row over time.

What decides:

| reading | result | means |
|---|---|---|
| share of hull-to-piece rows with a holder behind | rare | every design unnecessary |
| `p` over pinched rows | under 1 nearly always | E; A, B, C buy timeliness only |
| `p` over 1 with `q` under 1 (block priced between ceiling and the piece's bound), and the block not parting while the hull stays | common | a transmitter is necessary: C rebuilt, or B |
| `p` over 1 with `q` over 1, or holder is terrain or a block with no way | common | the true answer is the piece giving: D necessary, a transmitter optional |
| pinched rows by piece mass | mostly under 1 kg | the hull is past them in a step: E, or D's work form if the owner accepts it |
| hull loss in rows on unmoving pieces, of total loss | a few per cent | E |
| the same | a fifth or more (my line, not a physical one) | the bar's "slows by what the heap weighs and drags" fails: build |
| parked: rigid-row overlap | settles within a piece's thickness | high-centring holds; no design needed for it |
| parked: overlap still growing after 10 s | | C rebuilt or B necessary for support |
| pinched rows by collider kind | mostly soft run | the rigid-row ceiling is not the mechanism; A, B and C aim at the wrong rows |

## 4. Ranking and recommendation

For a test-rig prototype, if the measurement calls for one:

1. **C rebuilt + D (force form).** Six-degree apparent inertia per node, tree rows skipping the
   normal loop (B's single fork flag), an edge's cap the lesser of the holder's dealt bound and
   the piece's `σ_c × A × h`, the piece dusted or split when its work is done. Linear cost, no
   cap, one fork line. Its open risks are loops, load concentration and pass count.
2. **B**, for the single pinched piece only (8-row blocks). Not for heaps: the cap.
3. **A.** Heaviest fork change, late for the smallest pieces, a rigid lock its law does not
   justify.
4. **D alone (force form):** inert where the failure is. **D alone (work form):** works, cheap,
   not a physical truth; owner's call.
5. **E** is where the evidence points today.

**Recommendation: measure first, build nothing.** Record the limit as it is now understood: a
piece passes on at most about `1.1·m·(12,920·c + 90,960·d)` N; the failure is a holder priced
above that; the cost is the hull's loss inside the piece and a late or absent parting.

**The smallest rig that would refute it** (refute "E is enough"), at real density so the
geometry is play's:

- Rig 1: a 57 t body at 0.5 and at 3 m/s presses a 3 kg brick-density cube (11.8 cm) onto a
  block dealt the probe's p90 impulse bound and p90 work. E is wrong if the block has not parted
  by the time the body's face reaches the block, or if the body loses several times the block's
  work plus the piece's pick-up.
- Rig 2: the same body lowered at rest, on its rigid surface, onto three layers of 20 to 30 kg
  brick-density pieces on fixed ground. E is wrong if the overlap does not settle within one
  piece's thickness.

If both pass and the probe's table reads E, the solver stays as it is.

**Before a builder starts on any design:**

- The probe of section 3.
- The shipped soft-run law (the startup log line `envelope law:`), which says whether belts on
  small pieces lag.
- For C rebuilt: its own recurrence check, on the facts report's script, with the row law's
  shed term in it, on a real-density piece and on the parked 5 kg case. That run costs minutes
  and would have shown attack 1.
- For D: the bearing section of real pieces from their chunk shapes (a census of `A` and
  thickness by mass), and the owner's ruling on dust (mass leaves) against split.

## Not determined

- How the built count treats the first substep of a driven piece (the 3 to 5 % in case (a)).
- Whether the hall junction's "rides on pieces" is carried by soft runs or rigid rows.
- The hull's inertia: a box was assumed for C's attack 2; the tank's data would replace it.
- C's attack 1 is hand arithmetic from the row law and the solve order, not run.
- Real pieces' mass distribution beyond the p50 of 30 kg for pieces that part blocks.
