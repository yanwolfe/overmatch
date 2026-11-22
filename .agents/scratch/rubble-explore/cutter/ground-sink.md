# Ground sink

Subject: the game's building model (game state only): why loose rubble cells fall through the
terrain. Every number is MEASURED unless marked DERIVED: release build, governed contact, compute
pool capped at two threads, one run per case and arm (runs differ from one another once the first
piece differs, so pieces are not comparable one for one across arms).

## Causes

Two, independent, both confirmed in a bed without the game and in the game.

1. **The terrain is a surface with nothing under it** (confirmed). The heightfield answers a cell
   only while the cell's shorter way out is up. A flat cell 0.46 m tall on a flat field reports its
   true depth down to 0.20 m under and no contact at all from 0.25 m. `Site` buried a chunk only
   when the ground covered every corner, so a chunk the ground cut through above its middle was
   freed where the terrain does not answer it. On the shipped house on six slopes (2 % to 40 %),
   268 of the 561 chunks the ground cuts were free and unanswered at their standing pose.
2. **parry drops a manifold's deepest point when the terrain is the second shape** (confirmed).
   `contact_manifold_pfm_pfm` (parry3d 0.31.1, a registry crate, not vendored) appends the
   closest-point witness only if the normal is bit-equal to the search direction after the edge
   fix has projected it. Avian's broad phase names the moving body first, and for the second shape
   the projection rebuilds the normal by a rotation there and back, so the equality holds by luck.
   The points that remain sit where the hull crosses the triangle's plane, at zero depth, while the
   lowest vertex is centimetres under; the cell turns about them through the surface. Mechanism
   and numbers: `upstream/parry-normal-constraints-drop-deepest-contact.md`.

## Checkpoint A: what each sinker was

Hall long wall, 16 sinkers:

| sinkers | mechanism | evidence |
|---|---|---|
| 3 | cause 1 | born with the centre 0.085 to 0.311 m under; ground pair from tick 2 with zero manifolds |
| 13 | cause 2 | cross later with a live manifold, upward normal, constraint present, points at 0.000 depth while the collider's lowest bound is 0.1 to 0.5 m under; spin 4 to 17 rad/s |

Bed (200 cells dropped with spin on a 33 by 33 field, five seeds, 512 ticks; sunk of 1000):

| query | flat | 8 % slope |
|---|---|---|
| body first (Avian's order) | 7 | 12 |
| heightfield first | 0 | 4 |
| heightfield first, witness kept | 0 | 0 |
| no edge fix (of 400) | 0 | 0 |

Refuted:

- **A ground pair asleep under an awake piece.** Every sinker's ground pair is in the active list.
- **Our clamp, bounded pricing, compliance.** No ground point is bounded or compliant.
- **No broad-phase pair at birth.** 228 of 228 newborns that never sink show the same: the first
  logged tick precedes the first physics step.
- **parry's "five times deeper" discard.** It removed no point in any pose tested.
- **Another ground collider.** One heightfield is the only ground-layer collider in any pair.
- **Dropping the edge fix as the remedy.** It cures cause 2 in the bed, but without it a cell
  under the surface is pushed further down (downward normals), and it exists for seam bumps.

No library mechanism gives a heightfield an inside: parry 0.31.1 has one heightfield flag
(`FIX_INTERNAL_EDGES`); margins and speculative distance do not supply one (DERIVED from source).

## Fixed

Cause 1, at `Site` (`src/construction/breakage.rs`): the soil holds a chunk whose centroid the
ground covers. On level ground the buried chunks grow: house 100 to 149 of 1,077, hall 22 to 92 of
1,007, brick wall 44 to 66 of 456. The newly held chunks stand 0.06 m over grade at the median on
the house and the wall, 0.20 m on the hall (0.64 m at most).

Residual: on the six slopes 10 of 273 freed cut chunks are still unanswered (centroid 2 to 24 mm
over the ground, more of the height under than over). A rule on vertical reach (buried when the
chunk reaches further under than over) leaves 1 of 275; I kept the centroid because it states a
soil fact and the other states the collider's geometry. Owner's call.

Cause 2 is not fixed. Its root is in parry; reaching it needs one of two choices outside this
lane's brief: vendor parry3d (4.2 MB, a fourth vendored crate) for a six-line fix upstream would
adopt, or carry about 80 lines in the Avian fork that walk the heightfield and re-add the witness
(the lever measured below, saved as a patch). The draft for upstream is committed either way.

## Before and after

| case, arm | sank / below at end | bodies at end | awake, moving at 10 s | crossing s | tick p50 / p95 ms | ticks over 15.6 ms |
|---|---|---|---|---|---|---|
| house, before | 2 / 14 | 420 | 250, 49 | 1.44 | 7.31 / 10.49 | 6 of 641 |
| house, `Site` fix | 6 / 0 | 371 | 132, 32 | 1.52 | 5.07 / 10.20 | 0 |
| house, fix and witness lever | 0 / 0 | 424 | 47, 47 | 1.58 | 7.60 / 13.21 | 11 |
| hall wall, before | 16 / 0 | 199 | 151, 27 | 3.34 | 7.28 / 12.86 | 17 |
| hall wall, `Site` fix | 15 / 0 | 179 | 21, 18 | 4.75 | 7.92 / 12.13 | 18 |
| hall wall, fix and witness lever | 0 / 0 | 236 | 11, 10 | 3.98 | 7.57 / 11.02 | 14 |
| junction, before | 31 / 0 | 401 | 365, 87 | 7.98 | 10.09 / 11.39 | 8 |
| junction, `Site` fix | 11 / 0 | 317 | 279, 65 | never | 9.80 / 11.70 | 5 |
| junction, fix and witness lever | 0 / 0 | 342 | 309, 53 | never | 9.71 / 10.73 | 7 |

The "before" rows reproduce checkpoint 3's (420 / 199 / 403 bodies, 2 / 16 / 31 sank).

- With the `Site` fix no piece is born under the ground in any case (before: 0 / 3 / 14); every
  remaining sinker crosses later in contact, which is cause 2. With the witness lever alone, before
  the fix, the remainder was the reverse: 1 / 2 / 11 sank, all born under.
- The junction's hull no longer crosses. Before, it crossed at 0.42 m/s least forward speed; it
  also stalled with the witness lever alone, so the stall does not need the `Site` fix. An
  observation for the owner's eye, not a target.
- The tick does not move as a whole: the junction's p50 stays over the 9.4 ms mean target (DERIVED
  from 15.6 ms) in every arm.

## Not verified

- Every game figure is one run per arm.
- The witness lever was measured, not reviewed as a patch; its hall-wall residual on real terrain
  is zero in one run.
- No client was watched; nothing was judged by eye.
- The `Site` rule on the shipped map: how many placements change their buried set was not counted.
- `PROTOCOL_REV` stays 108: no replicated type or wire definition changed.
- Backlog rows naming `src/construction/breakage.rs` stay open: destruction-04 (a resting body's
  weight in the part solve) and destruction-10 (ground joints over air) are different facts from
  this one.

## Instrument

`/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-brick/instrument/ground-sink/`:
the probe (`brick_log-ground-sink.rs`, applied over `../instrument-c3.patch`), the bed
(`sink_bed.rs`), the witness lever (`witness-lever-avian.patch`), scripts and per-run summaries
(`notes/`). Raw recordings deleted.
