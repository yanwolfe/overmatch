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
   `contact_manifold_pfm_pfm` (parry3d 0.31.1) adds the closest-point witness only if the normal
   is bit-equal to the search direction after the edge fix has projected it. Avian's broad phase
   names the moving body first, and for the second shape the projection rebuilds the normal by a
   rotation there and back, so the equality holds by luck. Without the witness the manifold has
   only clipped points, and parry clips a polyhedron's face by its first four corners: on a cell
   with a face of five or more, the lowest corner can be missing. The points that remain sit where
   the hull crosses the triangle's plane, while the lowest vertex is centimetres under; the cell
   turns about them through the surface. Mechanism and numbers:
   `upstream/parry-normal-constraints-drop-deepest-contact.md`.

## Checkpoint A: what each sinker was

Hall long wall, 16 sinkers:

| sinkers | mechanism | evidence |
|---|---|---|
| 3 | cause 1 | born with the centre 0.085 to 0.311 m under; ground pair from tick 2 with zero manifolds |
| 13 | cause 2 | cross later with a live manifold, upward normal, constraint present, points at 0.000 depth while the collider's lowest bound is 0.1 to 0.5 m under; spin 4 to 17 rad/s |

Bed (200 cells dropped with spin on a 33 by 33 field, five seeds, 512 ticks; sunk of 1000):

| query | flat | 8 % slope |
|---|---|---|
| published parry, body first (Avian's order) | 7 | 12 |
| published parry, heightfield first | 0 | 4 |
| the landed fix, body first | 0 | 0 |
| published parry, no edge fix (of 400) | 0 | 0 |

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
chunk reaches further under than over) leaves 1 of 275; the centroid stays because it states a
soil fact and the other states the collider's geometry (coordinator's ruling).

On the shipped map 82 of 717 placements change their buried set (the rest are trees, with no
buried chunk under either rule); buried chunks go from 7,903 to 11,957 of 107,820. 79 placements
hold a structure of their own because the terrain buries other chunks than level ground does.

Cause 2, at its root: parry3d 0.31.1 is vendored (`vendor/parry3d-0.31.1-overmatch`, 348 files,
2.8 MB, the largest 85 kB; benches, examples and integration tests left out) with one changed
condition, 6 fork lines: whether the edge fix changed the direction is judged within rounding
(`local_n1.dot(dir) >= 1.0 - 1.0e-6`), not bit for bit. Where the constraints do turn the normal
the witness is dropped as upstream drops it, so for any caller the result changes only where it
depended on shape order and on the last bit of a rotation. Dropping the gate altogether measured
the same in the bed and was not taken: it changes what a corrected normal reports. Guard:
`src/tests/parry_manifold_witness.rs` (red on the published condition: a six-sided prism 30 mm
under the surface reports no overlapping point, body first). Avian is the only dependent; the
lock's `parry2d`, `parry2d-f64` and `parry3d-f64` are not built.

Determinism: one `[patch]` entry serves every target, so client and server build the same parry.
The changed line is one dot product and a comparison on the scalar glam the tree pins; nothing in
it differs between targets. No replicated type moved; `PROTOCOL_REV` stays 108.

## Before and after

| case, arm | sank / below at end | bodies at end | awake, moving at 10 s | crossing s | tick p50 / p95 ms | ticks over 15.6 ms |
|---|---|---|---|---|---|---|
| house, before | 2 / 14 | 420 | 250, 49 | 1.44 | 7.31 / 10.49 | 6 of 641 |
| house, `Site` fix | 6 / 0 | 371 | 132, 32 | 1.52 | 5.07 / 10.20 | 0 |
| house, both fixes | 1 / 0 | 436 | 235, 48 | 1.48 | 6.49 / 12.45 | 3 |
| hall wall, before | 16 / 0 | 199 | 151, 27 | 3.34 | 7.28 / 12.86 | 17 |
| hall wall, `Site` fix | 15 / 0 | 179 | 21, 18 | 4.75 | 7.92 / 12.13 | 18 |
| hall wall, both fixes | 0 / 0 | 186 | 100, 8 | 3.28 | 6.71 / 9.98 | 5 |
| junction, before | 31 / 0 | 401 | 365, 87 | 7.98 | 10.09 / 11.39 | 8 |
| junction, `Site` fix | 11 / 0 | 317 | 279, 65 | never | 9.80 / 11.70 | 5 |
| junction, both fixes | 1 / 0 | 396 | 341, 132 | never | 10.18 / 11.80 | 3 |
| brick wall, before (checkpoint 3, ten threads) | 2 / 6 | 188 | 48, 48 | not recorded | not recorded | not recorded |
| brick wall, both fixes | 0 / 0 | 193 | 45, 45 | 0.98 | 2.65 / 6.06 | 1 |

The "before" rows of the first three cases reproduce checkpoint 3's (420 / 199 / 403 bodies,
2 / 16 / 31 sank). No pre-fix brick wall run was made at two threads.

- With the `Site` fix alone no piece is born under the ground (before: 0 / 3 / 14); every sinker
  left crosses later in contact, which is cause 2. With both fixes 2 pieces sink in four runs.
- The two that remain are neither cause as traced. The house's is a 4 kg cell resting 35 mm into
  the surface that takes a blow and moves 0.11 m down in one tick, at 11 m/s. The junction's is a
  66 kg cell that meets the ground falling at 8 m/s, 0.125 m a tick. Both end with the centre under
  a surface that has no inside. Not diagnosed further.
- The junction's hull does not cross in any arm after "before", where it crossed at 0.42 m/s least
  forward speed. An observation for the owner's eye, not a target.
- The junction's p50 stays over the 9.4 ms mean target (DERIVED from 15.6 ms) in every arm.

## The tank on the same ground

The track scorecard (`scripts/track-scorecard`, on demand) before and after the parry fix, `Site`
fix in both:

- **Loose heap (8c).** On the published parry the scorecard stops there: a piece of the settled
  heap no longer has a pose. With the fix the section runs: the heap sleeps after 2.9 s and the
  tank crosses it in both cases.
- **Rubble law check (8): passed, now fails**, on one row. "Pile ahead, 0.6 throttle": reaction
  ratio 0.973 (RMS 0.94) before, 0.922 (RMS 0.80) after, against a 0.05 band; the hull's vertical
  jitter is 0.22 m in both. The full-throttle row and the parked layer read 1.000 in both.
- **Slope law check (1): fails in both**, on the same row (sand, above the limit).
- **One-lane walls (4b):** the peak upward normal force on the hull moves from 3.0 to 4.0 kN
  (left lane) and from 1.2 to 3.5 kN (right lane); the last two seconds' means are unchanged.
- Moved in the last digits: the slope forces (under 0.4 %) and two step times (0.016 s).
  Acceleration, top speed, trenches and ride are unchanged to the printed digits.

The lib suite outside the on-demand tests passes with both fixes (2,710 tests, `track::` and the
54 `headless_test` among them).

## Not verified

- Every game figure is one run per arm.
- No client was watched; nothing was judged by eye.
- A body over a triangle's edge among coplanar neighbours can still lose its deepest point in
  either shape order (MEASURED once: 1.8 mm reported for 30 mm, with or without the fix); the
  dropped bodies in the bed did not sink by it. The guard states the invariant inside a triangle.
- Why the scorecard's rubble row moved, and whether the heap piece lost on the published parry
  went under the ground, were not traced.
- Upstream parry's head was not checked for the same fix.
- The six lenses were not run over the diff.
- Backlog rows naming `src/construction/breakage.rs` stay open: destruction-04 (a resting body's
  weight in the part solve) and destruction-10 (ground joints over air) are different facts from
  this one.

## Instrument

`/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-brick/instrument/ground-sink/`:
the probe (`brick_log-ground-sink.rs`, applied over `../instrument-c3.patch`), the bed
(`sink_bed.rs`), scripts and per-run summaries and both scorecards (`notes/`). Raw recordings and
the witness lever deleted.
