# A manifold under normal constraints loses its deepest point

Subject: rigid-body contact generation in a video game's server physics; game state only.

**Target:** published parry3d 0.31.1, `contact_manifold_pfm_pfm` in
`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`. **Status:** LOCAL DRAFT, not filed.
Upstream `master` still tests `&& local_n1 == dir` at this site (checked 2026-11-23 by reading
upstream head). The fork (`vendor/parry3d-0.31.1-overmatch`, its `OVERMATCH_PATCH.md`) applies
the overlap clause alone and clips every corner of a face
([draft](parry-support-face-first-four-corners.md)); what the rounding clause's absence changes
is in the fork's note. The tables below were measured on the published crate's four-corner clip,
the first witness table with the lesser form of the fix.

## Mechanism

After clipping the two support features, the 3D path appends the closest-point (GJK or EPA)
witness as one more contact, behind this gate:

```rust
// If normal constraints changed the GJK direction, it is no longer valid so we cant use it for this additional contact.
&& local_n1 == dir
```

The comparison is bitwise, and the projection rewrites the normal in two cases that change
nothing geometrically:

- **The constrained shape is second.** `project_local_normal2` rebuilds
  `normal1 = pos12.rotation * (-normal2)`: the direction rotated into the second frame and back,
  equal to `dir` only by luck. `contact_manifolds_heightfield_shape` and
  `contact_manifolds_trimesh_shape` take this path whenever the caller names the other shape
  first (`flipped`).
- **The neighbours are coplanar.** `project_into_cone` returns `*dir = face` when
  `closest_edge == face`, so the witness survives only where the EPA direction is bit-equal to the
  triangle's normal: usual on axis-aligned ground, rare on a slope.

The witness is the deepest point of the pair. In the published crate feature clipping does not
always reach it: `ConvexPolyhedron::local_support_feature` clips a face by its first four corners
(its own TODO; [draft](parry-support-face-first-four-corners.md)), so on a face of five or
more the lowest corner can be one of the others, and the face most aligned with the normal need
not hold the lowest vertex at all (DERIVED). The manifold then carries only the clipped
points, at or above zero depth where the hull crosses the triangle's plane, while the body's lowest
vertex is centimetres under it. A solver holds the body at those points and it turns through the
surface about them. With every corner clipped the first cause is gone and the second stays: for
such a hull, within one triangle, the overlap clause below is still what keeps the deepest point.
No test witnesses it; prisms and boxes cannot be such a hull.

Where the projection does turn the direction, the gate loses the only point there is. A thin body
tilted over a triangle's edge has its lowest corner under triangle A, which it passes beneath
without crossing (the closest pair points out of A's back, and the cone discards it), and overlaps
the neighbour B along a direction tilted off B's normal. The cone turns that direction to the face
normal; clipping along it finds nothing, because the body's lowest feature lies outside B's
outline; and the witness, the one point that says the pair overlaps, is dropped for the turn.

## Witness

MEASURED, one triangle of a flat heightfield with `FIX_INTERNAL_EDGES`, a convex hull (a cell of
a fractured wall, bounds 0.25 m by 0.46 m by 0.58 m) resting on an edge with its lowest vertex
61 mm under the triangle, well inside it: the triangle-first query returns dists `[0.0, 0.134, 0.042, 0.138, -0.061]`, the
shape-first query `[0.0, 0.134, 0.042, 0.138]`. Without constraints both orders return the five.

MEASURED, 200 such hulls dropped with spin onto a 33 by 33 heightfield, five seeds each, 512
steps at 64 Hz, counting hulls that end more than 0.6 m under the surface:

| query | flat | 8 % slope |
|---|---|---|
| published crate, shape first (Avian's order for a moving body on static ground) | 7 of 1000 | 12 of 1000 |
| published crate, heightfield first | 0 of 1000 | 4 of 1000 |
| the gate judged within rounding, shape first | 0 of 1000 | 0 of 1000 |
| published crate, no `FIX_INTERNAL_EDGES` | 0 of 400 | 0 of 400 |

MEASURED, a six-sided prism with its lowest corner 30 mm under one triangle, well inside it, in 96
orientations on ground of slope 0, 8 % and 30 %: the published crate reports no overlapping point
at all in some orientations with the shape first (deepest point 135 mm over the surface); with
either form of the fix both orders report 30 mm in every one
(`src/tests/parry_manifold_witness.rs`).

MEASURED, a splinter (a convex hull 1.38 m by 0.22 m by 0.1 m, a flat face of six corners under a
ridge that thins to an edge) on a heightfield of 1.5 m cells with `FIX_INTERNAL_EDGES`, shape
first, no prediction distance: tilted 2 to 60 degrees with its lowest corner 2 to 30 mm under the
surface, 48,000 poses over headings, places in a cell and slopes of 0 and 8 %. Poses in which no
manifold holds an overlapping point: 3,036 with the gate judged within rounding (351 of them with
no manifold at all), 40 with the gate removed; the 40 are the first paragraph of "Not covered".
The least of them: tilted 10 degrees, lowest corner 2 mm under level ground and a millimetre or
two inside A.

MEASURED, 400 bodies dropped with spin and drift onto a 65 by 65 heightfield of 1.465 m cells
rising 2 % along x and falling 3 % along z, three seeds each, 700 steps at 64 Hz, counting bodies
that end more than 1 m under the surface:

| gate | the splinter | a beam 4.39 m by 0.19 m by 0.16 m |
|---|---|---|
| judged within rounding | 142 of 1200 | 30 of 1200 |
| removed | 13 of 1200 | 4 of 1200 |
| removed, and a triangle overlapped towards its back answers along its cone ("Not covered") | 0 of 1200 | 3 of 1200 |

MEASURED, the same bed rebuilt (other draws), the three forms of the gate:

| gate | the splinter | the beam |
|---|---|---|
| judged within rounding | 148 of 1200 | 34 of 1200 |
| removed | 14 of 1200 | 3 of 1200 |
| within rounding, or the pair overlaps (the fix below) | 22 of 1200 | 5 of 1200 |

## Why the gate cannot simply go

MEASURED, a box (0.6 m by 0.4 m by 0.5 m) and the splinter sliding without friction at 1, 5 and
15 m/s across 1.465 m cells, 0.3 rad off the grid's x, lying flat (free to turn) and held tilted
on an edge and on a corner (rotation locked), 640 steps at 64 Hz at most; on level ground, on an
8 % slope and across creases (facets of plus and minus 5 % meeting every four cells, 5.7 degrees
between them). 54 cases, 36 of them rotation locked.

| | level, slope (24 locked) | creases (12 locked) |
|---|---|---|
| gate removed against within rounding | same speed, height and spin to the printed digit; 8 to 16 % more points | 7 cases end slower, the worst 4.58 m/s for 4.94 m/s from 5 m/s; hops |
| the fix below against within rounding | identical, points included | identical, points included |

The hop, one step of the box held on an edge at 5 m/s: on a facet falling at 5 %, mid facet, a
fourth point appears with normal (-0.021, 0.997, -0.071), 4.4 degrees off both facets, and a gap of
5.9 mm; the box's vertical speed goes from -0.255 to +0.208 m/s in that step and it leaves the
ground, landing at -0.64 m/s seven steps later. The point is a turned pair with a gap. The pair
lies along the uncorrected direction, towards the triangle's edge; read along the corrected normal
its gap is the pair's distance times the cosine of the turn (DERIVED), so a triangle a body is
beside, not over, reports its own plane a few millimetres away, and a solver stops the body
against a plane that has no surface there. Among coplanar neighbours the plane is the neighbour's
own, which is why level ground and the slope show nothing. This is the catching an internal-edge
correction exists to remove, and upstream's gate holds it off.

MEASURED statically (`src/tests/parry_manifold_witness.rs`, the box on an edge 10 to 50 mm over
creased ground, 0.1 m prediction, 10,800 poses): stepping from each reported point along its normal
by its gap ends on the surface within 3.3e-6 m under the fix, and up to 9.4 mm off it with the
gate removed.

A turned pair that overlaps is another matter: the shapes intersect at that pair, on the triangle
itself, and the depth read along the corrected normal is at most the pair's.

Lying flat and free to turn, the splinter under the gate judged within rounding left through the
surface in three of its nine cases (the 8 % slope at 1 m/s, the creases at 5 and 15 m/s); under
the fix it stays on the surface in all nine.

## Shape of a fix

Keep the pair where the direction is unchanged within rounding, or where the pair overlaps:

```rust
if (cfg!(feature = "dim3") || cfg!(feature = "dim2") && manifold.points.is_empty())
    && (local_n1.dot(dir) >= 1.0 - 1.0e-6 || dist < 0.0)
```

`dist` is the pair's own signed distance, as the function already has it; a touching pair counts
as a gap. `1.0e-6` is a resolution: an f32 unit vector rotated into another frame and back differs
from itself by about 1e-7.

The comment at the gate says the direction is no longer valid for this contact once the
constraints changed it. The contact does not use the direction: its depth is
`(p2_1 - p1).dot(local_n1)`, read along the corrected normal, and the pair lies along the
uncorrected one, so the depth is the pair's distance times the cosine of the turn (DERIVED). For an
overlapping pair that never reports more overlap than the pair has, it passes the function's own
"five times deeper" discard for the same reason, and a pair found overlapping from a triangle's
front keeps an overlapping point. For a pair with a gap the same cosine understates the gap, the
section above; those stay dropped.

Two lesser forms. Judging "changed" within rounding alone repairs the two rewrites that change
nothing and leaves the turned direction: the first row of each bed. Removing the gate holds a few
more bodies in the bed (a fast body is met one step sooner) and catches at creases.

The static sweep's 40 poses hold for the fix as for the removed gate (DERIVED: the sweep counts
overlapping points, and the two forms differ only in pairs with a gap).

## Not covered

A heightfield or mesh triangle has no inside: once a body's shorter way out of a triangle is
through its back, the projection discards the contact (`project_into_cone` returns
`dot_face >= 0.0`). That is the documented behaviour of a one-sided surface and a separate
question. A thin body over an edge meets it at overlaps under its own thickness, the 40 poses
above: its end pokes past a grid line, under triangles whose shorter way out is sideways over that
line, and crosses the surface nowhere else.

The prism of the first witness over a grid line on the 8 % slope reported 1.8 mm for 30 mm with
the shape first, the gate bitwise or judged within rounding (MEASURED once); not measured with the
fix.
