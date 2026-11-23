# A manifold under normal constraints loses its deepest point

Subject: rigid-body contact generation in a video game's server physics; game state only.

**Target:** published parry3d 0.31.1, `contact_manifold_pfm_pfm` in
`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`. **Status:** LOCAL DRAFT, not filed.
Upstream `master` still tests `&& local_n1 == dir` at this site (checked 2026-11-23 by reading
upstream head). Applied in `vendor/parry3d-0.31.1-overmatch` (its `OVERMATCH_PATCH.md`).
The first witness table below was measured with the lesser form of the fix.

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

The witness is the deepest point of the pair. Feature clipping does not always reach it:
`ConvexPolyhedron::local_support_feature` clips a face by its first four corners (its own TODO),
so on a face of five or more the lowest corner can be one of the others, and the face most aligned
with the normal need not hold the lowest vertex at all. The manifold then carries only the clipped
points, at or above zero depth where the hull crosses the triangle's plane, while the body's lowest
vertex is centimetres under it. A solver holds the body at those points and it turns through the
surface about them.

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
| removed (the fix below) | 13 of 1200 | 4 of 1200 |
| removed, and a triangle overlapped towards its back answers along its cone ("Not covered") | 0 of 1200 | 3 of 1200 |

## Shape of a fix

Remove the gate: in 3D the witness is always appended.

```rust
if cfg!(feature = "dim3") || cfg!(feature = "dim2") && manifold.points.is_empty() {
```

The comment being removed says the direction is no longer valid for this contact once the
constraints changed it. The contact does not use the direction: its depth is
`(p2_1 - p1).dot(local_n1)`, read along the corrected normal, and the pair lies along the
uncorrected one, so the depth is the pair's distance times the cosine of the turn (DERIVED). It
never reports more overlap than the pair has, it passes the function's own "five times deeper"
discard for the same reason, and a pair found overlapping from a triangle's front keeps an
overlapping point.

The lesser form, judging "changed" within rounding (`local_n1.dot(dir) >= 1.0 - 1.0e-6`), repairs
the two rewrites that change nothing and leaves the turned direction: the first row of the table.

## Not covered

A heightfield or mesh triangle has no inside: once a body's shorter way out of a triangle is
through its back, the projection discards the contact (`project_into_cone` returns
`dot_face >= 0.0`). That is the documented behaviour of a one-sided surface and a separate
question. A thin body over an edge meets it at overlaps under its own thickness, the 40 poses
above: its end pokes past a grid line, under triangles whose shorter way out is sideways over that
line, and crosses the surface nowhere else.

The prism of the first witness over a grid line on the 8 % slope reported 1.8 mm for 30 mm with
the shape first, the gate bitwise or judged within rounding (MEASURED once); not measured with the
gate removed.
