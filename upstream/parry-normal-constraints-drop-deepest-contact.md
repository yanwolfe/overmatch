# A manifold under normal constraints loses its deepest point

Subject: rigid-body contact generation in a video game's server physics; game state only.

**Target:** published parry3d 0.31.1, `contact_manifold_pfm_pfm` in
`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`. **Status:** LOCAL DRAFT, not filed;
upstream head unchecked. Source verified against the cached published crate, without network
access. Applied in `vendor/parry3d-0.31.1-overmatch` (its `OVERMATCH_PATCH.md`).

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
| the fix below, shape first | 0 of 1000 | 0 of 1000 |
| published crate, no `FIX_INTERNAL_EDGES` | 0 of 400 | 0 of 400 |

MEASURED, a six-sided prism with its lowest corner 30 mm under one triangle, well inside it, in 96
orientations on ground of slope 0, 8 % and 30 %: the published crate reports no overlapping point
at all in some orientations with the shape first (deepest point 135 mm over the surface); with the
fix both orders report 30 mm in every one (`src/tests/parry_manifold_witness.rs`).

## Shape of a fix

Judge "changed" within rounding instead of bit for bit:

```rust
// If normal constraints changed the GJK direction, it is no longer valid so we cant use it for this additional contact.
&& local_n1.dot(dir) >= 1.0 - 1.0e-6
```

It keeps the rule the comment states. For every caller the result differs from today's only where
the projection left the direction unchanged, which is where today's result already depends on the
order of the shapes and on the last bit of a rotation. Where the constraints turn the normal the
witness is dropped as before. Dropping the gate altogether measured the same (0 of 1000 in both
columns) but changes what a corrected normal reports, which is an argument this form avoids.

## Not covered

A heightfield or mesh triangle has no inside: once a body's shorter way out is through the back
face, the projection discards the contact (`project_into_cone` returns `dot_face >= 0.0`). That is
the documented behaviour of a one-sided surface and a separate question.

Nor a body over a triangle's edge among coplanar neighbours: the search against the triangle that
holds only a sliver of the body can leave sideways, and the pair it returns is not the deepest
vertex. MEASURED, the prism above over a grid line on the 8 % slope: 1.8 mm reported for 30 mm
with the shape first, 30 mm with the heightfield first, with or without the fix. The dropped
bodies above did not sink by it.
