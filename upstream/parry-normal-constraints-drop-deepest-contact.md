# A manifold under normal constraints loses its deepest point

Subject: rigid-body contact generation in a video game's server physics; game state only.

**Target:** published parry3d 0.31.1, `contact_manifold_pfm_pfm` in
`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`. **Status:** LOCAL DRAFT, not filed;
upstream head unchecked. Source verified against the cached published crate, without network
access. The game does not vendor parry, so no patch is applied; the measurements below come from
a temporary lever in the game's Avian fork that re-added the point.

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
`local_support_feature_toward` picks the face most aligned with the normal, and on an irregular
convex polyhedron resting on a vertex or an edge that face need not hold the lowest vertex. The
manifold then carries only the clipped points, at zero depth where the hull crosses the triangle's
plane, while the body's lowest vertex is centimetres under it. A solver holds the body at those
points and it turns through the surface about them.

## Witness

MEASURED, one triangle of a flat heightfield with `FIX_INTERNAL_EDGES`, a convex hull (a cell of
a fractured wall, bounds 0.25 m by 0.46 m by 0.58 m) resting on an edge with its lowest vertex
61 mm under the triangle, well inside it: the triangle-first query returns dists `[0.0, 0.134, 0.042, 0.138, -0.061]`, the
shape-first query `[0.0, 0.134, 0.042, 0.138]`. Without constraints both orders return the five.

MEASURED, 200 such hulls dropped with spin onto a 33 by 33 heightfield, five seeds each, 512
steps at 64 Hz, counting hulls that end more than 0.6 m under the surface:

| query | flat | 8 % slope |
|---|---|---|
| shape first (what Avian's broad phase produces for a moving body on static ground) | 7 of 1000 | 12 of 1000 |
| heightfield first | 0 of 1000 | 4 of 1000 |
| heightfield first, witness re-added when no kept point is as deep | 0 of 1000 | 0 of 1000 |
| no `FIX_INTERNAL_EDGES` | 0 of 400 | 0 of 400 |

## Shape of a fix

Keep the witness whenever the projection kept the contact, measured along the projected normal,
under the bound the function already applies to corrected normals:

```rust
if cfg!(feature = "dim3") || cfg!(feature = "dim2") && manifold.points.is_empty() {
    manifold.points.push(TrackedContact::new(
        p1,
        pos12.inverse_transform_point(p2_1),
        PackedFeatureId::UNKNOWN,
        PackedFeatureId::UNKNOWN,
        (p2_1 - p1).dot(local_n1),
    ));
}
```

The existing `retain` (drop a point more than five times deeper than the unprojected distance)
follows it and bounds what a strongly corrected normal can report. Where the projection changed
nothing, `(p2_1 - p1).dot(local_n1)` is `dist` and the result is today's triangle-first answer.

## Not covered

A heightfield or mesh triangle has no inside: once a body's shorter way out is through the back
face, the projection discards the contact (`project_into_cone` returns `dot_face >= 0.0`). That is
the documented behaviour of a one-sided surface and a separate question.
