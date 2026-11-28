# A hull's support face is clipped by its first four corners, on a plane read off three

Subject: rigid-body contact generation in a video game's server physics; game state only.

**Target:** published parry3d 0.31.1, `ConvexPolyhedron::local_support_feature` and
`ConvexPolyhedron::from_convex_mesh` in `src/shape/convex_polyhedron.rs`, `PolygonalFeature` and
its `contacts_face_face` in `src/shape/polygonal_feature3d.rs`.
**Status:** LOCAL DRAFT, not filed. Applied in `vendor/parry3d-0.31.1-overmatch` (its
`OVERMATCH_PATCH.md`).

## Mechanism

`PolygonalFeature` holds four vertices, and a hull's support face is cut to fit:

```rust
// TODO: if there are more than 4 vertices, we need to select four vertices that maximize the area.
let num_vertices = face.num_vertices_or_edges.min(4);
```

The four kept are the first four in the face's stored order, so a face of five corners or more is
clipped as the quadrilateral of those four: half of a regular hexagon. Two hulls lying face on
face whose overlap falls on the other corners clip to nothing, and the manifold is the lone
closest-point (GJK or EPA) pair appended after the clip. Between parallel faces that pair is any
point of the overlap, and it moves from step to step.

A solver holds a body at the points it is given. One point under a whole face holds no torque:
the body turns about it into the other until the point moves, then turns back.

## Witness

MEASURED in the game, loose hulls of 4 to 7 corners a face at rest in a pile, 64 Hz. Of 12 pairs
overlapping 20 mm or more whose manifold was one point, 8 had faces along the contact normal
(cosine 0.93 to 1.00) overlapping by 660 to 3790 cm² with every corner counted and by nothing with
the first four of each; the point moved a median 113 to 943 mm a step in 7 of the 8, and the
hull's spin reversed on up to 23 of 23 steps. Each such hull keeps its whole island awake.

`src/tests/parry_manifold_witness.rs`,
`two_hulls_face_on_face_report_every_corner_inside_the_other_face`: two prisms of 5, 6, 9 and 12
corners a face, face on face, 60 poses each. The published crate fails the first pose.

## Fix

The feature keeps every corner of the face, and `contacts_face_face`, whose loops already run to
`num_vertices`, projects as many:

- `PolygonalFeature::{vertices, vids, eids}` become `SmallVec<[_; 8]>`, each at least
  `num_vertices` long: a hull's face as long as it is, the fixed shapes' four slots assigned
  whole. Eight are inline; a face of more is kept on the heap.
- `ConvexPolyhedron::local_support_feature` writes the whole face.
- `contacts_face_face` projects each feature's first `num_vertices` vertices.

The field types are public, so this is a breaking change upstream; `smallvec` is behind `alloc`
there, and the struct is not, which the proposal has to settle (an inline array with a larger
bound keeps the truncation at that bound).

The clip between two faces of n and m corners then returns the overlap polygon's corners, at
most n + m, plus the closest-point pair, where it returned at most 8 and the pair; a caller that
keeps four prunes more. Cost (DERIVED): 2·n·m side tests and n·m edge pairs where the published
crate capped both at 4·4, and for a face over eight corners three allocations for the feature and
one or two in the clip (first or second shape), so nine where both faces are; plus one pass
over each face's corners for its plane per clip, with no allocation (the next section). MEASURED
in the game's shipped hulls: 804 of 77 960 faces have more than eight corners (1.0 %), the most
65; that is a share of faces, and the share of queries that spill is unmeasured, as is the narrow
phase's cost under the clip.

## The face's plane

`contacts_face_face` reads each face's plane off its first three corners:

```rust
let normal2_1 = (vertices2_1[2] - vertices2_1[1]).cross(vertices2_1[0] - vertices2_1[1]);
```

On a face of many corners three neighbours are near a line, and their plane is not the face's.
MEASURED over the game's shipped hulls: a cap of 31 corners whose first three have a sine of
0.0088 between their edges, with a corner 725 mm off their plane. This is in the published crate
as well, since the four corners it keeps start with the same three. The fix reads the mean plane
of all the face's corners: their mean, and Newell's sum about it for the normal.

Witness: `a_corner_over_a_face_of_many_corners_is_held_at_its_depth_under_the_face`, a box set
into the roof of a 48-sided drum; red with the plane of the first three corners.

## Merged faces bend

`from_convex_mesh` merges every two neighbouring triangles whose normals are within
`sqrt(DEFAULT_EPSILON)` of each other (about 1.5 degrees, DERIVED), so a row of triangles, each a
little off the last, becomes one face bent without bound, and the hull keeps the first triangle's
normal for it. MEASURED: faces of 26 to 28 corners along a 10-chord arc bent 95 to 253 mm from
that normal's plane and 110 mm from their own mean plane. The clip reads one plane per face, so a
corner over such a face is held at the wrong depth.

The fix merges a triangle into the face of the first triangle it lies within the tolerance of
(a flood from each unclaimed triangle), so every triangle of a face is within the tolerance of
one normal. After it the furthest corner of any shipped face from its mean plane is 0.23 mm
(MEASURED). The flood is one pass over the hull's triangles and runs where a hull is built only
(DERIVED from the game's callers: at load and at a tank's spawn). Witness:
`a_hulls_face_bends_by_no_more_than_the_merge_tolerance`.

## Not covered

The TODO's own remedy, four corners chosen for area, keeps the quadrilateral: an overlap on the
corners it leaves out still clips to nothing.
