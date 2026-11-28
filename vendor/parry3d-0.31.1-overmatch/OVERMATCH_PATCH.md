# Overmatch parry3d patch

This directory is the published `parry3d` 0.31.1 crate with the source changes below.

Produced from `https://static.crates.io/crates/parry3d/parry3d-0.31.1.crate`, whose sha256
`7b626edc0b497dba4bdf6e1e0797895294ed9f61dac2bd28cb5d34d37ad2ca7e` matches the crates.io index's
`cksum`, by `tar -xzf parry3d-0.31.1.crate -C vendor/parry3d-0.31.1-overmatch --strip-components=1
--exclude='*/Cargo.lock' --exclude='*/.cargo_vcs_info.json' --exclude='parry3d-0.31.1/benches'
--exclude='parry3d-0.31.1/examples' --exclude='parry3d-0.31.1/tests'`. The published lockfile and
VCS record are omitted; neither is read for a path dependency. The benches, examples and
integration tests (85 files, 0.6 MB) are omitted: a dependency's are never built, and cargo does
not look for the files `Cargo.toml` still lists for them. `Cargo.toml` is the published one.

Subject: the game's server physics (contact generation between rigid bodies). Game state only.

Avian is the only dependent (`cargo tree -i parry3d`). The lock's `parry2d`, `parry2d-f64` and
`parry3d-f64` are optional dependencies of crates the game does not enable; none is built.

Each patched file tags its changes `OVERMATCH PATCH` where they begin; the `Corners::from_slice`
sites that follow from the field types carry no tag of their own. Fork lines by
`diff --minimal -ruN` of `src/` against the published crate: 239 (MEASURED), 8 of them the first
change and 32 the face merge.

## The closest-point pair under normal constraints (`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`): kept where it overlaps

`contact_manifold_pfm_pfm` adds the closest-point (GJK or EPA) pair to the clipped manifold, and
upstream drops it wherever the normal constraints changed the contact direction, tested bit for
bit. The fork also keeps it where **the pair overlaps**, `dist < 0.0` on the pair's own signed
distance (touching counts as a gap): the contact's depth is read along the corrected normal and
never uses the dropped direction, and an overlapping pair that reports no point is a defect.

A turned pair with a gap stays dropped, as upstream has it: kept, a sliding body is stopped at
creases.

What it covers with every corner clipped (below): an overlap that a triangle reports from its
front, once the triangle's cone has turned the normal; and, within one triangle, a hull whose
face most aligned with the normal does not hold its lowest vertex (DERIVED; an irregular fracture
cell can be so), whose deepest point only this clause keeps. No test witnesses the second: the
prisms and boxes of the guard cannot be such a hull.

The fork once also kept a pair whose direction was unchanged within rounding. Without it, a pair
with a gap is kept only where the direction is bit-equal, so speculative contact on the terrain
comes from the face clip alone, and for such a hull contact appears one step later, as an overlap
of at most one step's closing travel.

Guard: `src/tests/parry_manifold_witness.rs`. Mechanism and measurements:
[local upstream draft](../../upstream/parry-normal-constraints-drop-deepest-contact.md).

Not covered: a triangle has no inside. Where a body's shorter way out of a triangle is through its
back, the projection discards the contact (`project_into_cone` returns `dot_face >= 0.0`), and a
thin body over an edge meets that at overlaps under its own thickness: the draft's "Not covered".

## A hull's support face (`src/shape/polygonal_feature3d.rs`, `src/shape/convex_polyhedron.rs`): every corner, one plane

Upstream cuts a hull's support face to its first four corners, so two hulls face on face can clip
to nothing and rest on one moving point:
[local upstream draft](../../upstream/parry-support-face-first-four-corners.md), which owns the
mechanism, the two faults below it, the measurements and the cost. Its plane and merge fixes are
applied at `mean_plane` and `ConvexPolyhedron::from_convex_mesh`. Fork-specific:

- **`Corners<T>`**, a `SmallVec` of eight inline, holds a feature's `vertices`, `vids` and `eids`.
  `Resolution`: the inline size alone; a face of more corners is kept whole on the heap.
- **The fixed shapes** (`cuboid.rs`, `polygonal_feature_map.rs`) assign each list whole, so
  `len() >= num_vertices` holds for every feature; padding beyond `num_vertices` is not the
  feature's.
- **`smallvec` is behind parry's `alloc` feature**, which Avian enables; the fork does not build
  without it.

Every shipped hull (MEASURED, one boot of the shipped map with its ten constructions: 10 610
hulls, 77 960 faces):

| Hulls | Faces | Over eight corners | Most corners |
|---|---|---|---|
| construction elements | 33 666 | 635 (1.9 %) | 12 |
| fracture cells | 43 446 | 129 (0.3 %) | 11 |
| track stops | 152 | 12 | 31 |
| track side plates | 478 | 16 | 65 |
| track soft runs | 152 | 8 | 36 |
| tank primitives | 66 | 4 | 16 |

The inline size stays eight: the faces over it are 1.0 % of all faces (the share of queries that
spill is unmeasured), and on the tracks they are the wheel-side caps, which face sideways, not
the rim faces that meet the ground each tick (DERIVED from the colliders' build). The hosted
narrow phase's cost under the clip is not measured.

What else the merge moved: a hull's faces are what `to_trimesh` fans, which parry's hull mass
properties read (a rubble chunk's mass, `src/construction/breakage.rs`), and the track's belt
support on rubble (`src/track/rubble.rs`) and Avian's trimesh builder with them. Where an old
face bent, its fan triangles were chords inside the hull, so mass and support surface were
slightly under the true hull and now match it (DERIVED). How many rubble hulls changed their
face grouping, and by how much mass, is unmeasured.

Guards, in `src/tests/parry_manifold_witness.rs`:
`two_hulls_face_on_face_report_every_corner_inside_the_other_face`,
`a_corner_over_a_face_of_many_corners_is_held_at_its_depth_under_the_face`,
`a_hulls_face_bends_by_no_more_than_the_merge_tolerance`.
