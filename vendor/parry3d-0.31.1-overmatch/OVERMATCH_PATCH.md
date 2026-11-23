# Overmatch parry3d patch

This directory is the published `parry3d` 0.31.1 crate with the one source change below.

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

The patched site is tagged `OVERMATCH PATCH`. Fork lines by `diff --minimal -ruN` of `src/` against
the published crate: 11 (MEASURED, 7 added and 4 removed).

## The closest-point pair under normal constraints (`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`): kept

`contact_manifold_pfm_pfm` adds the closest-point (GJK or EPA) pair to the clipped manifold.
Upstream drops the pair wherever the normal constraints changed the contact direction, tested bit
for bit; the fork keeps it where the direction is unchanged within rounding
(`local_n1.dot(dir) >= 1.0 - 1.0e-6`) and where the pair overlaps. Its depth is read along the
corrected normal, as the function already reads it: the pair's distance times the cosine of the
turn (DERIVED: the pair lies along the uncorrected direction), so it never reports more overlap
than the pair has, and a pair found overlapping from a triangle's front reports an overlapping
point.

A turned pair with a gap stays dropped, as upstream has it. The same cosine understates a gap, so
a triangle the body is beside reports its plane as a surface beyond its outline. MEASURED with the
gate removed: a box sliding without friction across creases of 5.7 degrees at 5 m/s is stopped
along such a normal mid facet, hops, and ends at 4.58 m/s for 4.94 m/s; with the pair kept only
where it overlaps, 36 rotation-locked slides (level, 8 % slope, creases; 1, 5 and 15 m/s) match
the rounding-only gate to the printed digit, points included.

Kept as an upstream bug fix: without the pair, a pair of shapes the search found overlapping, with
a normal the constraints accepted, can report no overlapping point. Two ways, both MEASURED on the
game's heightfield with the body named first:

- **The projection rewrites a direction it leaves unchanged.** For a constrained second shape it
  rotates the direction into that shape's frame and back, and among coplanar neighbours it replaces
  it by the face normal; the bitwise test then drops the pair. The clipped points alone need not
  reach the deepest point: a polyhedron's face is clipped by its first four corners.
- **The body overlaps a triangle beside its own deepest feature.** A thin body tilted over a
  triangle's edge has its lowest corner under one triangle, which it passes beneath without
  crossing, and overlaps the neighbour with a tilted direction. The cone turns that direction to
  the face normal, clipping along it finds no point (the body's lowest feature lies outside the
  neighbour's outline), and the pair was the only point. A splinter 1.38 m long tilted 10 degrees
  with its lowest corner 2 mm under level ground, a millimetre or two inside the first triangle:
  no manifold.

Upstream's comment says the direction "is no longer valid" for the pair once corrected; the bound
above is the answer the draft gives. Guard: `src/tests/parry_manifold_witness.rs`. Mechanism and
measurements: [local upstream draft](../../upstream/parry-normal-constraints-drop-deepest-contact.md).

Not covered: a triangle has no inside. Where a body's shorter way out of a triangle is through its
back, the projection discards the contact (`project_into_cone` returns `dot_face >= 0.0`), and a
thin body over an edge meets that at overlaps under its own thickness: the draft's "Not covered".
