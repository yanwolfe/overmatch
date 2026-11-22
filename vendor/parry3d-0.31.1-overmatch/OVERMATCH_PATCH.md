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
the published crate: 6 (MEASURED, 5 added and 1 removed).

## The closest-point witness under normal constraints (`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`): kept

`contact_manifold_pfm_pfm` adds the closest-point (GJK or EPA) pair to the clipped manifold unless
the normal constraints changed the contact direction. Upstream tests "changed" bit for bit; the
fork tests it within rounding (`local_n1.dot(dir) >= 1.0 - 1.0e-6`; Resolution: an f32 unit vector
rotated into another frame and back differs by a few units in the last place, and a constraint
that turns the normal turns it by far more).

Kept as an upstream bug fix. The projection rewrites a direction it leaves unchanged: for a
constrained second shape it rotates the direction into that shape's frame and back, and among
coplanar neighbours it replaces it by the face normal. The bitwise test then drops the pair, which
is the deepest point of the contact, and the clipped points alone need not reach it: a polyhedron's
face is clipped by its first four corners. A body is then held where its hull crosses the surface
and turns through it. Where the constraints do change the direction the fork is upstream.

Guard: `src/tests/parry_manifold_witness.rs`. Mechanism and measurements:
[local upstream draft](../../upstream/parry-normal-constraints-drop-deepest-contact.md).
