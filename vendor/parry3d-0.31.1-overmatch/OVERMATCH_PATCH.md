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
the published crate: 11 (MEASURED, 9 added and 2 removed).

## The closest-point pair under normal constraints (`src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`): kept in two cases

`contact_manifold_pfm_pfm` adds the closest-point (GJK or EPA) pair to the clipped manifold, and
upstream drops it wherever the normal constraints changed the contact direction, tested bit for
bit. The fork keeps it where:

- **the direction is unchanged within rounding**, `local_n1.dot(dir) >= 1.0 - 1.0e-6`. Upstream's
  rule, judged as a float: the projection rewrites a direction it leaves unchanged. `Resolution`:
  an f32 unit vector rotated into another frame and back differs from itself by about 1e-7.
- **the pair overlaps**, `dist < 0.0` on the pair's own signed distance (touching counts as a
  gap). A divergence from upstream's rule, resting on the draft's argument: the contact's depth is
  read along the corrected normal and never uses the dropped direction, and an overlapping pair
  that reports no point is a defect.

A turned pair with a gap stays dropped, as upstream has it: kept, a sliding body is stopped at
creases.

Guard: `src/tests/parry_manifold_witness.rs`. Mechanism and measurements:
[local upstream draft](../../upstream/parry-normal-constraints-drop-deepest-contact.md).

Not covered: a triangle has no inside. Where a body's shorter way out of a triangle is through its
back, the projection discards the contact (`project_into_cone` returns `dot_face >= 0.0`), and a
thin body over an edge meets that at overlaps under its own thickness: the draft's "Not covered".
