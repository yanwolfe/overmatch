//! What the vendored parry3d's patches (`vendor/parry3d-0.31.1-overmatch/OVERMATCH_PATCH.md`)
//! hold for a convex body. The solver holds every loose body (a rubble cell, a hull) at the points
//! it is given, so a manifold that lacks a point where the bodies meet lets the body through the
//! surface, one with a point nearer than the ground stops the body in the air, and one point
//! under a whole face lets the body turn about it.
//!
//! On the terrain, in the body-first order Avian's broad phase produces:
//! * over triangle edges, an overlap that a triangle reports from its front stays an overlapping
//!   point with the ground's normal once the triangle's cone has corrected the normal. Upstream's
//!   bit-equality gate turns it red;
//! * beside a crease, every reported point lies on the ground: its gap along its normal ends on
//!   the surface. Keeping the closest-point pair whatever the cone did turns it red.
//!
//! A triangle overlapped only towards its back is outside both: the patch note's "Not covered".
//!
//! Between two hulls, face on face, the query reports every corner of either face inside the
//! other, each at the pair's depth, and no two points of a manifold share a feature id pair
//! (Avian carries a point's impulse to the next step by that pair). Clipping a face by its first
//! four corners or its first eight, and writing one id or none for every corner, each turn it
//! red. The narrow phase then keeps four of the points: `avian_contact_pruning` owns that.
//!
//! A hull's face is one plane to the clip: its triangles lie within the merge tolerance of one
//! normal (merging every two neighbours within it turns that red), and a corner over a face of
//! many corners is held at its depth under the face (the plane of the face's first three corners
//! turns that red).

use avian3d::collision::collider::contact_query::contact_manifolds;
use avian3d::parry::shape::{HeightFieldFlags, SharedShape};
use avian3d::prelude::*;
use bevy::prelude::*;

use crate::terrain_grid::{FIXTURE_EXTENT, HeightGrid, TerrainExtent, heightfield_collider};

/// Fixture: how deep the upper hull's face lies in the lower's, m: far under either hull's
/// reach sideways, so the shorter way out is up.
const DEPTH_M: f32 = 0.03;
/// Resolution: agreement between a reported point and the geometry, m: far under `DEPTH_M`,
/// `CLEARANCES_M` and `INSIDE_M`, and far over f32 rounding at these sizes (about 1e-6 m, DERIVED:
/// a few operations on coordinates of metres at 1.2e-7 relative).
const AGREE_M: f32 = 1e-3;
/// Resolution: the cosine over which a manifold's normal is the ground's (2.6 degrees, DERIVED
/// from its arc cosine). What it refuses is a normal out through the side or the back of the
/// surface, tens of degrees off.
const SAME_NORMAL_COS: f32 = 0.999;
/// Fixture: ground rises along x, m per metre: level and a gentle slope.
const RISES: [f32; 2] = [0.0, 0.08];
/// Fixture: radians per step of an offset's direction and of the upper hull's yaw: irrational
/// parts of a turn, so the poses do not repeat.
const TURN_STEPS: [f32; 2] = [2.399_963, 1.324_718];

/// Fixture: a splinter 1.38 m long, 0.22 m wide and 0.1 m thick, m: a flat face of six corners
/// under a ridge that thins to an edge all round, the shape of a split board. Tilted, an end of it
/// is under one triangle while the rest lies over the neighbours.
const SPLINTER: [Vec3; 12] = [
    Vec3::new(0.0635, -0.0282, 0.6800),
    Vec3::new(0.0184, -0.0282, 0.7149),
    Vec3::new(-0.1147, -0.0282, 0.4159),
    Vec3::new(0.0635, 0.0529, 0.2355),
    Vec3::new(0.0164, 0.0272, 0.4127),
    Vec3::new(0.0635, -0.0282, -0.6644),
    Vec3::new(0.0635, 0.0682, -0.2279),
    Vec3::new(0.0610, 0.0673, -0.2337),
    Vec3::new(0.0609, -0.0282, -0.6662),
    Vec3::new(-0.1566, -0.0282, -0.1215),
    Vec3::new(-0.0201, 0.0383, -0.3918),
    Vec3::new(0.0000, 0.0284, -0.4528),
];
/// Fixture: ground sample spacing, m: near the splinter's length, so it lies over several
/// triangles in most places.
const EDGE_SPACING_M: f32 = 1.5;
/// Fixture: how far the splinter's lowest corner is under the surface, m: from a resting
/// contact's overlap to a pressed one's, all under its thickness.
const OVERLAPS_M: [f32; 5] = [0.002, 0.005, 0.01, 0.02, 0.03];
/// Fixture: the splinter's long axis over level, degrees.
const TILTS_DEG: [f32; 6] = [2.0, 5.0, 10.0, 15.0, 30.0, 60.0];
/// Fixture: headings per tilt.
const HEADINGS: usize = 8;
/// Fixture: radians every heading is off the grid lines and the diagonals, so no pose is a
/// symmetric one.
const HEADING_OFFSET: f32 = 0.1;
/// Fixture: places per heading along each side of a grid cell.
const PLACES: usize = 10;
/// Fixture: the first place's offset from a grid corner, m: off the lines and the diagonal.
const PLACE_OFFSET_M: (f32, f32) = (0.07, 0.03);

/// Fixture: a box 0.6 m by 0.4 m by 0.5 m, m, as the hull of its corners: a hull takes parry's
/// general convex path, which the patch is in (a cuboid has its own triangle query).
const BOX: [Vec3; 8] = [
    Vec3::new(-0.3, -0.2, -0.25),
    Vec3::new(0.3, -0.2, -0.25),
    Vec3::new(-0.3, 0.2, -0.25),
    Vec3::new(0.3, 0.2, -0.25),
    Vec3::new(-0.3, -0.2, 0.25),
    Vec3::new(0.3, -0.2, 0.25),
    Vec3::new(-0.3, 0.2, 0.25),
    Vec3::new(0.3, 0.2, 0.25),
];
/// Fixture: creased ground's sample spacing, m: near a shipped map's.
const CREASE_SPACING_M: f32 = 1.465;
/// Fixture: the rise of each facet of the creased ground, m per metre: facets 5.7 degrees apart
/// (DERIVED, twice the arc tangent), a gentle fold of open ground.
const CREASE_RISE: f32 = 0.05;
/// Fixture: sample columns from a ridge to the next valley.
const CREASE_HALF_PERIOD: usize = 2;
/// Fixture: the box's yaw and its tilt onto one edge, radians: off the grid lines, an edge lowest.
const ON_AN_EDGE: (f32, f32) = (0.3, 0.349);
/// Fixture: the box's height over the highest ground under it, m.
const CLEARANCES_M: [f32; 3] = [0.01, 0.03, 0.05];
/// Fixture: how far ahead the query reports points, m: over a step's travel at 5 m/s (0.078 m,
/// DERIVED at 64 Hz), so it reaches the triangles beside the box.
const PREDICTION_M: f32 = 0.1;
/// Fixture: places along x, from one valley over the ridge to the next valley, and as many
/// along z across one cell.
const CREASE_PLACES: usize = 60;

/// Fixture: corners per face of the stacked prisms: over upstream's four, and over the eight a
/// feature holds inline.
const FACE_CORNERS: [usize; 4] = [5, 6, 9, 12];
/// Fixture: a stacked prism's height and its corners' distance from its axis, m, and the first
/// corner's angle about the axis, radians: off the axes.
const PRISM_M: (f32, f32) = (0.3, 0.28);
const PRISM_PHASE: f32 = 0.17;
/// Fixture: how far the upper prism's axis is off the lower's, m: under the least inner radius of
/// the faces (0.23 m, DERIVED for five corners), so each face has a corner inside the other.
const OFFSET_M: f32 = 0.15;
/// Fixture: directions of that offset, so the shared area falls on every corner of a face in turn.
const OFFSETS: usize = 12;
/// Fixture: yaws of the upper prism per direction, so the two faces' corners do not line up.
const YAWS: usize = 5;
/// Resolution: how far inside the other face a corner lies before a point is expected at it, m:
/// far over `AGREE_M`, so a corner on the other face's outline, which rounding puts either side,
/// is not judged.
const INSIDE_M: f32 = 0.02;

/// Fixture: an arc's radius (m), sweep and step (radians), extruded `ARC_WIDTH_M`: the step
/// under parry's merge tolerance, so every two neighbouring facets of the rim are within it, and
/// the sweep many times over it.
const ARC: (f32, f32, f32) = (1.0, 1.2, 0.01);
const ARC_WIDTH_M: f32 = 0.4;
/// Fixture: a drum's radius and height, m, the corners of its rim, and the bend of its roof from
/// rim to rim, radians: under parry's merge tolerance, so the roof is one face, and every three
/// neighbouring corners of that face are near a line and off the roof's plane.
const DRUM_M: (f32, f32) = (0.3, 0.3);
const DRUM_CORNERS: usize = 48;
const ROOF_BEND: f32 = 0.016;
/// Resolution: agreement between a reported depth and the bent roof, m: the roof's sag (1.2 mm,
/// DERIVED: bend x diameter / 8), which no plane is nearer to all of it than; far under the 5 mm
/// and more by which a plane through three neighbouring corners misses the far rim (DERIVED:
/// their plane is the roof's tangent at the rim, bend / 2 off the mean over a diameter).
const ROOF_AGREE_M: f32 = 1.5e-3;
/// Fixture: the box on the roof: its side, m, where its axis stands along the bend, m, and its
/// yaw, radians: every lower corner over the roof, off the drum's axes.
const BOX_ON_ROOF_M: f32 = 0.12;
const ROOF_PLACES_M: [f32; 3] = [-0.15, 0.0, 0.15];
const ROOF_YAW: f32 = 0.3;

/// Ground of `spacing` m between samples, its height a function of the sample's column and x.
fn ground_of(spacing: f32, height: impl Fn(usize, f32) -> f32) -> HeightGrid {
    const SIDE: usize = 9;
    let extent = TerrainExtent {
        world_size_m: (SIDE - 1) as f32 * spacing,
        ..FIXTURE_EXTENT
    };
    let samples: Vec<f32> = (0..SIDE * SIDE)
        .map(|k| {
            height(
                k % SIDE,
                -extent.half_extent() + (k % SIDE) as f32 * spacing,
            )
        })
        .collect();
    HeightGrid::new(samples.into(), SIDE as u32, extent)
}

/// Ground of `spacing` m between samples, rising `rise` m per metre along x.
fn ground(rise: f32, spacing: f32) -> HeightGrid {
    ground_of(spacing, |_, x| rise * x)
}

/// The manifolds of `body` at `at`, turned `turn`, against `ground` at the origin, the body named
/// first: the order Avian's broad phase produces for a moving body on static ground.
fn body_first(
    body: &Collider,
    at: Vec3,
    turn: Quat,
    ground: &Collider,
    prediction: f32,
) -> Vec<ContactManifold> {
    let mut manifolds = Vec::new();
    contact_manifolds(
        body,
        at,
        turn,
        ground,
        Vec3::ZERO,
        Quat::IDENTITY,
        prediction,
        &mut manifolds,
    );
    manifolds
}

/// The least height along `up` of `corners` turned `turn`, m: how far the lowest corner is over
/// a plane of normal `up` through the body's origin.
fn lowest(corners: &[Vec3], turn: Quat, up: Vec3) -> f32 {
    (corners.iter())
        .map(|&corner| (turn * corner).dot(up))
        .fold(f32::MAX, f32::min)
}

/// The deepest overlap any manifold reports, m. Every overlapping point's normal is the ground's,
/// pointing from the first shape to the second.
fn deepest(manifolds: &[ContactManifold], toward_second: Vec3) -> f32 {
    let mut deepest = f32::MIN;
    for manifold in manifolds {
        for point in &manifold.points {
            if point.penetration > 0.0 {
                assert!(
                    manifold.normal.dot(toward_second) > SAME_NORMAL_COS,
                    "an overlapping point's normal is the ground's: {:?}",
                    manifold.normal
                );
            }
            deepest = deepest.max(point.penetration);
        }
    }
    deepest
}

/// `field` with its triangles' normal cones off: what each triangle reports alone.
fn uncorrected(field: &Collider) -> Collider {
    let mut alone = (field.shape_scaled().as_heightfield())
        .expect("the terrain is a heightfield")
        .clone();
    alone.set_flags(HeightFieldFlags::empty());
    Collider::from(SharedShape::new(alone))
}

#[test]
fn an_overlap_a_triangle_reports_from_its_front_survives_the_normal_correction() {
    let body = Collider::convex_hull(&SPLINTER).expect("the fixture is a solid");
    let mut premise_held = false;
    for rise in RISES {
        let grid = ground(rise, EDGE_SPACING_M);
        let field = heightfield_collider(&grid);
        let alone = uncorrected(&field);
        let up = Vec3::new(-rise, 1.0, 0.0).normalize();
        for (tilt, heading) in TILTS_DEG
            .into_iter()
            .flat_map(|tilt| (0..HEADINGS).map(move |heading| (tilt, heading)))
        {
            let turn = Quat::from_rotation_y(
                heading as f32 * core::f32::consts::FRAC_PI_4 + HEADING_OFFSET,
            ) * Quat::from_rotation_x(tilt.to_radians());
            let lowest = lowest(&SPLINTER, turn, up);
            for (overlap, place) in OVERLAPS_M
                .into_iter()
                .flat_map(|overlap| (0..PLACES * PLACES).map(move |place| (overlap, place)))
            {
                let step = EDGE_SPACING_M / PLACES as f32;
                let (x, z) = (
                    PLACE_OFFSET_M.0 + (place % PLACES) as f32 * step,
                    PLACE_OFFSET_M.1 + (place / PLACES) as f32 * step,
                );
                let at = Vec3::new(x, grid.height_at(x, z), z) - up * (lowest + overlap);
                let from_the_front =
                    body_first(&body, at, turn, &alone, 0.0)
                        .iter()
                        .any(|manifold| {
                            manifold.normal.dot(-up) > 0.0
                                && manifold.points.iter().any(|point| point.penetration > 0.0)
                        });
                premise_held |= from_the_front;
                let corrected = body_first(&body, at, turn, &field, 0.0);
                assert!(
                    !from_the_front || deepest(&corrected, -up) > 0.0,
                    "rise {rise}, tilt {tilt}, heading {heading}, {overlap} m under at \
                     ({x}, {z}): no overlapping point in {corrected:?}"
                );
            }
        }
    }
    assert!(
        premise_held,
        "no pose had a triangle reporting an overlap from its front: the fixture tests nothing"
    );
}

/// Bodies sliding across creases rely on this: every point the terrain reports is a point of the
/// ground, so stepping from it along its normal by its gap lands on the surface. A triangle the
/// body is beside, not over, has its closest-point pair running sideways to its edge; read along
/// the normal the cone corrects to, that pair is a point of the triangle's plane tilted beyond
/// its outline, where there is no ground, and a sliding body is stopped against it. Appending
/// the pair whatever the cone did to its direction (the gate removed) turns this red.
#[test]
fn beside_a_crease_every_reported_point_lies_on_the_ground() {
    let body = Collider::convex_hull(&BOX).expect("the fixture is a solid");
    // Ridges and valleys along x, a ridge through the origin.
    let grid = ground_of(CREASE_SPACING_M, |column, _| {
        let from_valley = (column + CREASE_HALF_PERIOD) % (2 * CREASE_HALF_PERIOD);
        let from_ridge = (from_valley as f32 - CREASE_HALF_PERIOD as f32).abs();
        -CREASE_RISE * CREASE_SPACING_M * from_ridge
    });
    let field = heightfield_collider(&grid);
    let turn = Quat::from_rotation_y(ON_AN_EDGE.0) * Quat::from_rotation_x(ON_AN_EDGE.1);
    let reach = (BOX.iter()).fold(0.0_f32, |reach, &corner| reach.max((turn * corner).x.abs()));
    let span = CREASE_HALF_PERIOD as f32 * CREASE_SPACING_M;
    let mut reported = false;
    for (clearance, place) in CLEARANCES_M.into_iter().flat_map(|clearance| {
        (0..CREASE_PLACES * CREASE_PLACES).map(move |place| (clearance, place))
    }) {
        let along = |place: usize| (place as f32 + 0.5) / CREASE_PLACES as f32;
        let x = -span + 2.0 * span * along(place % CREASE_PLACES);
        let z = CREASE_SPACING_M * along(place / CREASE_PLACES);
        // The ground's height varies with x alone and peaks at the ridge, so over the box's
        // footprint it is highest at the footprint's end nearest the ridge, or on the ridge.
        let highest = [x - reach, x + reach]
            .into_iter()
            .chain((x.abs() <= reach).then_some(0.0))
            .map(|under| grid.height_at(under, z))
            .fold(f32::MIN, f32::max);
        let at = Vec3::new(x, highest + clearance - lowest(&BOX, turn, Vec3::Y), z);
        for manifold in body_first(&body, at, turn, &field, PREDICTION_M) {
            for point in &manifold.points {
                reported = true;
                // The point is midway between the body's and the ground's; the normal runs from
                // the body to the ground.
                let foot = point.point - manifold.normal * (0.5 * point.penetration);
                let off = (foot.y - grid.height_at(foot.x, foot.z)).abs();
                assert!(
                    off < AGREE_M,
                    "{clearance} m over the ground at ({x}, {z}): a point {} m away along {:?} \
                     ends {off} m off the surface",
                    -point.penetration,
                    manifold.normal
                );
            }
        }
    }
    assert!(
        reported,
        "no pose reported a point: the fixture tests nothing"
    );
}

/// A right prism on a regular face of `corners` corners ([`PRISM_M`]): the lower face's corners,
/// then the upper's, each in one winding.
fn prism(corners: usize) -> Vec<Vec3> {
    let (height, radius) = PRISM_M;
    [-0.5 * height, 0.5 * height]
        .into_iter()
        .flat_map(|y| {
            (0..corners).map(move |k| {
                let angle = PRISM_PHASE + std::f32::consts::TAU * k as f32 / corners as f32;
                Vec3::new(radius * angle.cos(), y, radius * angle.sin())
            })
        })
        .collect()
}

/// Whether `point` lies more than `INSIDE_M` inside the convex outline `face`, both seen from
/// above.
fn inside(face: &[Vec3], point: Vec3) -> bool {
    let flat = |corner: Vec3| Vec2::new(corner.x, corner.z);
    let sides: Vec<f32> = (0..face.len())
        .map(|k| {
            let (from, to) = (flat(face[k]), flat(face[(k + 1) % face.len()]));
            (to - from).normalize().perp_dot(flat(point) - from)
        })
        .collect();
    sides.iter().all(|&side| side > INSIDE_M) || sides.iter().all(|&side| side < -INSIDE_M)
}

/// A pile at rest relies on this: a piece lying face on face is held at every corner of either
/// face inside the other, each at the pair's depth; with one point under the face the solver
/// turns the piece about it, and the pile never rests. The query reports every such corner, on
/// faces of any corner count; the narrow phase then keeps four (`avian_contact_pruning.rs`).
/// Avian carries each point's impulse to the next step by its pair of feature ids, so every
/// expected corner has both ids and no two points of a manifold share a pair.
#[test]
fn two_hulls_face_on_face_report_every_corner_inside_the_other_face() {
    for corners in FACE_CORNERS {
        let points = prism(corners);
        let body = Collider::convex_hull(&points).expect("the fixture is a solid");
        let (under, over) = points.split_at(corners);
        for offset in 0..OFFSETS {
            for yaw in 0..YAWS {
                let toward = Quat::from_rotation_y(offset as f32 * TURN_STEPS[0]);
                let turn = Quat::from_rotation_y(yaw as f32 * TURN_STEPS[1]);
                let at = toward * Vec3::X * OFFSET_M + Vec3::Y * (PRISM_M.0 - DEPTH_M);
                // The lower prism's upper face and the upper prism's lower face, in the world.
                let lower: Vec<Vec3> = over.to_vec();
                let upper: Vec<Vec3> = under.iter().map(|&corner| at + turn * corner).collect();
                let expected: Vec<Vec3> = (upper.iter().filter(|&&corner| inside(&lower, corner)))
                    .chain(lower.iter().filter(|&&corner| inside(&upper, corner)))
                    .copied()
                    .collect();
                let pose = format!("{corners} corners, offset {offset}, yaw {yaw}");
                assert!(
                    !expected.is_empty(),
                    "{pose}: the fixture has no corner inside the other face"
                );
                let mut manifolds = Vec::new();
                contact_manifolds(
                    &body,
                    Vec3::ZERO,
                    Quat::IDENTITY,
                    &body,
                    at,
                    turn,
                    0.0,
                    &mut manifolds,
                );
                for manifold in &manifolds {
                    let mut pairs: Vec<(u32, u32)> = (manifold.points.iter())
                        .filter(|point| {
                            !point.feature_id1.is_unknown() && !point.feature_id2.is_unknown()
                        })
                        .map(|point| (point.feature_id1.0, point.feature_id2.0))
                        .collect();
                    let known = pairs.len();
                    pairs.sort_unstable();
                    pairs.dedup();
                    assert_eq!(
                        pairs.len(),
                        known,
                        "{pose}: two points share a pair of feature ids: {manifold:?}"
                    );
                }
                for corner in expected {
                    let held = manifolds.iter().flat_map(|m| &m.points).any(|point| {
                        Vec2::new(point.point.x - corner.x, point.point.z - corner.z).length()
                            < AGREE_M
                            && (point.penetration - DEPTH_M).abs() < AGREE_M
                            && !point.feature_id1.is_unknown()
                            && !point.feature_id2.is_unknown()
                    });
                    assert!(
                        held,
                        "{pose}: no point {DEPTH_M} m deep with both feature ids at the corner \
                         {corner:?}: {manifolds:?}"
                    );
                }
            }
        }
    }
}

/// The clipper relies on this: it reads one plane per face, so a face bends by no more than the
/// tolerance parry merges triangles within. Fixture: a rim of facets each within the tolerance
/// of the next, far over it from end to end. Merging every two neighbours within the tolerance
/// makes the rim one face and turns this red.
#[test]
fn a_hulls_face_bends_by_no_more_than_the_merge_tolerance() {
    let (radius, sweep, step) = ARC;
    let points: Vec<Vec3> = (0..=(sweep / step) as usize)
        .flat_map(|k| {
            let (sin, cos) = (k as f32 * step).sin_cos();
            [-0.5 * ARC_WIDTH_M, 0.5 * ARC_WIDTH_M]
                .map(|z| Vec3::new(radius * cos, radius * sin, z))
        })
        .collect();
    let collider = Collider::convex_hull(&points).expect("the fixture is a solid");
    let hull = (collider.shape_scaled().as_convex_polyhedron()).expect("a hull is a polyhedron");
    // Parry's tolerance as an angle: two unit normals are one where their dot is over this.
    let tolerance = (1.0 - avian3d::parry::math::DEFAULT_EPSILON.sqrt()).acos();
    assert!(step < tolerance && sweep > 10.0 * tolerance, "the fixture");
    for face in hull.faces() {
        let first = face.first_vertex_or_edge as usize;
        let corners: Vec<Vec3> = hull.vertices_adj_to_face()
            [first..first + face.num_vertices_or_edges as usize]
            .iter()
            .map(|&vertex| hull.points()[vertex as usize])
            .collect();
        // A face's triangles lie within the tolerance of its first, so a path over the face
        // leaves the first one's plane by under its length times the sine of the tolerance, and
        // no path over a face of the rim is longer than the rim and the width.
        let reach = radius * sweep + ARC_WIDTH_M;
        for corner in &corners {
            let off = (corner - corners[0]).dot(face.normal).abs();
            assert!(
                off <= reach * tolerance.sin(),
                "a face of {} corners has one {off} m off its plane",
                corners.len()
            );
        }
    }
}

/// A piece lying on a face of many corners relies on this: each of its corners over the face is
/// held at its own depth under the face. Fixture: a drum whose roof bends within the merge
/// tolerance and a box set into it. Reading the face's plane off its first three corners turns
/// this red.
#[test]
fn a_corner_over_a_face_of_many_corners_is_held_at_its_depth_under_the_face() {
    let (radius, height) = DRUM_M;
    // The roof's height at `x`: a parabola bending `ROOF_BEND` from rim to rim.
    let roof = |x: f32| 0.5 * height - 0.25 * ROOF_BEND / radius * x * x;
    let drum: Vec<Vec3> = (0..DRUM_CORNERS)
        .flat_map(|k| {
            let angle = PRISM_PHASE + std::f32::consts::TAU * k as f32 / DRUM_CORNERS as f32;
            let (x, z) = (radius * angle.cos(), radius * angle.sin());
            [Vec3::new(x, roof(x), z), Vec3::new(x, -0.5 * height, z)]
        })
        .collect();
    let drum = Collider::convex_hull(&drum).expect("the fixture is a solid");
    let side = 0.5 * BOX_ON_ROOF_M;
    let corners: Vec<Vec3> = (0..8)
        .map(|k| {
            Vec3::new(side, side, side)
                * Vec3::from(std::array::from_fn(|axis| {
                    if k >> axis & 1 == 0 { -1.0 } else { 1.0 }
                }))
        })
        .collect();
    let body = Collider::convex_hull(&corners).expect("the fixture is a solid");
    let turn = Quat::from_rotation_y(ROOF_YAW);
    for place in ROOF_PLACES_M {
        let at = Vec3::new(place, roof(place) + side - DEPTH_M, 0.0);
        let mut manifolds = Vec::new();
        contact_manifolds(
            &drum,
            Vec3::ZERO,
            Quat::IDENTITY,
            &body,
            at,
            turn,
            0.0,
            &mut manifolds,
        );
        for corner in corners.iter().filter(|corner| corner.y < 0.0) {
            let corner = at + turn * *corner;
            let depth = roof(corner.x) - corner.y;
            let held = manifolds.iter().flat_map(|m| &m.points).any(|point| {
                Vec2::new(point.point.x - corner.x, point.point.z - corner.z).length() < AGREE_M
                    && (point.penetration - depth).abs() < ROOF_AGREE_M
            });
            assert!(
                held,
                "the box at {place} m along the bend: no point {depth} m deep at its corner \
                 {corner:?}: {manifolds:?}"
            );
        }
    }
}
