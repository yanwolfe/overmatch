//! What the vendored parry3d's patch (`vendor/parry3d-0.31.1-overmatch/OVERMATCH_PATCH.md`) holds
//! for a convex body on the terrain, in the body-first order Avian's broad phase produces. The
//! solver holds every loose body (a rubble cell, a hull) at the points it is given, so a manifold
//! that lacks the overlap lets the body through the surface, and one with a point nearer than the
//! ground stops the body in the air:
//! * within one triangle the manifold keeps the body's deepest point, in either shape order;
//! * over triangle edges, an overlap that a triangle reports from its front stays an overlapping
//!   point with the ground's normal once the triangle's cone has corrected the normal;
//! * beside a crease, every reported point lies on the ground: its gap along its normal ends on
//!   the surface.
//!
//! Dropping the closest-point pair wherever the cone corrected the normal turns the second red;
//! upstream's bit-equality gate turns the first two red; keeping the pair whatever the cone did
//! turns the third red. A triangle overlapped only towards its back is outside all three: the
//! patch note's last paragraph.

use avian3d::collision::collider::contact_query::contact_manifolds;
use avian3d::parry::shape::{HeightFieldFlags, SharedShape};
use avian3d::prelude::*;
use bevy::prelude::*;

use crate::terrain_grid::{FIXTURE_EXTENT, HeightGrid, TerrainExtent, heightfield_collider};

/// Fixture: a six-sided prism 0.3 m tall and 0.56 m across, m. Its two six-cornered faces are the
/// case: parry clips a face by its first four corners, so the closest-point witness is the only
/// point that reaches a lowest corner among the other two.
const BODY: [Vec3; 12] = [
    Vec3::new(0.2757, -0.15, 0.0486),
    Vec3::new(0.0958, -0.15, 0.2631),
    Vec3::new(-0.1800, -0.15, 0.2145),
    Vec3::new(-0.2757, -0.15, -0.0486),
    Vec3::new(-0.0958, -0.15, -0.2631),
    Vec3::new(0.1800, -0.15, -0.2145),
    Vec3::new(0.2757, 0.15, 0.0486),
    Vec3::new(0.0958, 0.15, 0.2631),
    Vec3::new(-0.1800, 0.15, 0.2145),
    Vec3::new(-0.2757, 0.15, -0.0486),
    Vec3::new(-0.0958, 0.15, -0.2631),
    Vec3::new(0.1800, 0.15, -0.2145),
];
/// Fixture: how far the body's lowest vertex is under the surface, m: shallow, so its shorter way
/// out is up in every pose.
const DEPTH_M: f32 = 0.03;
/// Resolution: agreement between a reported point and the geometry, m: far under `DEPTH_M` and
/// `CLEARANCES_M`, and far over the largest disagreement in these poses, each test's own (MEASURED:
/// 1.3e-6 m in depth, 3.3e-6 m off the surface).
const AGREE_M: f32 = 1e-3;
/// Resolution: the cosine over which a manifold's normal is the ground's (2.6 degrees, DERIVED
/// from its arc cosine). What it refuses is a normal out through the side or the back of the
/// surface, tens of degrees off.
const SAME_NORMAL_COS: f32 = 0.999;
/// Fixture: poses per ground.
const POSES: usize = 96;
/// Fixture: ground rises along x, m per metre: level, a gentle slope and a steep one.
const RISES: [f32; 3] = [0.0, 0.08, 0.3];
/// Fixture: radians of yaw, pitch and roll per pose: irrational steps, so the poses spread over
/// the sphere without repeating.
const POSE_STEPS: [f32; 3] = [2.399_963, 0.754_877, 1.324_718];

/// Fixture: ground sample spacing, m: wide, so the prism lies within one triangle.
const WIDE_SPACING_M: f32 = 6.0;
/// Fixture: where the prism sits, m: clear of the cell's diagonal and of the grid lines by more
/// than the prism's reach.
const WITHIN_A_TRIANGLE: (f32, f32) = (4.4, 0.8);

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

#[test]
fn a_body_within_a_triangle_of_the_terrain_keeps_its_deepest_point_in_either_shape_order() {
    let body = Collider::convex_hull(&BODY).expect("the fixture is a solid");
    for rise in RISES {
        let grid = ground(rise, WIDE_SPACING_M);
        let field = heightfield_collider(&grid);
        let up = Vec3::new(-rise, 1.0, 0.0).normalize();
        for pose in 0..POSES {
            let k = pose as f32;
            let [yaw, pitch, roll] = POSE_STEPS.map(|step| k * step);
            let turn = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);
            let (x, z) = WITHIN_A_TRIANGLE;
            let at =
                Vec3::new(x, grid.height_at(x, z), z) - up * (lowest(&BODY, turn, up) + DEPTH_M);
            let body_named_first = deepest(&body_first(&body, at, turn, &field, 0.0), -up);
            let mut manifolds = Vec::new();
            contact_manifolds(
                &field,
                Vec3::ZERO,
                Quat::IDENTITY,
                &body,
                at,
                turn,
                0.0,
                &mut manifolds,
            );
            let field_first = deepest(&manifolds, up);
            assert!(
                (body_named_first - DEPTH_M).abs() < AGREE_M
                    && (field_first - DEPTH_M).abs() < AGREE_M,
                "rise {rise}, pose {pose}: {DEPTH_M} m under, body first {body_named_first}, \
                 heightfield first {field_first}"
            );
        }
    }
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
    for rise in [RISES[0], RISES[1]] {
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
