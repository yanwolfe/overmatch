//! What the vendored parry3d's patch (`vendor/parry3d-0.31.1-overmatch/OVERMATCH_PATCH.md`) holds
//! for a convex body on the terrain, in the body-first order Avian's broad phase produces. The
//! solver holds every loose body (a rubble cell, a hull) at the points it is given, so a manifold
//! that lacks the overlap lets the body through the surface:
//! * within one triangle the manifold keeps the body's deepest point, in either shape order;
//! * over triangle edges, an overlap that a triangle reports from its front stays an overlapping
//!   point with the ground's normal once the triangle's cone has corrected the normal.
//!
//! Dropping the closest-point pair wherever the cone corrected the normal turns the second red;
//! upstream's bit-equality gate turns both red. A triangle overlapped only towards its back is
//! outside both claims: the patch note's last paragraph.

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
/// Resolution: agreement between the reported and the geometric depth, m: far under `DEPTH_M`,
/// and far over the largest disagreement in these poses, 1.3e-6 m (MEASURED, this test).
const AGREE_M: f32 = 1e-3;
/// Resolution: the cosine over which a manifold's normal is the ground's (2.6 degrees, DERIVED
/// from its arc cosine). What it refuses is a normal out through the side or the back of the
/// surface, tens of degrees off.
const SAME_NORMAL_COS: f32 = 0.999;
/// Fixture: poses per ground.
const POSES: usize = 96;

/// Fixture: ground sample spacing, m: wide, so the prism lies within one triangle.
const WIDE_SPACING_M: f32 = 6.0;

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
/// Fixture: places per heading along each side of a grid cell.
const PLACES: usize = 10;

/// Ground of `spacing` m between samples, rising `rise` m per metre along x.
fn ground(rise: f32, spacing: f32) -> HeightGrid {
    const SIDE: usize = 9;
    let extent = TerrainExtent {
        world_size_m: (SIDE - 1) as f32 * spacing,
        ..FIXTURE_EXTENT
    };
    let samples: Vec<f32> = (0..SIDE * SIDE)
        .map(|k| rise * (-extent.half_extent() + (k % SIDE) as f32 * spacing))
        .collect();
    HeightGrid::new(samples.into(), SIDE as u32, extent)
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
    for rise in [0.0_f32, 0.08, 0.3] {
        let grid = ground(rise, WIDE_SPACING_M);
        let field = heightfield_collider(&grid);
        let up = Vec3::new(-rise, 1.0, 0.0).normalize();
        for pose in 0..POSES {
            // Turns spread over the sphere by irrational steps; the place is clear of both diagonals
            // and of the grid lines by more than the body's reach.
            let k = pose as f32;
            let turn = Quat::from_euler(EulerRot::YXZ, k * 2.399_963, k * 0.754_877, k * 1.324_718);
            let (x, z) = (4.4, 0.8);
            // Height over the plane along its normal, with the body's origin on the plane.
            let lowest = (BODY.iter())
                .map(|&corner| (turn * corner).dot(up))
                .fold(f32::MAX, f32::min);
            let at = Vec3::new(x, grid.height_at(x, z), z) - up * (lowest + DEPTH_M);
            let mut manifolds = Vec::new();
            contact_manifolds(
                &body,
                at,
                turn,
                &field,
                Vec3::ZERO,
                Quat::IDENTITY,
                0.0,
                &mut manifolds,
            );
            let body_first = deepest(&manifolds, -up);
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
                (body_first - DEPTH_M).abs() < AGREE_M && (field_first - DEPTH_M).abs() < AGREE_M,
                "rise {rise}, pose {pose}: {DEPTH_M} m under, body first {body_first}, \
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
    for rise in [0.0_f32, 0.08] {
        let grid = ground(rise, EDGE_SPACING_M);
        let field = heightfield_collider(&grid);
        let alone = uncorrected(&field);
        let up = Vec3::new(-rise, 1.0, 0.0).normalize();
        for (tilt, heading) in TILTS_DEG
            .into_iter()
            .flat_map(|tilt| (0..HEADINGS).map(move |heading| (tilt, heading)))
        {
            // Headings a tenth of a radian off the grid lines and the diagonals.
            let turn = Quat::from_rotation_y(heading as f32 * core::f32::consts::FRAC_PI_4 + 0.1)
                * Quat::from_rotation_x(tilt.to_radians());
            let lowest = (SPLINTER.iter())
                .map(|&corner| (turn * corner).dot(up))
                .fold(f32::MAX, f32::min);
            for (overlap, place) in OVERLAPS_M
                .into_iter()
                .flat_map(|overlap| (0..PLACES * PLACES).map(move |place| (overlap, place)))
            {
                let step = EDGE_SPACING_M / PLACES as f32;
                let (x, z) = (
                    0.07 + (place % PLACES) as f32 * step,
                    0.03 + (place / PLACES) as f32 * step,
                );
                let at = Vec3::new(x, grid.height_at(x, z), z) - up * (lowest + overlap);
                let query = |ground: &Collider| {
                    let mut manifolds = Vec::new();
                    contact_manifolds(
                        &body,
                        at,
                        turn,
                        ground,
                        Vec3::ZERO,
                        Quat::IDENTITY,
                        0.0,
                        &mut manifolds,
                    );
                    manifolds
                };
                let from_the_front = query(&alone).iter().any(|manifold| {
                    manifold.normal.dot(-up) > 0.0
                        && manifold.points.iter().any(|point| point.penetration > 0.0)
                });
                let corrected = query(&field);
                assert!(
                    !from_the_front || deepest(&corrected, -up) > 0.0,
                    "rise {rise}, tilt {tilt}, heading {heading}, {overlap} m under at \
                     ({x}, {z}): no overlapping point in {corrected:?}"
                );
            }
        }
    }
}
