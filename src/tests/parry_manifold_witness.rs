//! A convex body on the terrain keeps its deepest point in the contact manifold, whichever shape
//! the query names first. The solver holds every loose body (a rubble cell, a hull) at the points
//! it is given; a manifold without the deepest one holds a turning body where its hull crosses the
//! surface and lets it turn through. Guards the vendored parry3d's patch
//! (`vendor/parry3d-0.31.1-overmatch/OVERMATCH_PATCH.md`): restoring upstream's bit-equality gate
//! on the witness point turns this red in the body-first order, the order Avian's broad phase
//! produces.

use avian3d::collision::collider::contact_query::contact_manifolds;
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
/// Resolution: agreement between the reported and the geometric depth, m (parry's penetration
/// search converges to about 1e-5 at this size).
const AGREE_M: f32 = 1e-3;
/// Fixture: poses per ground.
const POSES: usize = 96;

/// Fixture: ground sample spacing, m: wide, so the body lies within one triangle.
const SPACING_M: f32 = 6.0;

/// Ground rising `rise` m per metre along x.
fn ground(rise: f32) -> HeightGrid {
    const SIDE: usize = 9;
    let extent = TerrainExtent {
        world_size_m: (SIDE - 1) as f32 * SPACING_M,
        ..FIXTURE_EXTENT
    };
    let samples: Vec<f32> = (0..SIDE * SIDE)
        .map(|k| rise * (-extent.half_extent() + (k % SIDE) as f32 * SPACING_M))
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
                    manifold.normal.dot(toward_second) > 0.999,
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
fn a_body_on_the_terrain_keeps_its_deepest_point_in_either_shape_order() {
    let body = Collider::convex_hull(&BODY).expect("the fixture is a solid");
    for rise in [0.0_f32, 0.08, 0.3] {
        let grid = ground(rise);
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
