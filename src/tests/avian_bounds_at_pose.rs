//! Guards for one invariant of the game's server physics (game state only): when the narrow phase
//! runs, every enabled collider's `ColliderAabb` is its shape's bounds at the collider's pose.
//! Who relies on it: the broad phase, and the bounds test before the geometry query
//! (`vendor/avian3d-0.8.0-overmatch`, `OVERMATCH_PATCH.md`), which reads a pair's bounds as the
//! shapes' distance. Stock Avian leaves a body that left the solver in the step that moved it one
//! step behind (`upstream/avian-bounds-of-a-body-leaving-the-solver.md`); the game's
//! `avian_ext::child_poses::refresh_reenabled_bounds` covers a collider enabled again.
//!
//! Speeds, sizes and step counts below are Fixture data.

use avian3d::prelude::*;
use bevy::prelude::*;

use super::avian_child_pose_churn::physics_app;

/// Fixture: the radius of every sphere here (m).
const RADIUS: f32 = 0.5;

fn tolerance(app: &App) -> f32 {
    app.world()
        .resource::<NarrowPhaseConfig>()
        .contact_tolerance
        * app.world().resource::<PhysicsLengthUnit>().0
}

fn drifting_sphere(app: &mut App, velocity: Vec3) -> Entity {
    app.world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vec3::ZERO),
            Collider::sphere(RADIUS),
            LinearVelocity(velocity),
        ))
        .id()
}

fn assert_bounds_at_pose(app: &App, collider: Entity) {
    let world = app.world();
    let at_pose = world.get::<Collider>(collider).unwrap().aabb(
        world.get::<Position>(collider).unwrap().0,
        *world.get::<Rotation>(collider).unwrap(),
        tolerance(app),
    );
    assert_eq!(*world.get::<ColliderAabb>(collider).unwrap(), at_pose);
}

/// A body whose island sleeps in the step that moved it. A mutant that refreshes the bounds of
/// indexed solver bodies alone leaves this one where the previous step put it.
/// Fixture: half Avian's default rest speed, with a rest timer made mature before the step.
#[test]
fn a_body_that_sleeps_in_the_step_that_moved_it_has_bounds_at_its_pose() {
    let mut app = physics_app();
    app.insert_resource(Gravity::ZERO);
    let rest_speed =
        SleepThreshold::default().linear * app.world().resource::<PhysicsLengthUnit>().0;
    let body = drifting_sphere(&mut app, Vec3::X * 0.5 * rest_speed);
    for _ in 0..2 {
        app.update();
    }
    let before = app.world().get::<Position>(body).unwrap().0;
    let mature = app.world().resource::<TimeToSleep>().0;
    app.world_mut().get_mut::<SleepTimer>(body).unwrap().0 = mature;
    app.update();
    assert!(
        app.world().get::<Sleeping>(body).is_some(),
        "fixture: the body sleeps in this step"
    );
    assert_ne!(
        app.world().get::<Position>(body).unwrap().0,
        before,
        "fixture: the step moved it"
    );
    assert_bounds_at_pose(&app, body);
}

#[derive(Resource)]
struct MadeStatic(Entity);

fn make_static(mut commands: Commands, body: Res<MadeStatic>) {
    commands.entity(body.0).insert(RigidBody::Static);
}

/// A body made static between the solver's write-back and the bounds pass, by a system in the
/// physics schedule. It leaves the solver for the static tree, whose
/// colliders the pass must still reach. Fixture: 1 m/s.
#[test]
fn a_body_made_static_in_the_step_that_moved_it_has_bounds_at_its_pose() {
    let mut app = physics_app();
    app.insert_resource(Gravity::ZERO);
    let body = drifting_sphere(&mut app, Vec3::X);
    for _ in 0..2 {
        app.update();
    }
    let before = app.world().get::<Position>(body).unwrap().0;
    app.insert_resource(MadeStatic(body)).add_systems(
        PhysicsSchedule,
        make_static.in_set(PhysicsStepSystems::Sleeping),
    );
    app.update();
    assert!(
        app.world().get::<RigidBody>(body).unwrap().is_static(),
        "fixture: the body is static by the end of the step"
    );
    assert_ne!(
        app.world().get::<Position>(body).unwrap().0,
        before,
        "fixture: the step moved it"
    );
    assert_bounds_at_pose(&app, body);
}

/// A collider enabled again after its pose changed while it was disabled meets what stands at its
/// pose in the very next step. Its bounds stood still while it was disabled and no change tick
/// moves when the marker leaves, so a mutant without the game's mark keeps the old bounds through
/// that step's broad and narrow phase and finds no pair.
/// Fixture: sensor spheres, so no solver response; the move is 10 radii.
#[test]
fn a_collider_enabled_again_meets_what_stands_at_its_pose() {
    let mut app = physics_app();
    crate::headless_test::start_fixed_clock(&mut app);
    app.insert_resource(Gravity::ZERO);
    let there = Vec3::X * 10.0 * RADIUS;
    let fixed = app
        .world_mut()
        .spawn((
            RigidBody::Static,
            Transform::from_translation(there),
            Collider::sphere(RADIUS),
        ))
        .id();
    let moved = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Sensor,
            Transform::IDENTITY,
            Collider::sphere(RADIUS),
        ))
        .id();
    for _ in 0..4 {
        app.update();
    }
    app.world_mut().entity_mut(moved).insert(ColliderDisabled);
    app.world_mut().get_mut::<Position>(moved).unwrap().0 = there;
    for _ in 0..2 {
        app.update();
    }
    app.world_mut()
        .entity_mut(moved)
        .remove::<ColliderDisabled>();
    app.update();
    assert_eq!(
        app.world().get::<Position>(moved).unwrap().0,
        there,
        "fixture: the collider stands at its new pose"
    );
    let graph = app.world().resource::<ContactGraph>();
    assert!(
        graph
            .get(fixed, moved)
            .is_some_and(|(_, pair)| pair.is_touching()),
        "the first step after the collider returns pairs it where it stands"
    );
}
