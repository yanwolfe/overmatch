//! Guards for one invariant of the game's server physics (game state only): when the narrow phase
//! runs, every enabled collider's `ColliderAabb` is its shape's bounds at the collider's pose.
//! Who relies on it: the broad phase, and the bounds test before the geometry query, which reads a
//! pair's bounds as the shapes' distance. Stock Avian breaks it twice; the fork's two fixes and
//! their mechanisms are in `vendor/avian3d-0.8.0-overmatch/src/collider_tree/update.rs`
//! (`OVERMATCH_PATCH.md`; record: `upstream/avian-bounds-of-a-body-leaving-the-solver.md`).
//!
//! Speeds, sizes and step counts below are Fixture data.

use avian3d::collision::collider::ColliderAabbMargin;
use avian3d::prelude::*;
use bevy::ecs::entity_disabling::Disabled;
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;

use super::avian_child_pose_churn::{physics_app, tolerance};

/// Fixture: the radius of every sphere here (m).
const RADIUS: f32 = 0.5;

/// A dynamic sphere at the origin after two settled steps, in zero gravity.
fn sphere(app: &mut App) -> Entity {
    app.insert_resource(Gravity::ZERO);
    let body = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(Vec3::ZERO),
            Collider::sphere(RADIUS),
        ))
        .id();
    for _ in 0..2 {
        app.update();
    }
    body
}

/// Steps once at `velocity` and returns the pose before the step, having checked the step moved
/// the body.
fn step_at(app: &mut App, body: Entity, velocity: Vec3) -> Vec3 {
    let before = app.world().get::<Position>(body).unwrap().0;
    app.world_mut().get_mut::<LinearVelocity>(body).unwrap().0 = velocity;
    app.update();
    assert_ne!(
        app.world().get::<Position>(body).unwrap().0,
        before,
        "fixture: the step moved the body"
    );
    before
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

/// A body whose island sleeps in the step that moved it. A bounds pass that runs once the island
/// has slept finds no solver body and leaves the bounds where the previous step put them.
/// Fixture: half the body's rest speed, with a rest timer made mature before the step.
#[test]
fn a_body_that_sleeps_in_the_step_that_moved_it_has_bounds_at_its_pose() {
    let mut app = physics_app();
    let body = sphere(&mut app);
    let rest_speed = app
        .world()
        .get::<SleepThreshold>(body)
        .copied()
        .unwrap_or_default()
        .linear
        * app.world().resource::<PhysicsLengthUnit>().0;
    let mature = app.world().resource::<TimeToSleep>().0;
    app.world_mut().get_mut::<SleepTimer>(body).unwrap().0 = mature;
    step_at(&mut app, body, Vec3::X * 0.5 * rest_speed);
    assert!(
        app.world().get::<Sleeping>(body).is_some(),
        "fixture: the body sleeps in this step"
    );
    assert_bounds_at_pose(&app, body);
}

/// How a system in the physics schedule takes a body out of the solver after the solver moved it.
#[derive(Resource, Clone, Copy)]
enum Leaves {
    MadeStatic(Entity),
    Disabled(Entity),
}

fn leave(mut commands: Commands, leaves: Res<Leaves>) {
    match *leaves {
        Leaves::MadeStatic(body) => commands.entity(body).insert(RigidBody::Static),
        Leaves::Disabled(body) => commands.entity(body).insert(RigidBodyDisabled),
    };
}

/// The collider a straight-down ray over `at` meets, found through the collider trees. A tripwire
/// on upstream's transfer of a proxy between trees, for the broad phase, which relies on it; it
/// kills no mutant of the fork's patch on its own.
fn hit_below(app: &mut App, at: Vec3) -> Option<Entity> {
    app.world_mut()
        .run_system_once(move |query: SpatialQuery| {
            query
                .cast_ray(
                    at + Vec3::Y * 2.0 * RADIUS,
                    Dir3::NEG_Y,
                    4.0 * RADIUS,
                    true,
                    &SpatialQueryFilter::default(),
                )
                .map(|hit| hit.entity)
        })
        .expect("the ray cast runs")
}

/// Fixture: a step that carries the sphere clear of the enlarged bounds it had, so its tree proxy
/// must move too: twice its radius plus its own enlargement margin.
fn leaves_after_a_step_clear_of_its_bounds(leaves: fn(Entity) -> Leaves) -> (App, Entity) {
    let mut app = physics_app();
    let body = sphere(&mut app);
    let margin = app.world().get::<ColliderAabbMargin>(body).unwrap().0;
    let dt = app
        .world()
        .resource::<Time<Fixed>>()
        .timestep()
        .as_secs_f32();
    app.insert_resource(leaves(body))
        .add_systems(PhysicsSchedule, leave.in_set(PhysicsStepSystems::Sleeping));
    let travel = 2.0 * (RADIUS + margin);
    let before = step_at(&mut app, body, Vec3::X * travel / dt);
    assert_bounds_at_pose(&app, body);
    let after = app.world().get::<Position>(body).unwrap().0;
    assert_eq!(
        (hit_below(&mut app, after), hit_below(&mut app, before)),
        (Some(body), None),
        "the trees hold the collider where it stands"
    );
    (app, body)
}

/// A body made static after the solver moved it, in the same step: it has no solver body by the
/// end of the step, and a bounds pass that runs then leaves its bounds and its tree proxy a step
/// behind.
#[test]
fn a_body_made_static_in_the_step_that_moved_it_has_bounds_at_its_pose() {
    let (app, body) = leaves_after_a_step_clear_of_its_bounds(Leaves::MadeStatic);
    assert!(
        app.world().get::<RigidBody>(body).unwrap().is_static(),
        "fixture: the body is static by the end of the step"
    );
}

/// The same for a body disabled after the solver moved it. The ray cast relies on a
/// `RigidBodyDisabled` body's proxy staying in its tree.
#[test]
fn a_body_disabled_in_the_step_that_moved_it_has_bounds_at_its_pose() {
    let (app, body) = leaves_after_a_step_clear_of_its_bounds(Leaves::Disabled);
    assert!(
        app.world().get::<RigidBodyDisabled>(body).is_some(),
        "fixture: the body is disabled by the end of the step"
    );
}

/// Fixture: a sensor sphere (so no solver response) taken out by `disable`, moved 10 radii onto a
/// static sphere while it is out, and brought back by `enable` two steps later. Asserts the first
/// step after its return pairs the two.
fn meets_what_stands_at_its_pose_after(
    disable: fn(&mut EntityWorldMut),
    enable: fn(&mut EntityWorldMut),
) {
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
    disable(&mut app.world_mut().entity_mut(moved));
    app.world_mut().get_mut::<Position>(moved).unwrap().0 = there;
    for _ in 0..2 {
        app.update();
    }
    enable(&mut app.world_mut().entity_mut(moved));
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

/// A collider enabled again after its pose changed while it was out meets what stands at its pose
/// in the very next step. A bounds pass that examines only a pose or shape changed since the last
/// step keeps the bounds the collider was disabled with through that step's broad and narrow
/// phase, and finds no pair.
#[test]
fn a_collider_enabled_again_meets_what_stands_at_its_pose() {
    meets_what_stands_at_its_pose_after(
        |entity| {
            entity.insert(ColliderDisabled);
        },
        |entity| {
            entity.remove::<ColliderDisabled>();
        },
    );
}

/// The same for an entity that carried Bevy's `Disabled`.
#[test]
fn an_entity_enabled_again_meets_what_stands_at_its_pose() {
    meets_what_stands_at_its_pose_after(
        |entity| {
            entity.insert(Disabled);
        },
        |entity| {
            entity.remove::<Disabled>();
        },
    );
}
