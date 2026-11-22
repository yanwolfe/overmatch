//! The contact law wired into the step, on a generic wall fixture (game state only).

use avian3d::prelude::*;

use super::super::tests::composed_kind;
use super::*;
use crate::construction::tests::wall_construction;

/// Fixture: a brick leaf 6 m long, 2.4 m high and 0.25 m thick in 1 m by 0.6 m cells, and a
/// rigid box 2 m wide, 1.2 m high and 2 m long sent at its middle along +z from 0.175 m short.
const WALL: Vec3 = Vec3::new(6.0, 2.4, 0.25);
const BOX: Vec3 = Vec3::new(2.0, 1.2, 2.0);
const START: Vec3 = Vec3::new(0.0, 1.0, -1.3);

/// The box of `mass` kg sent at `speed` m/s, after 2.5 s: where its centre is along z (m), its
/// speed along z (m/s) and the cells the wall lost.
fn sent(mass: f32, speed: f32) -> (f32, f32, usize) {
    let kind = wall_construction(WALL, [6, 4, 1], "Brick");
    let (mut app, _) = composed_kind(false, &kind);
    for _ in 0..3 {
        app.update();
    }
    let body = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(START),
            Rotation::default(),
            Collider::cuboid(BOX.x, BOX.y, BOX.z),
            Mass(mass),
            NoAutoMass,
            piece_layers(),
            LinearVelocity(Vec3::Z * speed),
            SleepingDisabled,
        ))
        .id();
    let mut freed = 0;
    for _ in 0..160 {
        app.update();
        freed += app.world().resource::<BoundAccount>().freed.len();
    }
    (
        app.world().get::<Position>(body).unwrap().0.z,
        app.world().get::<LinearVelocity>(body).unwrap().0.z,
        freed,
    )
}

/// Invariant: the law reaches the solver and the release. A body whose momentum the leaf's bonds
/// cannot take crosses it still moving, and the cells in its way leave; a body they can stop
/// does not pass the same leaf. Every body that meets standing matter relies on it. Mutants: no
/// bound handed to the solver (the heavy box stops dead on a rigid wall), a block bounded at
/// nothing or freed by any touch (the light box passes).
#[test]
fn a_leaf_gives_to_a_body_its_bonds_cannot_stop_and_stops_one_they_can() {
    // Fixture: 57 t and 20 kg, each at 4.5 m/s.
    let (reached, speed, freed) = sent(57_000.0, 4.5);
    assert!(
        reached - BOX.z / 2.0 > WALL.z && speed > 0.0 && freed > 0,
        "the heavy box crosses: at {reached} m, {speed} m/s, {freed} cells freed"
    );
    let (reached, ..) = sent(20.0, 4.5);
    assert!(
        reached + BOX.z / 2.0 < WALL.z,
        "the light box does not pass: at {reached} m"
    );
}

/// Fixture: one force on a belt's body, staged once.
#[derive(Resource)]
struct Once(Option<crate::contact_transfer::StagedForce>);

fn press_once(
    mut once: ResMut<Once>,
    mut transfers: ResMut<ContactTransfers>,
    mut bodies: Query<Forces>,
) {
    if let Some(force) = once.0.take() {
        (bodies.get_mut(force.body).unwrap()).apply_force_at_point(force.force, force.point);
        transfers.stage(force);
    }
}

/// Invariant: a belt takes from standing matter no more than its bonds return, and the cells it
/// presses past that leave. `track::rubble::record_static_contacts` stages a belt's whole force
/// on fixed construction and relies on this to cap it. Mutants: [`bound_belts`] unmounted (the
/// body keeps the whole force and nothing leaves), the force taken back with its sign flipped
/// (the body gains more than the force), the pressed cells never queued (nothing leaves).
/// Fixture: a 1 t body clear of the wall and at rest, pushed off the middle of the wall's face
/// for one tick with four times what the cell there returns.
#[test]
fn a_belt_keeps_what_standing_matter_returns_and_the_rest_leaves() {
    let kind = wall_construction(WALL, [6, 4, 1], "Brick");
    let (mut app, root) = composed_kind(false, &kind);
    app.insert_resource(Once(None)).add_systems(
        FixedPostUpdate,
        press_once.in_set(BeltSet).before(bound_belts),
    );
    for _ in 0..3 {
        app.update();
    }
    // The cell (3, 2, 0) of the fixture's 1 m by 0.6 m cells, at the centre of its near face.
    let (pressed, point) = (3 + 6 * 2, Vec3::new(0.5, 1.5, -WALL.z / 2.0));
    let dt = f64::from(
        app.world()
            .resource::<Time<Fixed>>()
            .timestep()
            .as_secs_f32(),
    );
    let mut intact = Breakage::new(kind, 0);
    let push = Push {
        along: DVec3::Z,
        closing: 0.0,
        lean: 1.0,
    };
    let given = (standing(&mut intact).give_along(&[pressed], push, 0.0, dt))
        .expect("a single leaf gives through its far face");
    let world = app.world_mut();
    let structure_collider = (world.query_filtered::<(Entity, &ColliderOf), With<Element>>())
        .iter(world)
        .find_map(|(collider, of)| (of.body == root).then_some(collider))
        .unwrap();
    let mass = 1000.0;
    let body = world
        .spawn((
            RigidBody::Dynamic,
            Position(Vec3::new(0.0, 1.5, -5.0)),
            Rotation::default(),
            Mass(mass),
            NoAutoMass,
            SleepingDisabled,
        ))
        .id();
    world.resource_mut::<Once>().0 = Some(crate::contact_transfer::StagedForce {
        structure: root,
        structure_collider,
        body,
        point,
        force: Vec3::NEG_Z * (4.0 * given.impulse / dt) as f32,
    });
    app.update();
    let kept = -f64::from(app.world().get::<LinearVelocity>(body).unwrap().0.z * mass);
    assert!(
        // f32 integration over the step's substeps.
        (kept - given.impulse).abs() < 1e-3 * given.impulse,
        "the body keeps {kept} N·s of the {} N·s the cell returns",
        given.impulse
    );
    let freed: Vec<usize> = (app.world().resource::<BoundAccount>().freed.iter())
        .map(|&(_, c)| usize::from(c))
        .collect();
    assert_eq!(
        freed, given.leaves,
        "the cells pressed past their bound leave"
    );
}
