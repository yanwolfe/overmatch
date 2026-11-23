//! The contact law wired into the step, on a generic wall fixture (game state only).

use avian3d::prelude::*;

use super::super::tests::composed_kind;
use super::*;
use crate::construction::fracture::bound::Price;
use crate::construction::tests::wall_construction;

/// Fixture: a brick leaf 6 m long, 2.4 m high and 0.25 m thick in 1 m by 0.6 m cells, and a
/// rigid box 2 m wide, 1.2 m high and 2 m long sent at its middle along +z from 0.175 m short.
const WALL: Vec3 = Vec3::new(6.0, 2.4, 0.25);
const BOX: Vec3 = Vec3::new(2.0, 1.2, 2.0);
const START: Vec3 = Vec3::new(0.0, 1.0, -1.3);

/// The fixture wall with a box of `mass` kg at `at`, moving at `speed` m/s along +z.
fn wall_and_box(mass: f32, at: Vec3, speed: f32) -> (App, Entity, Entity) {
    let kind = wall_construction(WALL, [6, 4, 1], "Brick");
    let (mut app, root) = composed_kind(false, &kind);
    for _ in 0..3 {
        app.update();
    }
    let body = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Position(at),
            Rotation::default(),
            Collider::cuboid(BOX.x, BOX.y, BOX.z),
            Mass(mass),
            NoAutoMass,
            piece_layers(),
            LinearVelocity(Vec3::Z * speed),
            SleepingDisabled,
        ))
        .id();
    (app, root, body)
}

/// The work `root`'s cells have taken, J, per chunk.
fn work(app: &App, root: Entity) -> Vec<f64> {
    app.world().get::<Breakage>(root).unwrap().live.work.clone()
}

/// The box of `mass` kg sent at `speed` m/s, after 2.5 s: where its centre is along z (m), its
/// speed along z (m/s) and the cells the wall lost.
fn sent(mass: f32, speed: f32) -> (f32, f32, usize) {
    let (mut app, _, body) = wall_and_box(mass, START, speed);
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

/// Invariant: a cell's bonds never give work back, they take no more than the body lost, and
/// cells leave only on a step that spent a way: its work reached, or all it had left held in the
/// step. The stall the law replaced was this count lost; every body meeting standing matter
/// relies on it. The bound on the work is the body's own loss: with no gravity and nothing else
/// touching it, what its kinetic energy fell by is the most its contacts did on anything.
/// Mutants: the work counted once per give that holds a point, or at the price in place of what
/// was returned (it passes the body's loss); the count cleared on a step that frees nothing (a
/// cell's work falls); a block kept past its work; a block freed by a touch (cells leave on a
/// step whose rows show neither). Fixture: 57 t at 4.5 m/s and at 0.5 m/s, the energy read
/// until the first cells leave and become bodies of their own.
#[test]
fn bonds_take_the_work_done_on_them_once_and_part_only_when_a_way_is_spent() {
    for speed in [4.5_f32, 0.5] {
        let mass = 57_000.0_f32;
        let (mut app, root, body) = wall_and_box(mass, START, speed);
        let dt = f64::from(
            app.world()
                .resource::<Time<Fixed>>()
                .timestep()
                .as_secs_f32(),
        );
        let lost = |app: &App| {
            let after = app.world().get::<LinearVelocity>(body).unwrap().0;
            0.5 * f64::from(mass) * f64::from(speed * speed - after.length_squared())
        };
        let mut before = work(&app, root);
        let mut parted = false;
        for tick in 0..160 {
            app.update();
            let now = work(&app, root);
            assert!(
                before.iter().zip(&now).all(|(before, now)| now >= before),
                "tick {tick}: a cell's work fell"
            );
            let account = app.world().resource::<BoundAccount>();
            let reached = (account.blocks.iter()).any(|block| block.taken >= block.price[1]);
            // `reaction` rations a step below the way's strength only when the body crosses the
            // give left within it.
            let held_all = (account.blocks.iter()).any(|block| block.bound < block.price[0] * dt);
            assert!(
                !reached || !account.freed.is_empty(),
                "tick {tick}: a block stands past its work: {:?}",
                account.blocks
            );
            assert!(
                account.freed.is_empty() || reached || held_all,
                "tick {tick}: cells leave with no way spent: {:?}",
                account.blocks
            );
            if !parted {
                let (taken, lost) = (now.iter().sum::<f64>(), lost(&app));
                assert!(
                    // f32 velocities.
                    taken <= lost * (1.0 + 1e-4),
                    "tick {tick} at {speed} m/s: the cells took {taken} J of the {lost} J the \
                     box lost"
                );
            }
            parted |= !account.freed.is_empty();
            before = now;
        }
        assert!(parted, "at {speed} m/s the box frees cells");
    }
}

/// Fixture: the two cells squarely in the box's path, (2, 1) and (3, 1) of the wall's 1 m by
/// 0.6 m cells.
const AHEAD: [usize; 2] = [2 + 6, 3 + 6];

/// Invariant: a way whose step held all the work it had left, and did not stop the body, is
/// spent on that step, so a cell does not stand while a body it cannot stop is deeper in it than
/// its give and the travel of the steps that find and cross it. A hull driving through a wall
/// relies on it: cells that stood on while returning nothing let it sink 0.1 to 0.25 m into
/// standing matter (MEASURED before this rule, 10 km/h). Mutant: a block parted on the counted
/// work alone (the last work is rationed at the solve's speed and counted at a lower one, so
/// the count closes on the way's work without reaching it). Fixture: 57 t at 0.5 m/s; two
/// steps of travel, one for the narrow phase to hand the contact over and one to cross.
#[test]
fn a_cell_does_not_stand_in_a_body_past_its_give() {
    let speed = 0.5_f32;
    let (mut app, root, body) = wall_and_box(57_000.0, START, speed);
    let dt = app
        .world()
        .resource::<Time<Fixed>>()
        .timestep()
        .as_secs_f32();
    let mut give = 0.0_f64;
    for tick in 0..160 {
        app.update();
        let world = app.world();
        give = (world.resource::<BoundAccount>().blocks.iter())
            .map(|block| block.price[1] / block.price[0])
            .fold(give, f64::max);
        let state = &world.get::<Breakage>(root).unwrap().live.state;
        if AHEAD.iter().any(|&c| state.standing(c)) {
            let inside = world.get::<Position>(body).unwrap().0.z + BOX.z / 2.0 + WALL.z / 2.0;
            assert!(
                f64::from(inside) <= give + f64::from(2.0 * speed * dt),
                "tick {tick}: the box is {inside} m inside cells that stand, their give {give} m"
            );
        }
    }
    let state = &app.world().get::<Breakage>(root).unwrap().live.state;
    assert!(
        AHEAD.iter().all(|&c| !state.standing(c)),
        "the box frees the cells in its path"
    );
}

/// Invariant: the work a cell takes is the impulse it returned times the body's mean speed into
/// it over the step, which for a force constant over the step is the work that force did. So a
/// body the cells stop gives them the kinetic energy it lost, however few steps the stop takes.
/// Loose pieces rely on it: one stopped inside a step is charged half its entry speed times the
/// impulse, and one thrown is not charged at the speed it left with. Mutants: the speed read at
/// the step's end alone (a body stopped in one step does no work), or at its start alone (twice
/// the loss). Fixture: 20 kg at 4.5 m/s, which the leaf stops; no gravity. The stop is uneven
/// over the face, so up to a tenth of the loss goes to friction and to the spin it leaves.
#[test]
fn cells_take_the_kinetic_energy_of_a_body_they_stop() {
    let (mass, speed) = (20.0_f32, 4.5_f32);
    let (mut app, root, body) = wall_and_box(mass, START, speed);
    for _ in 0..64 {
        app.update();
    }
    let after = app.world().get::<LinearVelocity>(body).unwrap().0;
    let lost = 0.5 * f64::from(mass) * f64::from(speed * speed - after.length_squared());
    let taken = work(&app, root).iter().sum::<f64>();
    assert!(
        after.z < 0.01 * speed && taken <= lost && taken > 0.9 * lost,
        "the stopped box, now at {after} m/s, lost {lost} J and the cells took {taken} J"
    );
}

/// Invariant: work is force through an advance, so a body that does not advance does none: one
/// pushed back out of the cells it overlaps, one held against them by a force they bear, and one
/// resting on them under gravity. Rubble born beside standing matter and rubble resting on it
/// rely on this not to wear a building away. Mutants: the sign of the advance dropped (the body
/// pushed out does work); the work counted from the impulse alone, or the entering speed read
/// after the step's forces are added to it (the held and the resting body are charged every
/// step: the resting one its weight times half a step's fall). Fixture: a 1 t box 50 mm inside
/// the leaf's face and at rest; the same box touching the face under 5 kN, and under a gravity
/// turned along +z, which makes the face its floor, each read over 6 s after 1 s to settle.
#[test]
fn a_body_pushed_out_held_or_resting_does_no_work() {
    let face = -WALL.z / 2.0 - BOX.z / 2.0;
    let (mut app, root, _) = wall_and_box(1000.0, Vec3::new(0.0, 1.0, face + 0.05), 0.0);
    for _ in 0..64 {
        app.update();
        assert!(app.world().resource::<BoundAccount>().freed.is_empty());
    }
    assert_eq!(work(&app, root).iter().sum::<f64>(), 0.0);

    for gravity in [false, true] {
        let (mut app, root, body) = wall_and_box(1000.0, Vec3::new(0.0, 1.0, face + 0.001), 0.0);
        if gravity {
            app.insert_resource(Gravity(Vec3::Z * 9.81));
        } else {
            (app.world_mut().entity_mut(body)).insert(ConstantForce(Vec3::Z * 5000.0));
        }
        for _ in 0..64 {
            app.update();
        }
        let settled = work(&app, root).iter().sum::<f64>();
        for _ in 0..384 {
            app.update();
        }
        let account = app.world().resource::<BoundAccount>();
        let least = (account.blocks.iter())
            .map(|block| block.price[1])
            .fold(f64::INFINITY, f64::min);
        let grown = work(&app, root).iter().sum::<f64>() - settled;
        assert!(
            // The solver leaves a held body a residual speed of f32 rounding.
            least.is_finite() && grown < 1e-5 * least,
            "gravity {gravity}: over 6 s the cells took {grown} J against a price of {least} J"
        );
    }
}

/// Fixture: one force on a belt's body, staged every tick while set.
#[derive(Resource)]
struct Staging(Option<crate::contact_transfer::StagedForce>);

fn press(
    staging: Res<Staging>,
    mut transfers: ResMut<ContactTransfers>,
    mut bodies: Query<Forces>,
) {
    if let Some(force) = staging.0 {
        (bodies.get_mut(force.body).unwrap()).apply_force_at_point(force.force, force.point);
        transfers.stage(force);
    }
}

/// Fixture: the cell (3, 2, 0) of the wall's 1 m by 0.6 m cells, which a belt presses at the
/// centre of its near face.
const PRESSED: usize = 3 + 6 * 2;

/// What a way of giving holds, and the cells that leave by it.
type Parting = (Price, Vec<usize>);

/// The fixture wall with [`press`] mounted, a body of `mass` kg without a collider 5 m before
/// the middle of the face of one cell, and what that cell holds against a push through the
/// leaf, with the cells that leave: through its far face, and shed sideways. Returns the app, a
/// force of nothing staged on the cell for the caller to size, the two ways and the step.
fn wall_and_belt(mass: f32) -> (App, crate::contact_transfer::StagedForce, [Parting; 2], f64) {
    let kind = wall_construction(WALL, [6, 4, 1], "Brick");
    let (mut app, root) = composed_kind(false, &kind);
    app.insert_resource(Staging(None))
        .add_systems(FixedPostUpdate, press.in_set(BeltSet).before(bound_belts));
    for _ in 0..3 {
        app.update();
    }
    let (pressed, point) = (PRESSED, Vec3::new(0.5, 1.5, -WALL.z / 2.0));
    let dt = f64::from(
        app.world()
            .resource::<Time<Fixed>>()
            .timestep()
            .as_secs_f32(),
    );
    let mut intact = Breakage::new(kind, 0);
    let standing = standing(&mut intact);
    let through = (standing.shear_out(&standing.plug(&[pressed], DVec3::Z), DVec3::Z))
        .expect("a single leaf gives through its far face");
    let shed = (standing.shed(&[pressed], DVec3::Z)).expect("a leaf sheds a cell");
    let world = app.world_mut();
    let structure_collider = (world.query_filtered::<(Entity, &ColliderOf), With<Element>>())
        .iter(world)
        .find_map(|(collider, of)| (of.body == root).then_some(collider))
        .unwrap();
    let body = world
        .spawn((
            RigidBody::Dynamic,
            // In line with the force, which then turns nothing.
            Position(point - Vec3::Z * 5.0),
            Rotation::default(),
            Mass(mass),
            NoAutoMass,
            SleepingDisabled,
        ))
        .id();
    let force = crate::contact_transfer::StagedForce {
        structure: root,
        structure_collider,
        body,
        point,
        force: Vec3::ZERO,
    };
    (app, force, [(through, vec![pressed]), shed], dt)
}

/// Invariant: a belt takes from standing matter no more than its bonds return, and a belt that
/// does not advance into the cells frees none. `track::rubble::record_static_contacts` stages a
/// belt's whole force on fixed construction and relies on this to cap it. Mutants:
/// [`bound_belts`] unmounted (the body keeps the whole force), the force taken back with its
/// sign flipped (the body gains more than the force), cells freed by an over-push alone (they
/// leave under a belt the push drives away). Fixture: a 1 t body clear of the wall and at rest,
/// pushed off the cell's face for one tick with four times the least force it holds.
#[test]
fn a_belt_keeps_what_standing_matter_returns() {
    let mass = 1000.0;
    let (mut app, force, ways, dt) = wall_and_belt(mass);
    let holds = ways[0].0.force.min(ways[1].0.force);
    app.world_mut().resource_mut::<Staging>().0 = Some(crate::contact_transfer::StagedForce {
        force: Vec3::NEG_Z * (4.0 * holds) as f32,
        ..force
    });
    app.update();
    let kept = -f64::from(app.world().get::<LinearVelocity>(force.body).unwrap().0.z * mass);
    assert!(
        // f32 integration over the step's substeps.
        (kept - holds * dt).abs() < 1e-3 * holds * dt,
        "the body keeps {kept} N·s of the {} N·s the cell returns",
        holds * dt
    );
    assert!(app.world().resource::<BoundAccount>().freed.is_empty());
}

/// Invariant: a belt's force on standing cells is read as a contact point's is: the cells take
/// what they returned times the belt's advance into them, and leave on the step that spends a
/// way's work, not before and not after. A hull whose belts ride onto standing matter relies on
/// it; the staged force has no solver point to be read from. Mutants: the belt's points not
/// counted (the cells never leave); cells freed by the press alone (they leave on the first
/// tick); a way spent on the counted work alone (the last step is rationed to the work left and
/// the belt keeps less than it pressed with, so the count closes on the way's work over several
/// more ticks). Fixture: a body too heavy for the force to slow, advancing at 2 mm/s under half
/// the least force the cell holds, so each tick does `F v dt` and the way's work is due on tick
/// `W / (F v dt)`, rounded up.
#[test]
fn cells_a_belt_advances_into_leave_on_the_step_that_spends_their_work() {
    let (mut app, force, ways, dt) = wall_and_belt(1e9);
    let pressing = 0.5 * ways[0].0.force.min(ways[1].0.force);
    let (least, leaves) = (ways.iter())
        .min_by(|(a, _), (b, _)| a.work.total_cmp(&b.work))
        .unwrap();
    let advance = 0.002;
    (app.world_mut().entity_mut(force.body)).insert(LinearVelocity(Vec3::Z * advance as f32));
    app.world_mut().resource_mut::<Staging>().0 = Some(crate::contact_transfer::StagedForce {
        force: Vec3::NEG_Z * pressing as f32,
        ..force
    });
    let due = least.work / (pressing * advance * dt);
    assert!(due > 8.0, "fixture: the work takes several ticks, {due}");
    let mut ticks = 0.0;
    while app.world().resource::<BoundAccount>().freed.is_empty() {
        app.update();
        ticks += 1.0;
        assert!(
            ticks <= due.ceil(),
            "the cell stands past its work, {due} ticks"
        );
    }
    let freed: Vec<usize> = (app.world().resource::<BoundAccount>().freed.iter())
        .map(|&(_, c)| usize::from(c))
        .collect();
    assert!(
        ticks == due.ceil() && freed == *leaves,
        "cells {freed:?} left after {ticks} ticks; the work is due after {due}"
    );
}
