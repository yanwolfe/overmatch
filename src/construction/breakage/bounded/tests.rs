//! The contact law wired into the step, on a generic wall fixture (game state only).

use avian3d::prelude::*;
use bevy::ecs::system::RunSystemOnce;

use super::super::tests::composed_kind;
use super::*;
use crate::construction::fracture::bound::Price;
use crate::construction::tests::wall_construction;

/// Invariant: each owner's result comes back in the owners' order, whichever worker took it and
/// however many there are. [`bound_contacts`], [`bound_belts`] and [`release`] hand bounds,
/// forces and freed cells on in that order. This witnesses the order alone; that a task touches
/// no other owner's state is by construction ([`per_owner`]). Mutant: results gathered as
/// workers finish, or split by worker and not rejoined in order. Fixture: nine owners, one more
/// than the widest pool, each task yielding so that later ones can finish first.
#[test]
fn owners_come_back_in_their_order_on_any_pool() {
    let owners: Vec<usize> = (0..9).collect();
    for threads in [1, 2, 8] {
        let pool = bevy::tasks::TaskPoolBuilder::new()
            .num_threads(threads)
            .build();
        let done = per_owner(&pool, &mut owners.clone(), |&mut owner| {
            if owner % 2 == 0 {
                std::thread::yield_now();
            }
            owner
        });
        assert_eq!(done, owners, "{threads} threads");
    }
}

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

/// The fixed step, s.
fn step(app: &App) -> f64 {
    let step = app.world().resource::<Time<Fixed>>().timestep();
    f64::from(step.as_secs_f32())
}

/// The kinetic energy of translation a body of `mass` kg has lost since it moved at `speed`
/// m/s, J.
fn lost(app: &App, body: Entity, mass: f32, speed: f32) -> f64 {
    let now = app.world().get::<LinearVelocity>(body).unwrap().0;
    0.5 * f64::from(mass) * (f64::from(speed).powi(2) - now.as_dvec3().length_squared())
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
/// a block parts on the step its counted work reaches a way's, and on no other. Without the
/// count kept from step to step a slowed body stands inside cells that never leave; every
/// body meeting standing matter relies on it. Premise of the bound on the work: no gravity,
/// nothing else touching the box, and a box that meets the leaf squarely, so it does not turn
/// and no substep's work is negative while it advances; then what its kinetic energy fell by
/// is the most its contacts did on anything, friction's share included. Mutants: the work
/// counted once per give that holds a point, or at the price in place of what was returned
/// (it passes the body's loss); the count cleared on a step that frees nothing (a cell's work
/// falls); a block kept past its work; a block freed by a touch (cells leave on a step whose
/// rows show no way's work reached). Fixture: 57 t at 4.5 m/s and at 0.5 m/s, the energy read
/// until the first cells leave and become bodies of their own.
#[test]
fn bonds_take_the_work_done_on_them_once_and_part_only_when_a_way_is_spent() {
    for speed in [4.5_f32, 0.5] {
        let mass = 57_000.0_f32;
        let (mut app, root, body) = wall_and_box(mass, START, speed);
        let entry = 0.5 * f64::from(mass) * f64::from(speed).powi(2);
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
            let reached = (account.blocks.iter()).any(|block| price(block).spent(block.taken));
            assert!(
                !reached || !account.freed.is_empty(),
                "tick {tick}: a block stands past its work: {:?}",
                account.blocks
            );
            assert!(
                account.freed.is_empty() || reached,
                "tick {tick}: cells leave with no way spent: {:?}",
                account.blocks
            );
            if !parted {
                let (taken, lost) = (now.iter().sum::<f64>(), lost(&app, body, mass, speed));
                assert!(
                    // The loss is a difference of two f32 speeds squared: each is off by half an
                    // ulp at most, so the difference by four ulps of the entry energy.
                    taken <= lost + 4.0 * f64::from(f32::EPSILON) * entry,
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

/// Invariant: a cell does not stand while a body it cannot stop is deeper in it than its give
/// and the travel of the steps that find and cross it. A hull driving through a wall relies on
/// it not to sink into standing matter. Mutant: a block returning nothing while it stands (its
/// work never counted, or counted and never reached). Fixture: 57 t at 1.5 m/s, 64 kJ, several times the
/// work of any block in its path; two steps of travel, one for the narrow phase to hand the
/// contact over and one to cross.
#[test]
fn a_cell_does_not_stand_in_a_body_past_its_give() {
    let speed = 1.5_f32;
    let (mut app, root, body) = wall_and_box(57_000.0, START, speed);
    let dt = step(&app);
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
                f64::from(inside) <= give + 2.0 * f64::from(speed) * dt,
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

/// Invariant: the work a block's cells take over a step is the kinetic energy its points took
/// from the body: `avian_ext::reaction_bound` counts, each substep, the points' impulses times
/// the mean of the speeds they close at before the substep's forces and after its last solve,
/// and `J (v0 + v1) / 2` with `J = m (v0 - v1)` is `m (v0² - v1²) / 2`, however the solve
/// spread the impulse. Loose pieces rely on it: one stopped inside a step is charged half its
/// entry speed times the impulse, and one thrown is not charged at the speed it left with.
/// Mutants: the speed read at the substep's end alone (a body stopped in one substep does no
/// work), or at its start alone (twice the loss). Fixture: 20 kg at 4.5 m/s, which the leaf
/// stops; no gravity, no friction and no turning, so the cells' normal impulses are all that
/// moves the box, and each step's work is the energy the step took while the box advances and
/// none while it backs out.
#[test]
fn cells_take_the_kinetic_energy_of_a_body_they_stop() {
    let (mass, speed) = (20.0_f32, 4.5_f32);
    let (mut app, root, body) = wall_and_box(mass, START, speed);
    (app.world_mut().entity_mut(body)).insert((
        LockedAxes::ROTATION_LOCKED,
        Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
    ));
    let entry = 0.5 * f64::from(mass) * f64::from(speed).powi(2);
    let velocity = |app: &App| {
        app.world()
            .get::<LinearVelocity>(body)
            .unwrap()
            .0
            .as_dvec3()
    };
    let (mut before, mut taken) = (velocity(&app), 0.0);
    for tick in 0..64 {
        app.update();
        let (after, now) = (velocity(&app), work(&app, root).iter().sum::<f64>());
        let advancing = (before + after).z > 0.0;
        let took = 0.5 * f64::from(mass) * (before.length_squared() - after.length_squared());
        let due = if advancing { took } else { 0.0 };
        assert!(
            // Each of the sixteen points moves the speed in each of a substep's two passes,
            // half an ulp a move, six substeps a step; an energy carries twice its speed's
            // error.
            (now - taken - due).abs() <= 192.0 * f64::from(f32::EPSILON) * entry,
            "tick {tick}: the cells took {} J on a step that took {due} J from the box",
            now - taken
        );
        (before, taken) = (after, now);
    }
    assert!(
        before.z < 0.01 * f64::from(speed) && taken > 0.5 * entry,
        "the box is stopped: now at {before} m/s, the cells took {taken} J of {entry} J"
    );
}

/// Invariant: work is force through an advance, so a body that does not advance does none: one
/// pushed back out of the cells it overlaps, one held against them by a force they bear, and one
/// resting on them under gravity. Rubble born beside standing matter and rubble resting on it
/// rely on this not to wear a building away. Mutants: the sign of the advance dropped (the body
/// pushed out does work); the work counted from the impulse alone, or the entering speed read
/// after the step's forces are added to it (the held and the resting body are charged every
/// step the force's impulse times half the speed it adds, `m a dt · a dt / 2`). Fixture: a 1 t
/// box 50 mm inside the leaf's face and at rest; the same box touching the face under 5 kN, and
/// under a gravity turned along +z, which makes the face its floor, each read over 6 s after
/// 1 s to settle.
#[test]
fn a_body_pushed_out_held_or_resting_does_no_work() {
    let face = -WALL.z / 2.0 - BOX.z / 2.0;
    let mass = 1000.0_f32;
    let (mut app, root, _) = wall_and_box(mass, Vec3::new(0.0, 1.0, face + 0.05), 0.0);
    for _ in 0..64 {
        app.update();
        assert!(app.world().resource::<BoundAccount>().freed.is_empty());
    }
    assert_eq!(work(&app, root).iter().sum::<f64>(), 0.0);

    // Fixture: the acceleration that holds the box on the face, m/s².
    for (gravity, held) in [(false, 5.0_f32), (true, 9.81)] {
        let (mut app, root, body) = wall_and_box(mass, Vec3::new(0.0, 1.0, face + 0.001), 0.0);
        if gravity {
            app.insert_resource(Gravity(Vec3::Z * held));
        } else {
            (app.world_mut().entity_mut(body)).insert(ConstantForce(Vec3::Z * held * mass));
        }
        for _ in 0..64 {
            app.update();
        }
        let settled = work(&app, root).iter().sum::<f64>();
        let steps = 384;
        for _ in 0..steps {
            app.update();
        }
        let grown = work(&app, root).iter().sum::<f64>() - settled;
        let added = f64::from(held) * step(&app);
        let mutant = f64::from(steps) * f64::from(mass) * added * added / 2.0;
        assert!(
            // A substep's force adds `a dt / 6` to the speed and the points take it back, each
            // of the sixteen rounding it half an ulp in each of the two passes: a substep ends
            // within sixteen ulps of that speed of rest, and its points, which return the
            // force's impulse `m a dt / 6`, do at most that impulse times the residual speed.
            // Six substeps a step: `16 ε m (a dt)² / 6` a step, a third of sixteen ulps of the
            // mutant's charge.
            grown <= 16.0 / 3.0 * f64::from(f32::EPSILON) * mutant,
            "held at {held} m/s² over 6 s the cells took {grown} J; the mutant charges {mutant} J"
        );
    }
}

/// Invariant: each body on a block may do the work the block had left at the step's start, so
/// two bodies that part it in one step do at most twice that between them, and one body no
/// more than it. The work left is read once, before the step, and a budget is one body's
/// (`avian_ext::reaction_bound`): the second body's share is the law's stated residual, and it
/// is bounded by the bodies on the block, never by their points. Mutants: a budget for each
/// point or each way (the block takes more than the bodies on it times its work); the block's
/// work left shared by nothing at all (it takes whatever stops the bodies). Fixture: boxes
/// 0.4 m wide abreast inside one column of the leaf's 1 m cells, 57 t at 4.5 m/s apiece, far
/// more than the column's bonds take, read on the step the first cells leave; two blocks are
/// equal when their prices are.
#[test]
fn bodies_parting_a_block_together_do_its_work_once_each_at_most() {
    for bodies in [1_usize, 2] {
        let (mut app, _, first) = wall_and_box(57_000.0, START, 4.5);
        app.world_mut().despawn(first);
        for n in 0..bodies {
            app.world_mut().spawn((
                RigidBody::Dynamic,
                Position(START.with_x(0.3 + 0.4 * n as f32)),
                Rotation::default(),
                Collider::cuboid(0.4, BOX.y, BOX.z),
                Mass(57_000.0),
                NoAutoMass,
                piece_layers(),
                LinearVelocity(Vec3::Z * 4.5),
                SleepingDisabled,
            ));
        }
        let rows = (0..32)
            .find_map(|_| {
                app.update();
                let account = app.world().resource::<BoundAccount>();
                (!account.freed.is_empty()).then(|| account.blocks.clone())
            })
            .expect("the boxes free cells");
        let parted: Vec<&BlockRow> = (rows.iter())
            .filter(|row| price(row).spent(row.taken))
            .collect();
        assert!(
            !parted.is_empty(),
            "fixture: the boxes part blocks: {rows:?}"
        );
        // Each of a box's sixteen points moves its speed in each of a substep's two passes,
        // half an ulp a move, six substeps a step; an energy carries twice its speed's error.
        let rounding = 192.0 * f64::from(f32::EPSILON) * 0.5 * 57_000.0 * 4.5 * 4.5;
        for row in parted {
            let on = (rows.iter().filter(|other| other.price == row.price)).count();
            assert_eq!(on, bodies, "fixture: every box is on every block: {rows:?}");
            // The first step on the block: all its work was left, and nothing but the block
            // touches a box, so the work the cut leaves a point is the work it does.
            let work = row.price[1];
            assert!(
                row.taken < on as f64 * (work + rounding),
                "{on} on a block of {work} J did {} J",
                row.taken
            );
        }
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

/// A row's price ([`BlockRow::price`]).
fn price(row: &BlockRow) -> Price {
    Price {
        force: row.price[0],
        work: row.price[1],
    }
}

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
    let dt = step(&app);
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

/// Invariant: a belt's force on standing cells is counted and spent by the rule a contact
/// point's is (the force itself differs: the whole staged force, `Point::Staged`): the cells
/// take what they returned times the belt's advance into them, and leave on the step that
/// spends the way's work, not before and not after. A hull whose belts ride onto standing matter relies on
/// it; the staged force has no solver point to be read from. Mutants: the belt's points not
/// counted (the cells never leave); cells freed by the press alone (they leave on the first
/// tick); a way spent on the counted work alone (the last step is rationed to the work left and
/// the belt keeps less than it pressed with, so the count closes on the way's work over several
/// more ticks). Fixture: a body too heavy for the force to slow, advancing at 2 mm/s under half
/// the least force any way of the cell holds, so the belt keeps its whole force whichever way
/// is priced and each tick does `F v dt`; the work is one count on the cell, so the way spent
/// first is the least in work, whatever it holds, due on tick `W / (F v dt)`, rounded up.
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
