//! GUARD for the bounded contact hooks (`vendor/avian3d-0.8.0-overmatch`, `OVERMATCH_PATCH.md`,
//! "Bounded contact") and the game's seam behind them, `crate::avian_ext::reaction_bound`, in the
//! game's building model (game state only). A standing block returns no more than its bonds can
//! and gives when the bodies on it have done its bonds' work (`construction::fracture::bound`), so
//! a group must return its bound over the step and no more, a budget's points must do its work
//! and no more whatever their number, and a point in neither must stay Avian's rigid contact.
//!
//! Fixture throughout: a static slab, no gravity unless a test says so, no friction, and bodies
//! of an isotropic angular inertia, so a body's kinetic energy is `½ m v² + ½ I ω²`.

use avian3d::prelude::*;
use bevy::ecs::system::RunSystemOnce;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;

use crate::avian_ext::reaction_bound::{At, Bounds, Member};

/// Fixture: what every body touching the slab is handed for its points on it, each step.
#[derive(Resource, Clone, Copy, Default)]
struct Hands {
    /// Each point a group of its own, of this bound, N·s.
    each: Option<f32>,
    /// All its points one group, of this bound, N·s, each at lean [`Self::lean`].
    group: Option<f32>,
    /// The lean of the group's members; one when not set.
    lean: Option<f32>,
    /// All its points one budget, of this work less what the points have done, J.
    work: Option<f32>,
}

/// One body's points on the slab as last handed, and their account since the fixture began.
#[derive(Default)]
struct Entry {
    points: Vec<At>,
    /// What its points returned over the last step, N·s.
    returned: f32,
    /// Its group of all its points, and what that returned over the last step by the seam's
    /// own reading (`Bounds::returned`), N·s.
    group: Option<usize>,
    group_returned: f32,
    /// The work its points have done, J: their budgets' counts, as the building model keeps
    /// them (`breakage::bounded::settle`).
    done: f32,
    /// The step, from 1, on which the count reached the hands' work: the step the building
    /// model parts a block on.
    spent: Option<usize>,
}

#[derive(Resource, Default)]
struct Ledger {
    bodies: HashMap<Entity, Entry>,
    steps: usize,
}

#[derive(Resource)]
struct Slab(Entity);

fn hand_over(
    graph: Res<ContactGraph>,
    slab: Res<Slab>,
    hands: Res<Hands>,
    mut bounds: ResMut<Bounds>,
    mut ledger: ResMut<Ledger>,
) {
    for entry in ledger.bodies.values_mut() {
        entry.points.clear();
    }
    for pair in graph.iter_active() {
        let body = match (pair.body1, pair.body2) {
            (Some(one), Some(other)) if one == slab.0 => other,
            (Some(one), Some(other)) if other == slab.0 => one,
            _ => continue, // Between two bodies: Avian's contact, unread.
        };
        let points = (pair.manifolds.iter().enumerate()).flat_map(|(manifold, points)| {
            (0..points.points.len()).map(move |point| At {
                contact: pair.contact_id,
                manifold,
                point,
            })
        });
        ledger.bodies.entry(body).or_default().points.extend(points);
    }
    for (&body, entry) in &mut ledger.bodies {
        for &at in &entry.points {
            bounds.read(at);
            if let Some(most) = hands.each {
                bounds.group(vec![Member { at, lean: 1.0 }], most);
            }
        }
        let lean = hands.lean.unwrap_or(1.0);
        let members = (entry.points.iter().map(|&at| Member { at, lean })).collect();
        entry.group = hands.group.map(|most| bounds.group(members, most));
        if let Some(work) = hands.work.filter(|_| entry.spent.is_none()) {
            bounds.budget(body, entry.points.clone(), work - entry.done);
        }
    }
}

/// After a step: what each body's points returned and did over it.
fn settle(bounds: Res<Bounds>, hands: Res<Hands>, mut ledger: ResMut<Ledger>) {
    ledger.steps += 1;
    let step = ledger.steps;
    for entry in ledger.bodies.values_mut() {
        entry.returned = entry.points.iter().map(|&at| bounds.impulse(at)).sum();
        entry.group_returned = entry.group.map_or(0.0, |group| bounds.returned(group));
        entry.done += (entry.points.iter())
            .map(|&at| bounds.work(at))
            .sum::<f32>();
        if hands.work.is_some_and(|work| entry.done >= work) {
            entry.spent.get_or_insert(step);
        }
    }
}

/// Fixture: a body's mass, kg, and its angular inertia about every axis, kg·m².
const INERTIA: f32 = 0.25;

/// The fixture's shapes, by the points each lays on the slab.
#[derive(Clone, Copy, Debug)]
enum Shape {
    /// A ball: one point.
    One,
    /// A box face: four.
    Four,
    /// Four box faces side by side: sixteen.
    Sixteen,
}

impl Shape {
    fn collider(self) -> Collider {
        match self {
            Self::One => Collider::sphere(0.5),
            Self::Four => Collider::cuboid(1.0, 1.0, 1.0),
            Self::Sixteen => Collider::compound(
                [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)]
                    .map(|(x, z)| {
                        (
                            Vec3::new(0.25 * x, 0.0, 0.25 * z),
                            Quat::IDENTITY,
                            Collider::cuboid(0.5, 1.0, 0.5),
                        )
                    })
                    .to_vec(),
            ),
        }
    }

    fn points(self) -> usize {
        match self {
            Self::One => 1,
            Self::Four => 4,
            Self::Sixteen => 16,
        }
    }
}

/// The fixture's world: the slab, its upper face at y = 0.5, and the hands.
fn world(hands: Hands) -> App {
    let mut app = super::avian_child_pose_churn::physics_app();
    app.insert_resource(Gravity(Vec3::ZERO))
        .insert_resource(hands)
        .init_resource::<Ledger>()
        .add_systems(
            PhysicsSchedule,
            hand_over
                .after(PhysicsStepSystems::NarrowPhase)
                .before(PhysicsStepSystems::Solver),
        );
    let slab = app
        .world_mut()
        .spawn((
            RigidBody::Static,
            Collider::cuboid(4.0, 1.0, 4.0),
            Friction::ZERO,
        ))
        .id();
    app.insert_resource(Slab(slab));
    app
}

/// A body of `shape` and `mass` kg, its lowest point `height` m over the slab's face, moving
/// down at `approach` m/s.
fn body(app: &mut App, shape: Shape, mass: f32, height: f32, approach: f32) -> Entity {
    app.world_mut()
        .spawn((
            RigidBody::Dynamic,
            shape.collider(),
            Mass(mass),
            AngularInertia::new(Vec3::splat(mass * INERTIA)),
            (NoAutoMass, NoAutoAngularInertia, SleepingDisabled),
            Friction::ZERO,
            Position(Vec3::Y * (1.0 + height)),
            LinearVelocity(Vec3::NEG_Y * approach),
        ))
        .id()
}

/// Fixture: a body starts 1 mm into what it rests on, so the first step has its contact.
const INTO: f32 = -0.001;

/// One physics step, settled. The app's first update runs none.
fn step(app: &mut App) {
    if app.world().resource::<Ledger>().steps == 0 {
        app.update();
        app.world_mut().resource_mut::<Ledger>().bodies.clear();
    }
    app.update();
    app.world_mut().run_system_once(settle).unwrap();
}

/// A body's kinetic energy, J.
fn energy(app: &App, body: Entity, mass: f32) -> f64 {
    let linear = app.world().get::<LinearVelocity>(body).unwrap().0;
    let angular = app.world().get::<AngularVelocity>(body).unwrap().0;
    0.5 * f64::from(mass)
        * (linear.as_dvec3().length_squared()
            + f64::from(INERTIA) * angular.as_dvec3().length_squared())
}

fn entry(app: &App, body: Entity) -> &Entry {
    &app.world().resource::<Ledger>().bodies[&body]
}

fn down(app: &App, body: Entity) -> f32 {
    -app.world().get::<LinearVelocity>(body).unwrap().0.y
}

fn substep(app: &App) -> f32 {
    (app.world().resource::<SubstepCount>().0 as f32).recip()
}

/// The f32 rounding of a body's speed over `steps` steps on `points` points, as a fraction of
/// the speed: the solve moves the speed once for each point in each of a substep's two passes,
/// and each move rounds to half an ulp of it. Times the body's momentum it bounds the error of
/// a momentum read from the speed, and times twice its energy that of an energy.
fn rounding(app: &App, points: usize, steps: usize) -> f32 {
    f32::EPSILON * (points * steps) as f32 / substep(app)
}

/// A point read in no group and no budget returns what Avian's rigid contact does, and reads
/// it. The building model's work account (`breakage::bounded::settle`) reads every point on a
/// standing cell, in a bounded block or not. Fixture: a 2 kg box at 1 m/s, which the slab stops.
/// Mutant: the seam's systems skipped on a step with no group (the points read nothing).
#[test]
fn a_point_read_in_no_group_reads_what_it_returned() {
    let (mass, approach) = (2.0_f32, 1.0);
    let mut app = world(Hands::default());
    let cube = body(&mut app, Shape::Four, mass, INTO, approach);
    step(&mut app);
    let lost = (approach - down(&app, cube)) * mass;
    let read = entry(&app, cube).returned;
    assert!(
        lost > 0.5 * mass * approach && (read - lost).abs() < rounding(&app, 4, 1) * lost,
        "the box lost {lost} N·s in the step and its points read {read}"
    );
}

/// A bounded point returns its bound over the step, whatever the approach needs, and what its
/// points read as returned is the momentum the body lost; a bound the approach does not reach
/// leaves the contact rigid. Fixture: a 2 kg box closing on the slab at 1 m/s, which a rigid
/// contact stops with 2 N·s through its four face points; each bounded at 0.125 N·s, it loses
/// 0.5 N·s a step. Mutants: the fork's clamp removed (the box stops); the bound given whole to
/// each substep (the loss is the substep count times larger); the reading taken from the last
/// substep alone (it reads the substep count times too little).
#[test]
fn a_bounded_point_returns_its_bound_over_the_step() {
    let (mass, bound, approach) = (2.0_f32, 0.125, 1.0);
    let mut allowed = 0.0;
    let mut run = |each: Option<f32>| {
        let mut app = world(Hands { each, ..default() });
        let cube = body(&mut app, Shape::Four, mass, INTO, approach);
        allowed = rounding(&app, 4, 1) * mass * approach;
        let mut steps = vec![(approach, 0.0)];
        for _ in 0..3 {
            step(&mut app);
            steps.push((down(&app, cube), entry(&app, cube).returned));
        }
        steps
    };
    for free in [None, Some(mass * approach)] {
        let stopped = run(free);
        assert!(
            stopped[3].0 <= 0.0,
            "a contact bounded at {free:?} stops the box: {stopped:?}"
        );
    }
    let bounded = run(Some(bound));
    for pair in bounded.windows(2) {
        let (loss, returned) = ((pair[0].0 - pair[1].0) * mass, pair[1].1);
        assert!(
            (loss - 4.0 * bound).abs() < allowed && (returned - loss).abs() < allowed,
            "each step returns {} N·s and reads it: {bounded:?}",
            4.0 * bound
        );
    }
}

/// A group returns its bound over the step between its points, wherever the load falls: the
/// sum never passes the bound, and a load on some of the points still meets all of it. Relied
/// on by the game's building model, where a block of cells returns what its bonds hold
/// whichever of its faces a body presses. Fixture: a 2,000 kg body closing at 4 m/s on four
/// points and on sixteen, all one group of 100 N·s; a step takes 100 N·s of its momentum.
/// Mutants: each wanting point handed all the group has left (a step takes more than the
/// bound); the bound split evenly among manifolds whatever their points carry.
#[test]
fn a_group_returns_its_bound_over_the_step_whatever_the_split() {
    let (mass, approach, bound) = (2_000.0_f32, 4.0, 100.0);
    for shape in [Shape::Four, Shape::Sixteen] {
        let mut app = world(Hands {
            group: Some(bound),
            ..default()
        });
        let it = body(&mut app, shape, mass, INTO, approach);
        let mut speeds = vec![approach];
        for _ in 0..4 {
            step(&mut app);
            speeds.push(down(&app, it));
        }
        let losses: Vec<f32> = (speeds.windows(2))
            .map(|pair| (pair[0] - pair[1]) * mass)
            .collect();
        let allowed = rounding(&app, shape.points(), 1) * mass * approach;
        assert!(
            losses.iter().all(|&loss| loss <= bound + allowed),
            "{shape:?}: no step returns more than {bound} N·s: {losses:?}"
        );
        // The first step's first substep deals the group before any solve says where the load
        // falls; a later step's starts from what the points carried into it.
        assert!(
            losses[1..].iter().all(|&loss| loss >= bound - allowed),
            "{shape:?}: each later step returns {bound} N·s: {losses:?}"
        );
    }
}

/// A body with more energy than a budget has left loses that work, whatever the number of its
/// points, and its points count it in the step; they then do no more. The building model's
/// block parts on that count, having cost the body its bonds' work and no more
/// (`breakage::bounded::settle`). Fixture: a 2,000 kg body at 4 m/s, 16,000 J, on a budget of
/// 400 J and no impulse bound. The budget is met in the step's first substep by the first point
/// solved, so no other point's work is estimated: the loss and the count are the budget but
/// for f32 ([`rounding`]). Mutants: the limiter's cut removed (the body stops and loses
/// 16,000 J); a budget for each point (four points lose four times one's); the count taken
/// from the impulse and the speed after it alone (MEASURED: it reads 397.5 J of the 400).
#[test]
fn a_body_that_can_pay_loses_the_budget_whatever_its_points() {
    let (mass, approach, work) = (2_000.0_f32, 4.0, 400.0);
    for shape in [Shape::One, Shape::Four, Shape::Sixteen] {
        let mut app = world(Hands {
            work: Some(work),
            ..default()
        });
        let it = body(&mut app, shape, mass, INTO, approach);
        let had = energy(&app, it, mass);
        step(&mut app);
        let (lost, entry) = (had - energy(&app, it, mass), entry(&app, it));
        assert_eq!(entry.points.len(), shape.points(), "fixture: {shape:?}");
        let allowed = 2.0 * f64::from(rounding(&app, shape.points(), 1)) * had;
        assert!(
            (lost - f64::from(work)).abs() < allowed && entry.spent == Some(1),
            "{shape:?} lost {lost} J to a budget of {work} J, counted on step {:?}",
            entry.spent
        );
        assert!(
            (f64::from(entry.done) - lost).abs() < allowed,
            "{shape:?}: its points counted {} J of the {lost} J it lost",
            entry.done
        );
    }
}

/// A body with less energy than a budget has left is stopped, the budget stands, and the
/// budget's points have done what the body had. The building model relies on it for a loose
/// piece meeting a standing block: its own energy is all it can cost the block's bonds, and it
/// never parts them by arriving. Fixture: a 2 kg body at 1 m/s, 1 J, on a budget of 400 J, on
/// one point and on four. Mutants: a budget spent by any impulse it bounds; the count taken
/// from the impulse and the speed after it alone (it reads nothing for a stopped body).
#[test]
fn a_body_that_cannot_pay_is_stopped_and_counted_for_what_it_had() {
    let (mass, approach, work) = (2.0_f32, 1.0, 400.0);
    for shape in [Shape::One, Shape::Four] {
        let mut app = world(Hands {
            work: Some(work),
            ..default()
        });
        let it = body(&mut app, shape, mass, INTO, approach);
        let had = energy(&app, it, mass);
        for _ in 0..4 {
            step(&mut app);
        }
        let (has, entry) = (energy(&app, it, mass), entry(&app, it));
        assert!(
            down(&app, it) <= 0.0 && entry.spent.is_none(),
            "{shape:?} is stopped and the budget stands: {} m/s, spent {:?}",
            down(&app, it),
            entry.spent
        );
        // The count is the energy lost through the points. The body keeps what backing out of
        // the fixture's overlap gave it: less than the overlap a step.
        let allowed = 2.0 * f64::from(rounding(&app, shape.points(), 4)) * had;
        let step = app.world().resource::<Time<Fixed>>().timestep();
        let backing = 0.5 * f64::from(mass * (INTO / step.as_secs_f32()).powi(2));
        assert!(
            (f64::from(entry.done) - (had - has)).abs() < allowed && has < backing,
            "{shape:?} had {had} J, has {has} J, and its points counted {} J",
            entry.done
        );
    }
}

/// Points held at their group's bound do their budget's work over the steps it takes, whatever
/// their number, and no more. The building model relies on it for a body its block's strength
/// does not stop: the body loses the bonds' work, not the strength times its travel. Fixture: a
/// 2,000 kg body at 4 m/s on a group of 100 N·s a step and a budget of 1,000 J, so a step does
/// about 400 J and the count reaches the budget on the third. Premise: nothing else touches
/// the body, so its points' impulses alone move the speed they close at, and the work the cut
/// leaves a point is the work it does: the loss and the count are the budget but for f32
/// ([`rounding`]; MEASURED: 1,000.01, 999.99 and 1,000.21 J on one, four and sixteen points).
/// Mutant: the limiter's cut removed, the budget read between substeps alone (MEASURED: the
/// one point loses 1,178 J).
#[test]
fn points_held_at_their_bound_do_the_budget_whatever_their_number() {
    let (mass, approach, bound, work) = (2_000.0_f32, 4.0, 100.0, 1_000.0);
    for shape in [Shape::One, Shape::Four, Shape::Sixteen] {
        let mut app = world(Hands {
            group: Some(bound),
            work: Some(work),
            ..default()
        });
        let it = body(&mut app, shape, mass, INTO, approach);
        let had = energy(&app, it, mass);
        for _ in 0..3 {
            step(&mut app);
        }
        let (lost, entry) = (had - energy(&app, it, mass), entry(&app, it));
        let allowed = 2.0 * f64::from(rounding(&app, shape.points(), 3)) * had;
        assert!(
            (lost - f64::from(work)).abs() < allowed && entry.spent == Some(3),
            "{shape:?} lost {lost} J to a budget of {work} J, counted on step {:?}",
            entry.spent
        );
        assert!(
            (f64::from(entry.done) - lost).abs() < allowed,
            "{shape:?}: its points counted {} J of the {lost} J it lost",
            entry.done
        );
    }
}

/// A heavy body pressing a light one against the slab loses the budget of the light one's
/// points, though the light body alone has a fraction of it, whatever the number of its
/// points. The building model relies on it for a loose piece driven into a standing block: the
/// block gives to whatever does the work through the piece, and costs it the bonds' work.
/// Fixture: a 2 kg box on the slab, on four points and on sixteen, under a 2,000 kg box, both
/// at 4 m/s; the light box's points in a group of 100 N·s a step and a budget of 1,000 J, the
/// contact between the boxes Avian's. A step does about 400 J, so the count is due on the
/// third. The driver's push on the piece is solved before the piece's points on the slab and
/// again in the next pass, so the speed a point's cut reads is not the speed the substep ends
/// at, and the substep that reaches the budget may pass it. Premise of the bound on that: the
/// points return at most the group's bound a substep, closing no faster than the approach
/// (MEASURED: the count is 1,015.5 J on four points and 1,018.0 J on sixteen). The pair's
/// energy is counted too: each solve that raises the points' impulse parts the boxes and lets
/// the contact between them rejoin them, losing at most the light box's energy at the
/// approach speed; the impulse rises by at most the light box's momentum a solve, so the rise
/// to the bound takes at most the bound a substep over that momentum, in solves. Mutants: the
/// work judged against the pressed body's own energy (the budget is never met and the heavy
/// box stops, losing 16,000 J); the limiter's cut removed (MEASURED: the points do 1,122 J).
///
/// The driver's mass over the piece's is 1,000 here. From about 10,000 Avian's contact between
/// the two lets the driver into the piece, the piece's points close at no speed and count
/// nothing, and the count comes late or not at all (MEASURED, `work-form.md`, the fix pass).
#[test]
fn a_light_body_pressed_by_a_heavy_one_costs_the_pair_the_budget() {
    let (light, heavy, approach, bound, work) = (2.0_f32, 2_000.0, 4.0, 100.0, 1_000.0);
    for shape in [Shape::Four, Shape::Sixteen] {
        let mut app = world(Hands {
            group: Some(bound),
            work: Some(work),
            ..default()
        });
        let piece = body(&mut app, shape, light, INTO, approach);
        let driver = body(&mut app, Shape::Four, heavy, 1.0 + 2.0 * INTO, approach);
        let energy = |app: &App| energy(app, piece, light) + energy(app, driver, heavy);
        let had = energy(&app);
        for _ in 0..3 {
            step(&mut app);
        }
        let entry = entry(&app, piece);
        let (lost, done) = (had - energy(&app), f64::from(entry.done));
        let a_substep = f64::from(bound * substep(&app) * approach);
        let rise = (bound * substep(&app) / (light * approach)).ceil();
        let rejoined = f64::from(rise * 0.5 * light * approach * approach);
        assert!(
            entry.spent == Some(3) && done >= f64::from(work) && done < f64::from(work) + a_substep,
            "{shape:?}: the light box's points did {done} J of a budget of {work} J, counted \
             on step {:?}",
            entry.spent
        );
        assert!(
            lost > done - a_substep && lost < done + rejoined,
            "{shape:?}: the pair lost {lost} J, its points did {done} J, and rejoining takes \
             up to {rejoined} J"
        );
    }
}

/// Fixture: the gravity of the test at rest, m/s².
const GRAVITY: f32 = 9.81;

/// A body at rest on the slab, and a light body a heavy one holds still against it, do no work
/// over many steps, and their budgets stand, however hard they press. The building model
/// relies on it for everything that stands or lies on standing cells: rest never parts a
/// block. Fixture: under gravity, a 2,000 kg box on the slab, and a 2 kg box under a 2,000 kg
/// one; a budget of 100 J on the points on the slab, read over 64 steps after 64 to settle.
/// The allowance: a substep's gravity adds `g dt / substeps` to the speed and the points take
/// it back, each move rounding it ([`rounding`]), so a substep ends within that rounding of
/// rest and its points, which return the weight's impulse `m g dt / substeps`, do at most
/// that impulse times the residual speed. Mutants: the cut taken at the least impulse that would do the work
/// left, where the solve found a greater one that does none (the light box's points hold the
/// heavy one's weight, and at a lesser impulse the light box alone would fly into the slab:
/// MEASURED, the budget reads spent on step 19 of the rest); the limiter reading no speed from
/// before the substep's forces (MEASURED: spent on step 4).
#[test]
fn a_body_at_rest_or_held_still_does_no_work() {
    let work = 100.0;
    for held in [false, true] {
        let mut app = world(Hands {
            work: Some(work),
            ..default()
        });
        app.insert_resource(Gravity(Vec3::NEG_Y * GRAVITY));
        let (lower, weight) = if held {
            let lower = body(&mut app, Shape::Four, 2.0, INTO, 0.0);
            body(&mut app, Shape::Four, 2_000.0, 1.0 + 2.0 * INTO, 0.0);
            (lower, 2_002.0)
        } else {
            (body(&mut app, Shape::Four, 2_000.0, INTO, 0.0), 2_000.0)
        };
        for _ in 0..64 {
            step(&mut app);
        }
        let settled = entry(&app, lower).done;
        for _ in 0..64 {
            step(&mut app);
        }
        let entry = entry(&app, lower);
        let step = app.world().resource::<Time<Fixed>>().timestep();
        let added = GRAVITY * step.as_secs_f32() * substep(&app);
        let allowed = rounding(&app, 4, 64) * weight * added * added;
        assert!(
            entry.done - settled < allowed && entry.spent.is_none(),
            "held {held}: the points did {} J in 64 steps at rest, spent {:?}",
            entry.done - settled,
            entry.spent
        );
    }
}
