//! GUARD for the bounded contact hook (`vendor/avian3d-0.8.0-overmatch`, `OVERMATCH_PATCH.md`,
//! "Bounded contact") and the game's seam behind it, `crate::avian_ext::reaction_bound`, in the
//! game's building model (game state only). A standing cell returns no more than its bonds can
//! (`construction::bound`), so a bounded point must return its bound over the step and no more,
//! say so, and an unbounded or unreached bound must leave the contact rigid.

use avian3d::prelude::*;
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;

use crate::avian_ext::reaction_bound::{Bound, Bounds, Member};

/// Fixture: the bound handed over; `None` leaves Avian's contact.
#[derive(Resource)]
struct Each(Option<Hand>);

#[derive(Clone, Copy)]
enum Hand {
    /// To every point, as a group of its own.
    Point(Bound),
    /// To all the points of all manifolds between them, as one group.
    Block(Bound),
}

/// The groups the last step named.
#[derive(Resource, Default)]
struct Named(Vec<usize>);

fn hand_over(
    graph: Res<ContactGraph>,
    mut bounds: ResMut<Bounds>,
    hand: Res<Each>,
    mut named: ResMut<Named>,
) {
    named.0.clear();
    let members: Vec<Member> = (graph.iter_active())
        .flat_map(|pair| {
            (pair.manifolds.iter().enumerate()).flat_map(move |(manifold, points)| {
                (0..points.points.len()).map(move |point| Member {
                    contact: pair.contact_id,
                    manifold,
                    point,
                    lean: 1.0,
                })
            })
        })
        .collect();
    match hand.0 {
        None => {}
        Some(Hand::Point(most)) => {
            for member in members {
                named.0.push(bounds.group(vec![member], most));
            }
        }
        Some(Hand::Block(most)) => named.0.push(bounds.group(members, most)),
    }
}

/// Whether every group ended the last step spent, and how many there were.
fn held(bounds: Res<Bounds>, named: Res<Named>) -> (bool, usize) {
    (
        named.0.iter().all(|&group| bounds.spent(group)),
        named.0.len(),
    )
}

/// Fixture: a box of `mass` kg closing at `approach` m/s, without gravity, on a slab whose four
/// face points each take `each`; returns its speed after each of four updates, and what
/// [`held`] reports.
fn press(mass: f32, approach: f32, each: Option<Hand>) -> (Vec<f32>, Vec<(bool, usize)>) {
    press_beside(mass, approach, each, false)
}

/// [`press`]; with `idle`, a second box rests on the slab beside the first, closing on nothing,
/// so its face points carry no load.
fn press_beside(
    mass: f32,
    approach: f32,
    each: Option<Hand>,
    idle: bool,
) -> (Vec<f32>, Vec<(bool, usize)>) {
    {
        let mut app = super::avian_child_pose_churn::physics_app();
        app.insert_resource(Gravity(Vec3::ZERO))
            .insert_resource(Each(each))
            .init_resource::<Named>()
            .add_systems(
                PhysicsSchedule,
                hand_over
                    .after(PhysicsStepSystems::NarrowPhase)
                    .before(PhysicsStepSystems::Solver),
            );
        app.world_mut()
            .spawn((RigidBody::Static, Collider::cuboid(4.0, 1.0, 4.0)));
        // Fixture: the box's lower face 1 mm into the slab's upper face.
        let cube = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::cuboid(1.0, 1.0, 1.0),
                Mass(mass),
                Position(Vec3::Y * 0.999),
                LinearVelocity(Vec3::NEG_Y * approach),
            ))
            .id();
        if idle {
            app.world_mut().spawn((
                RigidBody::Dynamic,
                Collider::cuboid(1.0, 1.0, 1.0),
                Mass(mass),
                Position(Vec3::new(1.5, 0.999, 0.0)),
            ));
        }
        let mut speeds = vec![];
        let mut reports = vec![];
        for _ in 0..4 {
            app.update();
            speeds.push(app.world().get::<LinearVelocity>(cube).unwrap().0.y);
            reports.push(app.world_mut().run_system_once(held).unwrap());
        }
        (speeds, reports)
    }
}

/// A bounded point returns its bound over the step, whatever the approach needs, and reports
/// that it did; a bound the approach does not reach leaves the contact rigid and reports that.
/// Fixture: a 2 kg box closing on a slab at 1 m/s, which a rigid contact stops with 2 N·s
/// through its four face points; each bounded at 0.125 N·s, it loses 0.5 N·s a step.
/// Mutants: the fork's clamp removed (the box stops); the bound given whole to each substep (the
/// loss is the substep count times larger).
#[test]
fn a_bounded_point_returns_its_bound_over_the_step() {
    let (mass, bound, approach) = (2.0, 0.125, 1.0);
    let step = |impulse: Option<f32>| {
        let rigid = Bound::RIGID;
        press(
            mass,
            approach,
            impulse.map(|impulse| Hand::Point(Bound { impulse, ..rigid })),
        )
    };
    let (rigid, _) = step(None);
    assert!(rigid[3] >= 0.0, "a rigid contact stops the box: {rigid:?}");
    let (generous, reports) = step(Some(mass * approach));
    assert!(
        generous[3] >= 0.0 && reports[3] == (false, 4),
        "a bound the approach does not reach stops the box and is not held: {generous:?}, \
         {reports:?}"
    );
    let (bounded, reports) = step(Some(bound));
    assert!(
        // The app's first update runs no physics step.
        reports[1..].iter().all(|&report| report == (true, 4)),
        "each of the four points ends every step at its bound: {reports:?}"
    );
    let losses: Vec<f32> = (bounded.windows(2))
        .map(|pair| (pair[1] - pair[0]) * mass)
        .collect();
    for loss in &losses {
        assert!(
            (loss - 4.0 * bound).abs() < 1e-3 * 4.0 * bound,
            "each step returns {} N·s: {losses:?} from {bounded:?}",
            4.0 * bound
        );
    }
}

/// A point bounded in work takes that work from the body crossing it at the speed the body has:
/// the kinetic energy a step removes is the points' work, whatever the speed, and a body whose
/// energy the work exceeds is stopped. Relied on by the game's building model, where a body
/// shoved inside a step meets a cell at a speed no writer saw. Fixture: a 2,000 kg box, four
/// points of 100 J each; at 4 m/s and at 8 m/s a step removes 400 J within a fifth. Mutant: the
/// fork's work clamp removed (the contact is rigid and stops the box).
#[test]
fn a_work_bounded_point_takes_its_work_at_the_speed_it_is_crossed() {
    let (mass, work) = (2_000.0_f32, 100.0);
    let each = Some(Hand::Point(Bound {
        work,
        ..Bound::RIGID
    }));
    for approach in [4.0, 8.0] {
        let (speeds, reports) = press(mass, approach, each);
        assert!(reports[1..].iter().all(|&report| report == (true, 4)));
        for pair in speeds.windows(2) {
            let taken = 0.5 * mass * (pair[0] * pair[0] - pair[1] * pair[1]);
            assert!(
                (taken - 4.0 * work).abs() < 0.2 * 4.0 * work,
                "a step at {approach} m/s takes {taken} J of {}: {speeds:?}",
                4.0 * work
            );
        }
    }
    // 2 kg at 1 m/s holds 1 J: the points stop it.
    let (stopped, _) = press(2.0, 1.0, each);
    assert!(
        stopped[3] >= 0.0,
        "a body short of the work is stopped: {stopped:?}"
    );
}

/// A group returns its bound over the step between its points, wherever the load falls: the
/// sum never passes the bound, and a load on some of the points still meets all of it. Relied
/// on by the game's building model, where a block of cells returns what its bonds hold
/// whichever of its faces a body presses. Fixture: a 2,000 kg box closing at 4 m/s on a slab,
/// alone and beside a box at rest, every face point on the slab in one group of 100 N·s; a
/// step takes 100 N·s of the closing box's momentum either way. Mutants: each wanting point
/// handed all the group has left (the first step beside the idle box takes more than the
/// bound); the bound split evenly whatever the points carry (beside the idle box a step takes
/// half the bound).
#[test]
fn a_group_returns_its_bound_over_the_step_whatever_the_split() {
    let (mass, approach, bound) = (2_000.0_f32, 4.0, 100.0);
    let most = Bound {
        impulse: bound,
        ..Bound::RIGID
    };
    for idle in [false, true] {
        let (speeds, reports) = press_beside(mass, approach, Some(Hand::Block(most)), idle);
        let losses: Vec<f32> = (speeds.windows(2))
            .map(|pair| (pair[1] - pair[0]) * mass)
            .collect();
        assert!(
            losses.iter().all(|&loss| loss <= bound * 1.001),
            "idle {idle}: no step returns more than {bound} N·s: {losses:?}"
        );
        // A step's first substep deals a new group before any solve says where the load falls.
        assert!(
            losses[1..].iter().all(|&loss| loss >= bound * 0.95),
            "idle {idle}: each later step returns {bound} N·s: {losses:?}, spent {reports:?}"
        );
    }
}
