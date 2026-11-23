//! Bounded contact: a group of contact points returns, over a physics step, at most an impulse
//! its writer names, summed over the points along their leans; and the points of a budget do, on
//! the bodies closing through them, at most a work its writer names, in place of Avian's rigid
//! contact, which returns whatever impulse stops the approach. A writer names groups and budgets
//! in [`Bounds`] after the narrow phase and before Avian prepares the step's contact constraints
//! (`SolverSystems::PrepareContactConstraints`). The store is cleared at the start of every
//! physics step, so nothing named outlives its step; what each point returned and did over the
//! step ([`Bounds::impulse`], [`Bounds::work`]) reads until the next step starts.
//!
//! **Impulse.** The solve bounds one point at a time: the fork's `ContactNormalPart::max_impulse`
//! (its `OVERMATCH_PATCH.md`, "Bounded contact") holds a point's accumulated impulse, which is
//! what one substep applies (the warm start reapplies the whole of it,
//! `SolverConfig::warm_start_coefficient` 1). So a group's bound is dealt to its points before
//! every substep ([`deal`]): each keeps what it carries, and what the group has left goes to the
//! points the last solve left wanting more ([`shares`]). The points' bounds always sum to the
//! group's, whatever the split, so the group returns at most its bound over the step and Coulomb
//! friction, which the solve clamps to the normal impulse, at most its coefficient times that. A
//! point in several groups takes the least of them.
//!
//! **Work.** The work an impulse `p` does over a substep on bodies closing at a point at `c0`
//! before the substep's forces and `c1` after its last solve is `p · ½(c0 + c1)`: the bodies'
//! kinetic-energy change is each impulse on them times the mean of the speeds across it, so the
//! speeds carry every other point, body and force. [`account`] sums it over a budget's points,
//! signed, after every substep; the sum floored at nothing is the budget's count for the
//! substep, taken from what the budget has left and given to the points that did positive work
//! by their share of it ([`Bounds::work`]). Inside a solve the fork's limiter hook ([`limit`]) cuts a
//! point whose impulse does more than its budget has left after the budget's other points'
//! work this substep, each at its last solve: with `u` the closing speed there less the point's
//! own impulse and `K` its inverse effective mass, `p` ends the solve closing at `u − pK`, so
//! it does `½ p (c0 + u − pK)`, and the cut is to that solved for `p` ([`cap`]). An impulse
//! that does less is left alone: a body the contact stops did what it had. A budget is one
//! dynamic body's; Avian solves two constraints of one dynamic body in turn (its graph colours
//! hold a body once, and the overflow colour is serial), so the budget's cell, which lives on
//! that solver body, has one writer at a time. A budget whose count has reached its work has
//! nothing left: its points do no more work for the rest of the step.
//!
//! A manifold point in no group and no budget is Avian's rigid contact bit for bit, and may
//! still be read ([`Bounds::read`]).
//!
//! Who relies on it: the game's building model (`construction::breakage::bounded`), where a block of
//! standing cells returns no more than its bonds can, wherever a body presses it, until the
//! body has done the work its bonds take.

use avian3d::collision::contact_types::ContactId;
use avian3d::dynamics::integrator::IntegrationSystems;
use avian3d::dynamics::solver::constraint_graph::ConstraintGraph;
use avian3d::dynamics::solver::contact::{ContactConstraint, ContactNormalPart, NORMAL_LIMIT};
use avian3d::dynamics::solver::schedule::{SolverSystems, SubstepSolverSystems};
use avian3d::dynamics::solver::solver_body::{SolverBodies, SolverBody};
use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::tasks::{ComputeTaskPool, ParallelSlice};

/// One contact point: its contact pair, manifold and point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct At {
    pub contact: ContactId,
    pub manifold: usize,
    pub point: usize,
}

/// One point of a group, and its lean: the share of its normal impulse that counts against the
/// group's (positive).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Member {
    pub at: At,
    pub lean: f32,
}

#[derive(Debug)]
struct Group {
    members: Vec<Member>,
    /// The most its points return between them over the step, each times its lean, N·s.
    most: f32,
}

/// The work one dynamic body's points may do between them over the step, J.
#[derive(Debug)]
struct Budget {
    body: Entity,
    points: Vec<At>,
    work: f32,
}

/// One read point: this substep's impulse bound, the least its groups dealt it; the normal
/// impulse it has returned over the step, N·s; and its share of its budget's count, J.
#[derive(Debug, Clone, Copy)]
struct Record {
    bound: f32,
    returned: f32,
    work: f32,
}

/// A read manifold: its prepared constraint and its points' records.
#[derive(Debug)]
struct Manifold {
    /// Colour and index in it; `None` for a manifold the step prepared no constraint for.
    slot: Option<(usize, usize)>,
    /// Where its points' records start in [`Bounds::points`], and how many.
    first: usize,
    len: usize,
}

impl Manifold {
    fn records(&self) -> std::ops::Range<usize> {
        self.first..self.first + self.len
    }
}

/// This step's groups and budgets, in the order their writers named them, and every point it
/// reads. The storage is kept from step to step.
#[derive(Resource, Default, Debug)]
pub struct Bounds {
    groups: Vec<Group>,
    budgets: Vec<Budget>,
    manifolds: HashMap<(u32, usize), Manifold>,
    points: Vec<Record>,
    /// Whether this step's first substep was dealt.
    dealt: bool,
    /// Whether this step's first substep was opened ([`account`]).
    opened: bool,
}

/// A point is read by the step that names it and until the next step starts.
const READ: &str = "the point was read in the last physics step";

impl Bounds {
    /// Reads what `at` returns and does over the step ([`Self::impulse`], [`Self::work`]); it
    /// stays rigid unless a group or a budget bounds it.
    pub fn read(&mut self, at: At) {
        let records = &mut self.points;
        let manifold = (self.manifolds)
            .entry((at.contact.0, at.manifold))
            .or_insert(Manifold {
                slot: None,
                first: records.len(),
                len: 0,
            });
        if manifold.len <= at.point {
            if manifold.records().end != records.len() {
                // Another manifold's records follow: this one's move to the end to grow.
                records.extend_from_within(manifold.records());
                manifold.first = records.len() - manifold.len;
            }
            manifold.len = at.point + 1;
            records.resize(
                manifold.records().end,
                Record {
                    bound: f32::INFINITY,
                    returned: 0.0,
                    work: 0.0,
                },
            );
        }
    }

    /// Bounds the sum of `members`' normal impulses, each times its lean, at `most` N·s over the
    /// step, and reads each ([`Self::read`]); returns the group's index for [`Self::returned`].
    pub fn group(&mut self, members: Vec<Member>, most: f32) -> usize {
        assert!(
            most >= 0.0 && most.is_finite(),
            "a group's bound of {most} N·s is negative, NaN or no bound: it pushes, bears \
             nothing, or has no share to deal"
        );
        assert!(
            members.iter().all(|member| member.lean > 0.0),
            "a group's members lean its way: {members:?}"
        );
        for member in &members {
            self.read(member.at);
        }
        self.groups.push(Group { members, most });
        self.groups.len() - 1
    }

    /// Bounds the work `points`, all of dynamic body `body`, do between them over the step at
    /// `work` J, and reads each ([`Self::read`]). A point is in one budget.
    pub fn budget(&mut self, body: Entity, points: Vec<At>, work: f32) {
        assert!(work >= 0.0, "a budget of {work} J is negative or NaN");
        for &at in &points {
            self.read(at);
        }
        (self.budgets).push(Budget { body, points, work });
    }

    fn record(&self, at: At) -> &Record {
        let manifold = (self.manifolds.get(&(at.contact.0, at.manifold))).expect(READ);
        assert!(at.point < manifold.len, "{READ}");
        &self.points[manifold.first + at.point]
    }

    /// The normal impulse a read point returned over the last step, N·s: the sum of what each
    /// substep applied through it, and restitution's increment.
    pub fn impulse(&self, at: At) -> f32 {
        self.record(at).returned
    }

    /// A budget's point's share of the work its budget's points did over the last step on the
    /// bodies closing through them, J, never negative (the module doc): the shares of a budget's
    /// points sum to its count. Nothing for a point in no budget. Restitution's increment, which
    /// no pair of speeds spans, is not in it.
    pub fn work(&self, at: At) -> f32 {
        self.record(at).work
    }

    /// What group `group`'s points returned between them over the last step, each times its
    /// lean, N·s: the operator's reading.
    pub fn returned(&self, group: usize) -> f32 {
        (self.groups[group].members.iter())
            .map(|member| self.impulse(member.at) * member.lean)
            .sum()
    }
}

pub(super) fn plugin(app: &mut App) {
    NORMAL_LIMIT.get_or_init(|| limit);
    app.init_resource::<Bounds>();
    app.get_schedule_mut(PhysicsSchedule)
        .expect("avian_ext::plugin is added after Avian's PhysicsPlugins")
        .add_systems((
            clear_bounds.in_set(PhysicsStepSystems::First),
            // `apply_laws` writes the constraints this reads.
            locate
                .after(super::compliance::apply_laws)
                .after(SolverSystems::PrepareContactConstraints)
                .before(SolverSystems::PreSubstep),
            // The last substep's work, before restitution moves the bodies.
            account
                .after(SolverSystems::Substep)
                .before(SolverSystems::PostSubstep),
            read_bounds
                .after(SolverSystems::Restitution)
                .before(SolverSystems::Finalize),
        ));
    app.get_schedule_mut(SubstepSchedule)
        .expect("avian_ext::plugin is added after Avian's PhysicsPlugins")
        .add_systems((
            // After the last substep's last solve and before this one's forces.
            account.before(IntegrationSystems::Velocity),
            // Before the warm start reapplies what each point carries.
            deal.after(IntegrationSystems::Velocity)
                .before(SubstepSolverSystems::WarmStart),
        ));
}

fn clear_bounds(mut bounds: ResMut<Bounds>) {
    bounds.groups.clear();
    bounds.budgets.clear();
    bounds.manifolds.clear();
    bounds.points.clear();
    bounds.dealt = false;
    bounds.opened = false;
}

/// A manifold's prepared constraint; none when the step prepared it none.
fn constraint<'a>(
    manifolds: &HashMap<(u32, usize), Manifold>,
    constraints: &'a ConstraintGraph,
    at: At,
) -> Option<&'a ContactConstraint> {
    let (color, index) = manifolds[&(at.contact.0, at.manifold)].slot?;
    Some(&constraints.colors[color].contact_constraints[index])
}

fn constraint_mut<'a>(
    manifolds: &HashMap<(u32, usize), Manifold>,
    constraints: &'a mut ConstraintGraph,
    at: At,
) -> Option<&'a mut ContactConstraint> {
    let (color, index) = manifolds[&(at.contact.0, at.manifold)].slot?;
    Some(&mut constraints.colors[color].contact_constraints[index])
}

/// The floats a budget keeps on its solver body (`SolverBody::limit_scratch`): the work it has
/// left at the substep's start, J, and what its points do this substep, each at its last solve.
const CELL: usize = 2;

/// Finds each read manifold's prepared constraint, bounded or not, and gives each budget its
/// cell on its body and each of its points the cell's token: the cell's index on the body, and
/// in the low bit which of the constraint's bodies holds it.
fn locate(
    mut bounds: ResMut<Bounds>,
    mut constraints: ResMut<ConstraintGraph>,
    mut bodies: ResMut<SolverBodies>,
) {
    for body in bodies.bodies_mut() {
        body.limit_scratch.clear();
    }
    if bounds.manifolds.is_empty() {
        return;
    }
    let manifolds = &bounds.manifolds;
    let colors: Vec<(usize, &_)> = constraints.colors.iter().enumerate().collect();
    let found = colors.par_splat_map(ComputeTaskPool::get(), None, |_, colors| {
        let mut found = Vec::new();
        for &(color, constraints) in colors {
            for (index, constraint) in constraints.contact_constraints.iter().enumerate() {
                let key = (constraint.contact_id.0, constraint.manifold_index);
                if let Some(manifold) = manifolds.get(&key) {
                    assert!(
                        manifold.len <= constraint.points.len(),
                        "a writer bounds points its manifold holds"
                    );
                    found.push((key, (color, index)));
                }
            }
        }
        found
    });
    for (key, slot) in found.into_iter().flatten() {
        bounds
            .manifolds
            .get_mut(&key)
            .expect("the key was read from this store")
            .slot = Some(slot);
    }
    let Bounds {
        budgets, manifolds, ..
    } = &*bounds;
    for budget in budgets {
        // The cell's index on its body, once a point's constraint has named the body.
        let mut cell = None;
        for &at in &budget.points {
            let Some(constraint) = constraint_mut(manifolds, &mut constraints, at) else {
                continue; // No constraint: the point returns nothing.
            };
            let indices = [constraint.body_index1, constraint.body_index2];
            let side = (indices.iter())
                .position(|&index| bodies.get_entity(index) == Some(budget.body))
                .expect("a budget's body is a body of each of its points' constraints");
            let cell = *cell.get_or_insert_with(|| {
                let scratch = &mut (bodies.get_mut(indices[side]))
                    .expect("the body was found at this index")
                    .limit_scratch;
                scratch.extend([budget.work, 0.0]);
                scratch.len() / CELL - 1
            });
            let part = &mut constraint.points[at.point].normal_part;
            // A point of no effective mass moves nothing and does no work.
            if part.effective_mass > 0.0 {
                part.limit = (cell << 1 | side) as u32;
            }
        }
    }
}

/// The least accumulated impulse of a point that does `left` J over the substep: the lesser
/// root of `½ p (sum − p k) = left`, `sum` the closing speed before the substep's forces plus
/// the closing speed now less the point's own impulse, m/s, and `k` the point's inverse
/// effective mass. Asked only where some impulse does more, so the root is real.
fn cap(left: f32, sum: f32, k: f32) -> f32 {
    if left <= 0.0 {
        return 0.0;
    }
    // `(sum − √root) / 2k`, in the form that keeps its digits as `k` goes to nothing.
    4.0 * left / (sum + (sum * sum - 8.0 * k * left).max(0.0).sqrt())
}

/// The fork's normal limiter (`NORMAL_LIMIT`), for a point [`locate`] gave a budget's token.
/// `ContactNormalPart::limit_state` is the point's closing speed before the substep's forces
/// and the work it does this substep as of its last solve. A solve
/// found the point's impulse, `increment` more than it held, the bodies closing at `closing`
/// m/s there before it; where that impulse does more work than the point's budget has left
/// after the budget's other points, it is cut to the impulse that does what is left ([`cap`]).
/// The work is judged at the impulse the solve found, never at a lesser one: a body the contact
/// stops did what it had, and a body held still does none, however hard it is pressed.
fn limit(
    part: &mut ContactNormalPart,
    body1: &mut SolverBody,
    body2: &mut SolverBody,
    closing: f32,
    increment: f32,
) -> f32 {
    let body = if part.limit & 1 == 0 { body1 } else { body2 };
    let cell = (part.limit >> 1) as usize * CELL;
    let [before, work] = part.limit_state;
    let left = body.limit_scratch[cell] - (body.limit_scratch[cell + 1] - work);
    let k = part.effective_mass.recip();
    let (found, held) = (part.impulse, part.impulse - increment);
    let sum = before + closing + held * k;
    let does = |impulse: f32| 0.5 * impulse * (sum - impulse * k);
    let cut = does(found) > left;
    if cut {
        // A root lies under `found`; the `min` holds it there against rounding.
        part.impulse = cap(left, sum, k).min(found);
        part.total_impulse -= found - part.impulse;
        // Held by the budget, not at the impulse bound: `deal` does not read it as wanting more.
        part.clamped = false;
    }
    let did = does(part.impulse);
    body.limit_scratch[cell + 1] += did - work;
    part.limit_state = [before, did];
    part.impulse - held
}

/// The bodies' closing speed at point `at` of `constraint`, m/s.
fn closing(constraint: &ContactConstraint, at: usize, bodies: &SolverBodies) -> f32 {
    let point = &constraint.points[at];
    // A static body has no solver body and does not move.
    let velocity = |index, anchor| {
        (bodies.get(index)).map_or(Vec3::ZERO, |body| body.velocity_at_point(anchor))
    };
    let relative = velocity(constraint.body_index2, point.anchor2)
        - velocity(constraint.body_index1, point.anchor1);
    -relative.dot(constraint.normal)
}

/// Between substeps, and after the last: counts the work each budget's points did over the
/// substep that ended (the module doc), takes the count from the budget and gives it to the
/// points; then opens the next substep, each point at its closing speed before the substep's
/// forces and at the work its carried impulse does if the bodies keep that speed, which its
/// first solve corrects. The budgets are walked on one thread, as [`deal`] walks the groups.
fn account(
    mut bounds: ResMut<Bounds>,
    mut constraints: ResMut<ConstraintGraph>,
    mut bodies: ResMut<SolverBodies>,
    mut works: Local<Vec<(usize, f32)>>,
) {
    let Bounds {
        budgets,
        manifolds,
        points,
        opened,
        ..
    } = &mut *bounds;
    for budget in budgets.iter() {
        // Each point's record and the work it did over the substep that ended, signed.
        works.clear();
        let (mut did, mut drawn) = (0.0, 0.0);
        let mut cell = None;
        for &at in &budget.points {
            let Some(constraint) = constraint_mut(manifolds, &mut constraints, at) else {
                continue; // No constraint: the point returned nothing.
            };
            let speed = closing(constraint, at.point, &bodies);
            let indices = [constraint.body_index1, constraint.body_index2];
            let part = &mut constraint.points[at.point].normal_part;
            if part.limit == u32::MAX {
                continue; // No effective mass: no work.
            }
            if *opened {
                let work = part.impulse * 0.5 * (part.limit_state[0] + speed);
                works.push((
                    manifolds[&(at.contact.0, at.manifold)].first + at.point,
                    work,
                ));
                did += work;
            }
            let carried = part.impulse * speed.max(0.0);
            part.limit_state = [speed, carried];
            drawn += carried;
            cell = Some((
                indices[(part.limit & 1) as usize],
                (part.limit >> 1) as usize,
            ));
        }
        let Some((body, cell)) = cell else {
            continue; // No point of it has a constraint this step.
        };
        let scratch = &mut (bodies.get_mut(body))
            .expect("`locate` found the body at this index")
            .limit_scratch[cell * CELL..][..CELL];
        // What one point gave back is taken from what another did; a substep the bodies left
        // on the whole counts nothing.
        let count = did.max(0.0);
        if count > 0.0 {
            let positive: f32 = works.iter().map(|&(_, work)| work.max(0.0)).sum();
            for &(record, work) in works.iter() {
                points[record].work += count * work.max(0.0) / positive;
            }
        }
        scratch[0] -= count;
        scratch[1] = drawn;
    }
    *opened = true;
}

/// Each point's share of a group's bound, given what each carries against it (a point's
/// impulse times its lean) and whether the last solve left it at its bound (`points`), whether
/// this is the step's first deal, and the bound. The shares sum to one, so the points' bounds sum
/// to the group's whatever the split: a point keeps what it carries, and what the group has left
/// goes evenly to the points that want more, or to all when none does. The first deal has no
/// solve behind it and gives what is left to the points by what they carried into the step.
/// Carried past the bound, every point is cut alike.
fn shares(points: &[(f32, bool)], first: bool, bound: f32) -> impl Iterator<Item = f32> + '_ {
    let total: f32 = points.iter().map(|&(carried, _)| carried).sum();
    let count = points.len() as f32;
    let wanters = points.iter().filter(|&&(_, wanting)| wanting).count() as f32;
    points.iter().map(move |&(carried, wanting)| {
        if total >= bound {
            // A bound of nothing deals nothing; the split is then no one's.
            return if total > 0.0 {
                carried / total
            } else {
                count.recip()
            };
        }
        let of_left = if first && total > 0.0 {
            carried / total
        } else if first || wanters == 0.0 {
            count.recip()
        } else if wanting {
            wanters.recip()
        } else {
            0.0
        };
        (carried + (bound - total) * of_left) / bound
    })
}

/// Adds what each of `manifold`'s points carries to what it has returned; nothing for a
/// manifold the step prepared no constraint for.
fn add_returned(manifold: &Manifold, records: &mut [Record], constraints: &ConstraintGraph) {
    let Some((color, index)) = manifold.slot else {
        return;
    };
    let constraint = &constraints.colors[color].contact_constraints[index];
    for (record, point) in records[manifold.records()]
        .iter_mut()
        .zip(&constraint.points)
    {
        record.returned += point.normal_part.impulse;
    }
}

/// Before each substep's solve: deals every group's bound over one substep to its points (the
/// module doc) and cuts what a point carries to its bound. Another writer of the substep's
/// constraints orders itself against this system. A point in several groups takes the least of
/// their deals, which no order of the groups changes; the groups are walked on one thread for
/// their size alone (backlog destruction-13).
pub(crate) fn deal(
    mut bounds: ResMut<Bounds>,
    mut constraints: ResMut<ConstraintGraph>,
    substeps: Res<SubstepCount>,
    mut carried: Local<Vec<(f32, bool)>>,
) {
    if bounds.manifolds.is_empty() {
        return;
    }
    let Bounds {
        groups,
        manifolds,
        points,
        dealt,
        ..
    } = &mut *bounds;
    let substep = (substeps.0 as f32).recip();
    if *dealt {
        for manifold in manifolds.values() {
            add_returned(manifold, points, &constraints); // What the last substep applied.
        }
    }
    for record in points.iter_mut() {
        record.bound = f32::INFINITY;
    }
    for group in groups.iter() {
        carried.clear();
        // A member whose manifold has no constraint this step carries and wants nothing.
        carried.extend(group.members.iter().map(|member| {
            (constraint(manifolds, &constraints, member.at))
                .map(|constraint| &constraint.points[member.at.point].normal_part)
                .map_or((0.0, false), |part| {
                    (part.impulse * member.lean, part.clamped)
                })
        }));
        let impulse = group.most * substep;
        for (member, share) in group.members.iter().zip(shares(&carried, !*dealt, impulse)) {
            let manifold = &manifolds[&(member.at.contact.0, member.at.manifold)];
            let bound = &mut points[manifold.first + member.at.point].bound;
            *bound = bound.min(impulse * share / member.lean);
        }
    }
    *dealt = true;
    for manifold in manifolds.values() {
        let Some((color, index)) = manifold.slot else {
            continue;
        };
        let constraint = &mut constraints.colors[color].contact_constraints[index];
        for (point, record) in (constraint.points.iter_mut()).zip(&points[manifold.records()]) {
            point.normal_part.max_impulse = record.bound;
            point.normal_part.impulse = point.normal_part.impulse.min(record.bound);
        }
    }
}

/// After the step's last solve: what the last substep and restitution applied
/// ([`Bounds::impulse`]).
fn read_bounds(mut bounds: ResMut<Bounds>, constraints: Res<ConstraintGraph>) {
    let Bounds {
        manifolds, points, ..
    } = &mut *bounds;
    for manifold in manifolds.values() {
        add_returned(manifold, points, &constraints);
    }
}

#[cfg(test)]
mod tests {
    use super::shares;

    /// The points' bounds sum to the group's, whatever the split, so a group never returns
    /// more than its bound in a substep; and a point is never dealt less than it carries while
    /// the group has room, so the deal itself cuts no contact. Relied on by the game's
    /// building model: a block's points return its bonds' bound between them. Fixture: splits
    /// of a bound of 8 over four points. Mutants: what is left handed whole to each wanting
    /// point (the sum passes the bound); the bound split evenly whatever is carried (a loaded
    /// point is dealt less than it carries).
    #[test]
    fn a_groups_points_are_dealt_exactly_its_bound_whatever_the_split() {
        let bound = 8.0_f32;
        let splits: [[f32; 4]; 5] = [
            [0.0, 0.0, 0.0, 0.0],
            [6.0, 0.0, 0.0, 0.0],
            [1.0, 2.0, 0.5, 0.0],
            [4.0, 4.0, 0.0, 0.0],
            [9.0, 3.0, 0.0, 0.0],
        ];
        let wants = [
            [false; 4],
            [true, false, false, false],
            [true, true, false, true],
            [true; 4],
        ];
        for carried in splits {
            for wanting in wants {
                for first in [true, false] {
                    let points: Vec<(f32, bool)> = carried.into_iter().zip(wanting).collect();
                    let shares: Vec<f32> = shares(&points, first, bound).collect();
                    let sum: f32 = shares.iter().sum();
                    assert!(
                        (sum - 1.0).abs() < 1e-5,
                        "{carried:?} {wanting:?} first {first}: shares {shares:?} sum to {sum}"
                    );
                    let over = carried.iter().sum::<f32>() > bound;
                    for (&carried, &share) in carried.iter().zip(&shares) {
                        assert!(
                            over || share * bound >= carried - 1e-5,
                            "a point carrying {carried} is dealt {}",
                            share * bound
                        );
                    }
                }
            }
        }
    }
}
