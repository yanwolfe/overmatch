//! Bounded contact: a group of contact points returns, over a physics step, at most an impulse
//! its writer names, summed over the points along their leans, and does at most a work its
//! writer names on the bodies closing through them, in place of Avian's rigid contact, which
//! returns whatever impulse stops the approach. A writer names a group in [`Bounds`] after the
//! narrow phase and before Avian prepares the step's contact constraints
//! (`SolverSystems::PrepareContactConstraints`). The store is cleared at the start of every
//! physics step, so a bound never outlives its step; what each point returned over the step
//! ([`Bounds::impulse`]) reads until the next step starts.
//!
//! The solve bounds one point at a time: the fork's `ContactNormalPart::max_impulse` and
//! `max_work` (its `OVERMATCH_PATCH.md`, "Bounded contact") hold a point's accumulated impulse,
//! which is what one substep applies (the warm start reapplies the whole of it,
//! `SolverConfig::warm_start_coefficient` 1), and its work at the closing speed the solve finds
//! there. So the group's bound is dealt to its points before every substep ([`deal`]): each
//! keeps what it carries, and what the group has left goes to the points the last solve left
//! wanting more ([`shares`]). The points' bounds always sum to the group's, whatever the split,
//! so the group returns at most its bound over the step and Coulomb friction, which the solve
//! clamps to the normal impulse, at most its coefficient times that. A point in several groups
//! takes the least of them. A manifold point in no group is Avian's rigid contact bit for bit,
//! and may still be read ([`Bounds::read`]).
//!
//! Who relies on it: the game's building model (`construction::breakage::bounded`), where a block of
//! standing cells returns no more than its bonds can, wherever a body presses it.

use avian3d::collision::contact_types::ContactId;
use avian3d::dynamics::integrator::IntegrationSystems;
use avian3d::dynamics::solver::constraint_graph::ConstraintGraph;
use avian3d::dynamics::solver::contact::ContactConstraint;
use avian3d::dynamics::solver::schedule::{SolverSystems, SubstepSolverSystems};
use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::tasks::{ComputeTaskPool, ParallelSlice};

/// The most a group returns over a step: an impulse, N·s, and the work it does on the bodies
/// closing through it, J. An infinite part does not bind: both are infinite on a point no group
/// bounds ([`Bound::RIGID`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bound {
    pub impulse: f32,
    pub work: f32,
}

impl Bound {
    pub const RIGID: Self = Self {
        impulse: f32::INFINITY,
        work: f32::INFINITY,
    };
}

/// One point of a group: its contact pair, manifold and point, and its lean, the share of its
/// normal impulse that counts against the group's (positive).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Member {
    pub contact: ContactId,
    pub manifold: usize,
    pub point: usize,
    pub lean: f32,
}

#[derive(Debug)]
struct Group {
    members: Vec<Member>,
    most: Bound,
    /// Where its members' deals start in [`Bounds::deals`].
    deals: usize,
}

/// One read point: this substep's bound, the least its groups dealt it; the normal impulse it
/// has returned over the step, N·s; and whether the step's last solve left it holding an impulse
/// at its work bound.
#[derive(Debug, Clone, Copy)]
struct Record {
    bound: Bound,
    returned: f32,
    held: bool,
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
    /// Its prepared constraint; none when the step prepared it none.
    fn constraint<'a>(&self, constraints: &'a ConstraintGraph) -> Option<&'a ContactConstraint> {
        let (color, index) = self.slot?;
        Some(&constraints.colors[color].contact_constraints[index])
    }

    fn records(&self) -> std::ops::Range<usize> {
        self.first..self.first + self.len
    }
}

/// This step's groups, in the order their writers named them, and every point it reads. The
/// storage is kept from step to step.
#[derive(Resource, Default, Debug)]
pub struct Bounds {
    groups: Vec<Group>,
    manifolds: HashMap<(u32, usize), Manifold>,
    points: Vec<Record>,
    /// The work each group's last deal gave each of its members, J.
    deals: Vec<f32>,
    /// Whether this step's first substep was dealt.
    dealt: bool,
}

/// A point is read by the step that names it and until the next step starts.
const READ: &str = "the point was read in the last physics step";

impl Bounds {
    /// Reads what `point` of `manifold` of `contact` returns over the step ([`Self::impulse`]);
    /// it stays rigid unless a group bounds it.
    pub fn read(&mut self, contact: ContactId, manifold: usize, point: usize) {
        let records = &mut self.points;
        let manifold = (self.manifolds)
            .entry((contact.0, manifold))
            .or_insert(Manifold {
                slot: None,
                first: records.len(),
                len: 0,
            });
        if manifold.len <= point {
            if manifold.records().end != records.len() {
                // Another manifold's records follow: this one's move to the end to grow.
                records.extend_from_within(manifold.records());
                manifold.first = records.len() - manifold.len;
            }
            manifold.len = point + 1;
            records.resize(
                manifold.records().end,
                Record {
                    bound: Bound::RIGID,
                    returned: 0.0,
                    held: false,
                },
            );
        }
    }

    /// Bounds the sum of `members`' normal impulses, each times its lean, at `most.impulse` over
    /// the step, and their work at `most.work`, and reads each ([`Self::read`]); returns the
    /// group's index for [`Self::returned`] and [`Self::limited`].
    pub fn group(&mut self, members: Vec<Member>, most: Bound) -> usize {
        assert!(
            most.impulse >= 0.0 && most.work >= 0.0,
            "a group's reaction bound {most:?} holds a negative or NaN: it pushes or bears nothing"
        );
        assert!(
            members.iter().all(|member| member.lean > 0.0),
            "a group's members lean its way: {members:?}"
        );
        assert!(
            most.impulse.is_finite(),
            "nothing a point carries measures the work it took, so a group's work follows its \
             impulse's deal, and {most:?} has none"
        );
        for member in &members {
            self.read(member.contact, member.manifold, member.point);
        }
        let deals = self.deals.len();
        self.deals.resize(deals + members.len(), f32::INFINITY);
        (self.groups).push(Group {
            members,
            most,
            deals,
        });
        self.groups.len() - 1
    }

    fn record(&self, contact: ContactId, manifold: usize, point: usize) -> &Record {
        let manifold = self.manifolds.get(&(contact.0, manifold)).expect(READ);
        assert!(point < manifold.len, "{READ}");
        &self.points[manifold.first + point]
    }

    /// The normal impulse a read point returned over the last step, N·s: the sum of what each
    /// substep applied through it, and restitution's increment.
    pub fn impulse(&self, contact: ContactId, manifold: usize, point: usize) -> f32 {
        self.record(contact, manifold, point).returned
    }

    /// What group `group`'s points returned between them over the last step, each times its
    /// lean, N·s: the operator's reading.
    pub fn returned(&self, group: usize) -> f32 {
        (self.groups[group].members.iter())
            .map(|member| self.impulse(member.contact, member.manifold, member.point) * member.lean)
            .sum()
    }

    /// Whether the body closing on group `group` took all the work the group dealt one of its
    /// points and kept closing: the step's last solve left that point holding an impulse at its
    /// work bound, and that bound is this group's deal. A point held at its impulse bound, or at
    /// another group's lesser work, says nothing of this one; groups that dealt a point the same
    /// least work are limited alike.
    pub fn limited(&self, group: usize) -> bool {
        let group = &self.groups[group];
        (group.members.iter().zip(&self.deals[group.deals..])).any(|(member, &dealt)| {
            let record = self.record(member.contact, member.manifold, member.point);
            record.held && dealt <= record.bound.work
        })
    }
}

pub(super) fn plugin(app: &mut App) {
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
            read_bounds
                .after(SolverSystems::Restitution)
                .before(SolverSystems::Finalize),
        ));
    app.get_schedule_mut(SubstepSchedule)
        .expect("avian_ext::plugin is added after Avian's PhysicsPlugins")
        .add_systems(
            // Before the warm start reapplies what each point carries.
            deal.after(IntegrationSystems::Velocity)
                .before(SubstepSolverSystems::WarmStart),
        );
}

fn clear_bounds(mut bounds: ResMut<Bounds>) {
    bounds.groups.clear();
    bounds.manifolds.clear();
    bounds.points.clear();
    bounds.deals.clear();
    bounds.dealt = false;
}

/// Finds each read manifold's prepared constraint, bounded or not.
fn locate(mut bounds: ResMut<Bounds>, constraints: Res<ConstraintGraph>) {
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
    let Some(constraint) = manifold.constraint(constraints) else {
        return;
    };
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
/// their deals, which no order of the groups changes.
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
        deals,
        dealt,
    } = &mut *bounds;
    let substep = (substeps.0 as f32).recip();
    if *dealt {
        for manifold in manifolds.values() {
            add_returned(manifold, points, &constraints); // What the last substep applied.
        }
    }
    for record in points.iter_mut() {
        record.bound = Bound::RIGID;
    }
    for group in groups.iter() {
        carried.clear();
        // A member whose manifold has no constraint this step carries and wants nothing.
        carried.extend(group.members.iter().map(|member| {
            (manifolds[&(member.contact.0, member.manifold)].constraint(&constraints))
                .map(|constraint| &constraint.points[member.point].normal_part)
                .map_or((0.0, false), |part| {
                    (part.impulse * member.lean, part.clamped)
                })
        }));
        let impulse = group.most.impulse * substep;
        let work = group.most.work * substep;
        let shares = shares(&carried, !*dealt, impulse);
        for ((member, share), dealt) in
            (group.members.iter().zip(shares)).zip(&mut deals[group.deals..])
        {
            let manifold = &manifolds[&(member.contact.0, member.manifold)];
            let bound = &mut points[manifold.first + member.point].bound;
            *dealt = work * share;
            bound.impulse = bound.impulse.min(impulse * share / member.lean);
            bound.work = bound.work.min(*dealt);
        }
    }
    *dealt = true;
    for manifold in manifolds.values() {
        let Some((color, index)) = manifold.slot else {
            continue;
        };
        let constraint = &mut constraints.colors[color].contact_constraints[index];
        for (point, record) in (constraint.points.iter_mut()).zip(&points[manifold.records()]) {
            point.normal_part.max_impulse = record.bound.impulse;
            point.normal_part.max_work = record.bound.work;
            point.normal_part.impulse = point.normal_part.impulse.min(record.bound.impulse);
        }
    }
}

/// After the step's last solve: what the last substep and restitution applied
/// ([`Bounds::impulse`]), and which points that solve left at their work bound
/// ([`Bounds::limited`]).
fn read_bounds(mut bounds: ResMut<Bounds>, constraints: Res<ConstraintGraph>) {
    let Bounds {
        manifolds, points, ..
    } = &mut *bounds;
    for manifold in manifolds.values() {
        add_returned(manifold, points, &constraints);
        let Some(constraint) = manifold.constraint(&constraints) else {
            continue; // No constraint: its points returned nothing and met no bound.
        };
        for (record, point) in points[manifold.records()]
            .iter_mut()
            .zip(&constraint.points)
        {
            let part = &point.normal_part;
            // The solve clamps at the lesser of the impulse bound and the work bound at its
            // closing speed (the fork's `solve_impulse`): clamped short of the first, the point
            // is held by the second.
            record.held = part.clamped && part.impulse < part.max_impulse;
        }
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
