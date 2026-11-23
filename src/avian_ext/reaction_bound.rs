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
//! takes the least of them. A manifold point in no group is Avian's rigid contact bit for bit.
//!
//! Who relies on it: the game's building model (`construction::breakage::bounded`), where a block of
//! standing cells returns no more than its bonds can, wherever a body presses it.

use avian3d::collision::contact_types::ContactId;
use avian3d::dynamics::integrator::IntegrationSystems;
use avian3d::dynamics::solver::constraint_graph::ConstraintGraph;
use avian3d::dynamics::solver::contact::ContactNormalPart;
use avian3d::dynamics::solver::schedule::{SolverSystems, SubstepSolverSystems};
use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::tasks::{ComputeTaskPool, ParallelSlice};

/// The most a group returns over a step: an impulse, N·s, and the work it does on the bodies
/// closing through it, J. A group's impulse is finite, since its deal carries the work; an
/// infinite work leaves that side rigid.
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
}

/// A read manifold's prepared constraint, and of each of its points this substep's bound, the
/// groups that dealt it (its impulse, its work), the normal impulse it has returned over the
/// step, N·s, and the group whose bound the step's last solve left it holding, if any.
#[derive(Debug)]
struct Manifold {
    /// Colour and index in it; `None` for a manifold the step prepared no constraint for.
    slot: Option<(usize, usize)>,
    points: Vec<Bound>,
    dealers: Vec<[Option<usize>; 2]>,
    returned: Vec<f32>,
    limited: Vec<Option<usize>>,
}

/// This step's groups, in the order their writers named them.
#[derive(Resource, Default, Debug)]
pub struct Bounds {
    groups: Vec<Group>,
    manifolds: HashMap<(u32, usize), Manifold>,
    /// Whether this step's first substep was dealt.
    dealt: bool,
}

impl Bounds {
    /// Reads what `point` of `manifold` of `contact` returns over the step ([`Self::impulse`]);
    /// it stays rigid unless a group bounds it.
    pub fn read(&mut self, contact: ContactId, manifold: usize, point: usize) {
        let manifold = (self.manifolds)
            .entry((contact.0, manifold))
            .or_insert(Manifold {
                slot: None,
                points: Vec::new(),
                dealers: Vec::new(),
                returned: Vec::new(),
                limited: Vec::new(),
            });
        if manifold.points.len() <= point {
            manifold.points.resize(point + 1, Bound::RIGID);
            manifold.returned.resize(point + 1, 0.0);
            manifold.dealers.resize(point + 1, [None; 2]);
            manifold.limited.resize(point + 1, None);
        }
    }

    /// Bounds the sum of `members`' normal impulses, each times its lean, at `most.impulse` over
    /// the step, and their work at `most.work`, and reads each ([`Self::read`]); returns the
    /// group's index for [`Self::returned`].
    pub fn group(&mut self, members: Vec<Member>, most: Bound) -> usize {
        assert!(
            most.impulse >= 0.0 && most.work >= 0.0,
            "a group's reaction bound {most:?} holds a negative or NaN: it pushes or bears nothing"
        );
        assert!(
            members.iter().all(|member| member.lean > 0.0),
            "a group's members lean its way: {members:?}"
        );
        // Nothing a point carries measures the work it took, so the work follows the impulse's
        // deal; without an impulse bound there is no deal to follow.
        assert!(
            most.impulse.is_finite(),
            "a group bounded in work alone has no deal: {most:?}"
        );
        for member in &members {
            self.read(member.contact, member.manifold, member.point);
        }
        self.groups.push(Group { members, most });
        self.groups.len() - 1
    }

    /// The normal impulse a read point returned over the last step, N·s: the sum of what each
    /// substep applied through it, and restitution's increment.
    pub fn impulse(&self, contact: ContactId, manifold: usize, point: usize) -> f32 {
        self.manifolds[&(contact.0, manifold)].returned[point]
    }

    /// What group `group`'s points returned between them over the last step, each times its
    /// lean, N·s: the operator's reading.
    pub fn returned(&self, group: usize) -> f32 {
        (self.groups[group].members.iter())
            .map(|member| self.impulse(member.contact, member.manifold, member.point) * member.lean)
            .sum()
    }

    /// Whether group `group` did not stop what closed on it over the last step: the step's last
    /// solve left one of its points holding an impulse at the bound this group dealt it, so the
    /// approach there wanted more than the group could return. A point held at another group's
    /// lesser bound says nothing of this one.
    pub fn limited(&self, group: usize) -> bool {
        (self.groups[group].members.iter()).any(|member| {
            self.manifolds[&(member.contact.0, member.manifold)].limited[member.point]
                == Some(group)
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
    bounds.dealt = false;
}

/// Finds each bounded manifold's prepared constraint.
fn locate(mut bounds: ResMut<Bounds>, constraints: Res<ConstraintGraph>) {
    if bounds.groups.is_empty() {
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
                        manifold.points.len() <= constraint.points.len(),
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

/// `member`'s prepared normal part; none when the step prepared its manifold no constraint.
fn part<'a>(
    member: &Member,
    manifolds: &HashMap<(u32, usize), Manifold>,
    constraints: &'a ConstraintGraph,
) -> Option<&'a ContactNormalPart> {
    let (color, index) = manifolds[&(member.contact.0, member.manifold)].slot?;
    Some(&constraints.colors[color].contact_constraints[index].points[member.point].normal_part)
}

/// Before each substep's solve: deals every group's bound over one substep to its points (the
/// module doc) and cuts what a point carries to its bound. Another writer of the substep's
/// constraints orders itself against this system. Groups are dealt one after another, in their
/// order: they share points, a shared point takes the least of its groups' deals, and of equal
/// deals the first group's is the one it is held by ([`Bounds::limited`]).
pub(crate) fn deal(
    mut bounds: ResMut<Bounds>,
    mut constraints: ResMut<ConstraintGraph>,
    substeps: Res<SubstepCount>,
    mut points: Local<Vec<(f32, bool)>>,
) {
    if bounds.groups.is_empty() {
        return;
    }
    let Bounds {
        groups,
        manifolds,
        dealt,
    } = &mut *bounds;
    let substep = (substeps.0 as f32).recip();
    for manifold in manifolds.values_mut() {
        manifold.points.fill(Bound::RIGID);
        manifold.dealers.fill([None; 2]);
        if *dealt {
            manifold.add_returned(&constraints); // What the last substep applied.
        }
    }
    for (index, group) in groups.iter().enumerate() {
        points.clear();
        // A member whose manifold has no constraint this step carries and wants nothing.
        points.extend(group.members.iter().map(|member| {
            part(member, manifolds, &constraints).map_or((0.0, false), |part| {
                (part.impulse * member.lean, part.clamped)
            })
        }));
        let impulse = group.most.impulse * substep;
        let work = group.most.work * substep;
        for (member, share) in (group.members.iter()).zip(shares(&points, !*dealt, impulse)) {
            let manifold = manifolds
                .get_mut(&(member.contact.0, member.manifold))
                .expect("the group's manifolds are in the store");
            let (point, dealers) = (
                &mut manifold.points[member.point],
                &mut manifold.dealers[member.point],
            );
            let dealt = Bound {
                impulse: impulse * share / member.lean,
                work: work * share,
            };
            if dealt.impulse < point.impulse {
                (point.impulse, dealers[0]) = (dealt.impulse, Some(index));
            }
            if dealt.work < point.work {
                (point.work, dealers[1]) = (dealt.work, Some(index));
            }
        }
    }
    *dealt = true;
    for manifold in manifolds.values() {
        let Some((color, index)) = manifold.slot else {
            continue;
        };
        let constraint = &mut constraints.colors[color].contact_constraints[index];
        for (point, bound) in constraint.points.iter_mut().zip(&manifold.points) {
            point.normal_part.max_impulse = bound.impulse;
            point.normal_part.max_work = bound.work;
            point.normal_part.impulse = point.normal_part.impulse.min(bound.impulse);
        }
    }
}

impl Manifold {
    /// Adds what each point carries to what it has returned; nothing for a manifold the step
    /// prepared no constraint for.
    fn add_returned(&mut self, constraints: &ConstraintGraph) {
        let Some((color, index)) = self.slot else {
            return;
        };
        let constraint = &constraints.colors[color].contact_constraints[index];
        for (returned, point) in self.returned.iter_mut().zip(&constraint.points) {
            *returned += point.normal_part.impulse;
        }
    }
}

/// After the step's last solve: what the last substep and restitution applied
/// ([`Bounds::impulse`]), and which points that solve left at their bound ([`Bounds::limited`]).
fn read_bounds(mut bounds: ResMut<Bounds>, constraints: Res<ConstraintGraph>) {
    for manifold in bounds.manifolds.values_mut() {
        manifold.add_returned(&constraints);
        let Some((color, index)) = manifold.slot else {
            continue; // No constraint: its points returned nothing and met no bound.
        };
        let constraint = &constraints.colors[color].contact_constraints[index];
        for ((limited, dealers), point) in
            (manifold.limited.iter_mut().zip(&manifold.dealers)).zip(&constraint.points)
        {
            let part = &point.normal_part;
            // A point dealt nothing sits at its bound without holding anything back. The solve
            // clamps at the lesser of the impulse bound and the work bound at its closing speed:
            // short of the first, it was held by the second.
            *limited = (part.clamped && part.impulse > 0.0)
                .then(|| dealers[usize::from(part.impulse < part.max_impulse)])
                .flatten();
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
