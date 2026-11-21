//! Bounded contact: a contact point's normal reaction over a physics step is at most an impulse
//! its writer names, in place of Avian's rigid contact, which returns whatever impulse stops the
//! approach. A writer sets a manifold's bounds in [`Bounds`] after the narrow phase and before
//! Avian prepares the step's contact constraints (`SolverSystems::PrepareContactConstraints`).
//! The store is cleared at the start of every physics step, so a bound never outlives its step;
//! what the step did with it ([`Bounds::held`]) reads until the next step starts.
//!
//! [`apply_bounds`] hands each prepared point the fork's `ContactNormalPart::max_impulse` (the
//! fork's `OVERMATCH_PATCH.md`, "Bounded contact"): its bound over one substep. A point's
//! accumulated impulse is what one substep applies (the warm start reapplies the whole of it,
//! `SolverConfig::warm_start_coefficient` 1), so the point returns at most its bound over the
//! step along its normal, and Coulomb friction, which the solve clamps to the normal impulse, at
//! most its coefficient times that. A manifold without bounds is Avian's rigid contact bit for
//! bit.
//!
//! Who relies on it: the game's building model (`construction::bound`), where a standing cell
//! returns no more than its bonds can.

use avian3d::collision::contact_types::ContactId;
use avian3d::dynamics::solver::constraint_graph::ConstraintGraph;
use avian3d::dynamics::solver::schedule::SolverSystems;
use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::tasks::{ComputeTaskPool, ParallelSlice, ParallelSliceMut};

/// One manifold's points, in manifold order: each point's bound over the step (N·s; infinite for
/// a point left rigid), and whether the step ended with the point at it.
#[derive(Debug)]
struct Points {
    impulse: Vec<f32>,
    held: Vec<bool>,
}

/// This step's bounds, by contact pair and manifold index.
#[derive(Resource, Default, Debug)]
pub struct Bounds(HashMap<(u32, usize), Points>);

#[cfg_attr(
    not(test),
    expect(dead_code, reason = "the game's writer lands with the law's wiring")
)]
impl Bounds {
    /// Bounds each point of manifold `manifold` of pair `contact` at its entry of `impulses`
    /// (N·s over the step, in manifold order; infinite leaves a point rigid), replacing any set
    /// before.
    pub fn set(&mut self, contact: ContactId, manifold: usize, impulses: Vec<f32>) {
        assert!(
            impulses.iter().all(|&impulse| impulse >= 0.0),
            "a contact's reaction bounds {impulses:?} N·s hold a negative or NaN: a point pushes \
             or bears nothing"
        );
        let held = vec![false; impulses.len()];
        self.0.insert(
            (contact.0, manifold),
            Points {
                impulse: impulses,
                held,
            },
        );
    }

    /// Per point of a bounded manifold, whether the last step ended with the point returning all
    /// its bound allows: the body was not stopped there.
    pub fn held(&self, contact: ContactId, manifold: usize) -> Option<&[bool]> {
        (self.0.get(&(contact.0, manifold))).map(|points| points.held.as_slice())
    }
}

pub(super) fn plugin(app: &mut App) {
    app.init_resource::<Bounds>();
    app.get_schedule_mut(PhysicsSchedule)
        .expect("avian_ext::plugin is added after Avian's PhysicsPlugins")
        .add_systems((
            clear_bounds.in_set(PhysicsStepSystems::First),
            // Both write the prepared constraints; the fields they write are disjoint.
            apply_bounds
                .after(super::compliance::apply_laws)
                .after(SolverSystems::PrepareContactConstraints)
                .before(SolverSystems::PreSubstep),
            read_bounds
                .after(SolverSystems::Restitution)
                .before(SolverSystems::Finalize),
        ));
}

fn clear_bounds(mut bounds: ResMut<Bounds>) {
    bounds.0.clear();
}

/// Hands each prepared constraint point of a bounded manifold its bound over a substep (see the
/// module doc). Colors own disjoint constraints and are handed in parallel.
pub(crate) fn apply_bounds(
    bounds: Res<Bounds>,
    mut constraints: ResMut<ConstraintGraph>,
    substeps: Res<SubstepCount>,
) {
    if bounds.0.is_empty() {
        return;
    }
    let share = (substeps.0 as f32).recip();
    constraints
        .colors
        .par_splat_map_mut(ComputeTaskPool::get(), None, |_, colors| {
            for constraint in colors
                .iter_mut()
                .flat_map(|color| &mut color.contact_constraints)
            {
                // A manifold without bounds stays rigid.
                let Some(points) =
                    (bounds.0).get(&(constraint.contact_id.0, constraint.manifold_index))
                else {
                    continue;
                };
                assert_eq!(
                    constraint.points.len(),
                    points.impulse.len(),
                    "a writer bounds a manifold point for point"
                );
                for (point, impulse) in constraint.points.iter_mut().zip(&points.impulse) {
                    let most = impulse * share;
                    point.normal_part.max_impulse = most;
                    // The warm start is last step's impulse, which this step's bound never saw.
                    point.normal_part.impulse = point.normal_part.impulse.min(most);
                }
            }
        });
}

/// After the step's last solve: which bounded points ended it at their bound. The clamp stores
/// the bound itself, so equality is exact.
fn read_bounds(mut bounds: ResMut<Bounds>, constraints: Res<ConstraintGraph>) {
    if bounds.0.is_empty() {
        return;
    }
    let at_bound = (constraints.colors).par_splat_map(ComputeTaskPool::get(), None, |_, colors| {
        let mut found = Vec::new();
        for constraint in colors.iter().flat_map(|color| &color.contact_constraints) {
            let key = (constraint.contact_id.0, constraint.manifold_index);
            if bounds.0.contains_key(&key) {
                let held = (constraint.points.iter())
                    .map(|point| point.normal_part.impulse >= point.normal_part.max_impulse)
                    .collect::<Vec<_>>();
                found.push((key, held));
            }
        }
        found
    });
    for (key, held) in at_bound.into_iter().flatten() {
        bounds
            .0
            .get_mut(&key)
            .expect("the key was read from this store")
            .held = held;
    }
}
