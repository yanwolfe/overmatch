# A body that leaves the solver in the step that moved it keeps the previous step's AABB

Found in a game using avian3d. Local draft; not filed.

**Crate:** avian3d 0.8.0 (crates.io). **Status:** not sent. Source audit: not run against upstream
`main`; the diff is against the published crate (MEASURED: `patch --dry-run` applies it to a fresh
extraction of the cached archive). The library compile check against the published crate was not
run; the same change compiles in the game's fork.

## Defect

After the solver, `update_solver_body_aabbs` (`src/collider_tree/update.rs`) refreshes the
`ColliderAabb` of every collider of a body `With<SolverBodyIndex>`. A body whose island goes to
sleep in `PhysicsStepSystems::Sleeping`, or that a system between the solver's write-back and
`PhysicsStepSystems::Finalize` disables or makes static, loses its `SolverBodyIndex` before that
system runs (`remove_solver_body`, `on_insert_rigid_body`), so it is skipped. The next step's
`update_moved_collider_aabbs` skips it too: the solver's write to `Position` is older than
`LastPhysicsTick`. Its `ColliderAabb`, `EnlargedAabb` and tree proxy stay at the pose before the
last step's motion until something else moves the body.

The lag is one step of motion: below the sleep thresholds for a sleeping island, the whole step's
travel for a body made static or disabled. The broad phase and spatial queries read AABBs that do
not hold the shape; anything that reads `ColliderAabb` as the shape's bounds is off by that much.

## Minimal repro

A dynamic sphere of radius 0.5 in zero gravity, drifting at half the default linear sleep
threshold; set its `SleepTimer` to `TimeToSleep` and step once. It sleeps in that step, the step
moved it, and its `ColliderAabb` differs from `collider.aabb(position, rotation, contact_tolerance)`
by the step's travel (MEASURED: 1.17 mm at 64 Hz). Making a body moving at 1 m/s static from a
system in `PhysicsStepSystems::Sleeping` leaves it 15.6 mm behind (MEASURED).

## Fix

`SolverBodies` records the bodies removed since the solver last prepared, and
`update_solver_body_aabbs` runs its per-body pass over those as well. A body that left as static
has moved to the static tree, so the pass handles all four trees. Against the pristine crate
(`a/` and `b/` are the crate root):

```diff
--- a/src/collider_tree/update.rs
+++ b/src/collider_tree/update.rs
@@ -9,7 +9,7 @@
     },
     collision::collider::{ColliderAabbMargin, EnlargedAabb},
     data_structures::bit_vec::BitVec,
-    dynamics::solver::solver_body::SolverBodyIndex,
+    dynamics::solver::solver_body::{SolverBodies, SolverBodyIndex},
     prelude::*,
     schedule::LastPhysicsTick,
     utils::{MIN_PAR_ITER_ENTITIES, ParallelQueryForEach},
@@ -18,7 +18,7 @@
     ecs::{
         change_detection::Tick,
         entity_disabling::Disabled,
-        query::QueryFilter,
+        query::{QueryFilter, ROQueryItem},
         system::{StaticSystemParam, SystemChangeTick},
     },
     platform::collections::HashMap,
@@ -687,18 +687,23 @@
 // TODO: Once dynamic an kinematic bodies have their own marker components,
 //       we should use those instead of `SolverBody`. Solver bodies should
 //       be an implementation detail of the solver.
+type AabbBody = (
+    &'static Position,
+    &'static ComputedCenterOfMass,
+    &'static LinearVelocity,
+    &'static AngularVelocity,
+    &'static RigidBodyColliders,
+    Option<&'static SpeculativeCcd>,
+);
+
 fn update_solver_body_aabbs<C: AnyCollider>(
-    body_query: Query<
-        (
-            &Position,
-            &ComputedCenterOfMass,
-            &LinearVelocity,
-            &AngularVelocity,
-            &RigidBodyColliders,
-            Option<&SpeculativeCcd>,
-        ),
-        With<SolverBodyIndex>,
-    >,
+    body_query: Query<AabbBody, With<SolverBodyIndex>>,
+    // A body the solver moved this step and that then left the solver (its island slept, it was
+    // disabled or made static) has no index here, and its pose change is older than the tick
+    // `update_moved_collider_aabbs` compares against: without this pass its AABB would stay one
+    // step behind its pose.
+    left_query: Query<AabbBody, (Without<SolverBodyIndex>, Allow<Disabled>)>,
+    solver_bodies: Res<SolverBodies>,
     mut colliders: ParamSet<(
         Query<
             (
@@ -734,11 +739,16 @@
     // TODO: Use a better way to track the number of proxies.
     let cap_dynamic = trees.dynamic_tree.proxies.capacity();
     let cap_kinematic = trees.kinematic_tree.proxies.capacity();
+    // A body that left the solver may now be static.
+    let cap_static = trees.static_tree.proxies.capacity();
+    let cap_standalone = trees.standalone_tree.proxies.capacity();
 
     // Clear and resize the enlarged proxy structures.
     let e = &mut enlarged_proxies;
     e.dynamic_proxies.clear_and_set_capacity(cap_dynamic);
     e.kinematic_proxies.clear_and_set_capacity(cap_kinematic);
+    e.static_proxies.clear_and_set_capacity(cap_static);
+    e.standalone_proxies.clear_and_set_capacity(cap_standalone);
 
     // A small, fixed contact tolerance is added to each AABB so the narrow phase
     // can predict slightly separated contacts.
@@ -748,9 +758,9 @@
 
     let collider_query = colliders.p0();
 
-    body_query.par_for_each(
-        MIN_PAR_ITER_ENTITIES,
-        |(rb_pos, center_of_mass, lin_vel, ang_vel, body_colliders, speculative_ccd)| {
+    let update_body = |item: ROQueryItem<'_, '_, AabbBody>| {
+        {
+            let (rb_pos, center_of_mass, lin_vel, ang_vel, body_colliders, speculative_ccd) = item;
             for collider_entity in body_colliders.iter() {
                 let Ok((
                     collider,
@@ -812,9 +822,14 @@
                     );
                 }
 
-                // Solver bodies are always dynamic or kinematic, so they use the size-relative
-                // AABB margin to allow some movement without triggering tree updates.
-                let moved = enlarged_aabb.update(&aabb, aabb_margin.0);
+                // Dynamic and kinematic colliders use the size-relative AABB margin to allow some
+                // movement without triggering tree updates. A body that left the solver as static
+                // keeps its AABB tight, as `update_moved_collider_aabbs` does.
+                let margin = match proxy_key.tree_type() {
+                    ColliderTreeType::Dynamic | ColliderTreeType::Kinematic => aabb_margin.0,
+                    ColliderTreeType::Static | ColliderTreeType::Standalone => contact_tolerance,
+                };
+                let moved = enlarged_aabb.update(&aabb, margin);
 
                 if moved {
                     let tree_type = proxy_key.tree_type();
@@ -825,7 +840,8 @@
                             let capacity = match tree_type {
                                 ColliderTreeType::Dynamic => cap_dynamic,
                                 ColliderTreeType::Kinematic => cap_kinematic,
-                                _ => unreachable!("Static or standalone proxy {proxy_key:?} moved in dynamic AABB update"),
+                                ColliderTreeType::Static => cap_static,
+                                ColliderTreeType::Standalone => cap_standalone,
                             };
                             let mut bit_vec = BitVec::new(capacity);
                             bit_vec.set_bit_count_and_clear(capacity);
@@ -836,12 +852,19 @@
                     thread_local_bit_vec.set(proxy_key.id().index());
                 }
             }
-        },
-    );
+        }
+    };
+    body_query.par_for_each(MIN_PAR_ITER_ENTITIES, &update_body);
+    // A body that left and came back is in both passes; the second repeats the first.
+    for &body in solver_bodies.left() {
+        if let Ok(item) = left_query.get(body) {
+            update_body(item);
+        }
+    }
 
-    // Update the AABBs of moved proxies in the dynamic and kinematic trees.
+    // Update the AABBs of moved proxies in each tree.
     let aabb_query = colliders.p1();
-    for &tree_type in &[ColliderTreeType::Dynamic, ColliderTreeType::Kinematic] {
+    for tree_type in ColliderTreeType::ALL {
         update_tree_for_moved_proxies(
             tree_type,
             trees.tree_for_type_mut(tree_type),
--- a/src/dynamics/solver/solver_body/mod.rs
+++ b/src/dynamics/solver/solver_body/mod.rs
@@ -574,6 +574,8 @@
     entities: Vec<Entity>,
     bodies: Vec<SolverBody>,
     inertias: Vec<SolverBodyInertia>,
+    /// The bodies removed since the solver last prepared.
+    left: Vec<Entity>,
 }
 
 impl SolverBodies {
@@ -584,9 +586,23 @@
             entities: Vec::new(),
             bodies: Vec::new(),
             inertias: Vec::new(),
+            left: Vec::new(),
         }
     }
 
+    /// Returns the bodies that left since the solver last prepared, repeats included.
+    /// One of them the solver moved this step is absent from every later per-solver-body pass.
+    #[inline]
+    pub(crate) fn left(&self) -> &[Entity] {
+        &self.left
+    }
+
+    /// Starts a step's record of the bodies that leave.
+    #[inline]
+    pub(crate) fn forget_left(&mut self) {
+        self.left.clear();
+    }
+
     /// Returns the number of solver bodies.
     #[inline]
     pub fn len(&self) -> usize {
@@ -702,7 +718,8 @@
     /// Panics if the index is out of bounds.
     #[inline]
     pub fn swap_remove(&mut self, index: SolverBodyIndex) -> Option<Entity> {
-        self.entities.swap_remove(index.0 as usize);
+        let left = self.entities.swap_remove(index.0 as usize);
+        self.left.push(left);
         self.bodies.swap_remove(index.0 as usize);
         self.inertias.swap_remove(index.0 as usize);
         // TODO: Do this differently
--- a/src/dynamics/solver/solver_body/plugin.rs
+++ b/src/dynamics/solver/solver_body/plugin.rs
@@ -248,6 +248,7 @@
         Has<CustomPositionIntegration>,
     )>,
 ) {
+    solver_bodies.forget_left();
     let access = solver_bodies.access();
 
     #[allow(unused_variables)]
```

## Notes

The per-body closure is upstream's loop body, unchanged apart from the enlargement margin and the
bit-vector capacity following the proxy's tree. A body in the list that has an index again (it
left and came back) is refreshed by both passes with the same result. A body removed outside the
window (between steps) is refreshed needlessly once or dropped by the next prepare; its AABB was
already current, since the previous step's pass ran while it still had its index.

A collider that carried `ColliderDisabled` while its pose changed has the same symptom from
another cause and is not covered here: the AABB passes skip a disabled collider, and removing the
marker moves no change tick, so the collider rejoins the tree with its old `EnlargedAabb` until
its pose next changes.
