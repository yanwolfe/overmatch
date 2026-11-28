# A collider's AABB lags its pose after its body leaves the solver mid-step, or after it is enabled again

Found in a game using avian3d. Local draft; not filed.

**Crate:** avian3d 0.8.0 (crates.io). **Status:** not sent. Kind: bug fix. Source audit: not run
against upstream `main`; the diff is against the published crate (MEASURED: `patch --dry-run`
applies it to a fresh extraction of the cached archive). The library compile check against the
published crate was not run; the same change compiles in the game's fork.

## Defect

Two cases leave `ColliderAabb`, `EnlargedAabb` and the tree proxy at an old pose, both in
`src/collider_tree/update.rs`.

1. `update_solver_body_aabbs` runs after `PhysicsStepSystems::Finalize` over bodies
   `With<SolverBodyIndex>`. A body whose island goes to sleep in `PhysicsStepSystems::Sleeping`,
   or that a system there disables or makes static, has lost its index by then
   (`remove_solver_body`, `on_insert_rigid_body`) and is skipped. The next step's
   `update_moved_collider_aabbs` skips it too: the solver's write to `Position` is older than
   `LastPhysicsTick`. The AABB stays one step of motion behind until something else moves the
   body: below the sleep thresholds for a sleeping island, the whole step's travel otherwise.
2. No AABB pass visits a collider that carries `ColliderDisabled` or `Disabled`, and removing
   the marker moves no pose or shape tick. `add_to_tree_on` re-adds the proxy with the stored
   `EnlargedAabb`, and `update_moved_collider_aabbs` skips the collider. It keeps the AABB it was
   disabled with, wherever its pose went meanwhile, until the pose next changes.

The broad phase and spatial queries read AABBs that do not hold the shape.

## Minimal repro

1. A dynamic sphere of radius 0.5 in zero gravity, drifting at half the default linear sleep
   threshold; set its `SleepTimer` to `TimeToSleep` and step once. It sleeps in that step, the
   step moved it, and its `ColliderAabb` differs from
   `collider.aabb(position, rotation, contact_tolerance)` by the step's travel (MEASURED:
   1.17 mm at 64 Hz). Made static from a system in `PhysicsStepSystems::Sleeping` while moving a
   step clear of its enlarged AABB, a ray cast finds it at the old pose and not at the new one.
2. A dynamic sensor sphere; insert `ColliderDisabled`, write its `Position` onto a static sphere
   ten radii away, step twice, remove `ColliderDisabled`, step once: no contact pair. The same
   with `Disabled`.

## Fix

Against the pristine crate (`a/` and `b/` are the crate root):

```diff
--- a/src/collider_tree/update.rs
+++ b/src/collider_tree/update.rs
@@ -57,12 +57,17 @@
         );
 
         // Clear moved proxies and update dynamic and kinematic collider AABBs.
+        // Last in the solver's set, while every body it moved still has its `SolverBodyIndex`:
+        // after `PhysicsStepSystems::Sleeping`, a body whose island slept in this step has lost
+        // its index, and its AABB would stay one step behind its pose. A pose written later in
+        // this schedule by anything but the solver is not seen until it changes again, as for a
+        // static body.
         app.add_systems(
             PhysicsSchedule,
             (clear_moved_proxies, update_solver_body_aabbs::<C>)
                 .chain()
-                .after(PhysicsStepSystems::Finalize)
-                .before(PhysicsStepSystems::Last),
+                .in_set(PhysicsStepSystems::Solver)
+                .after(SolverSystems::StoreContactImpulses),
         );
 
         // Initialize `ColliderAabb` for colliders.
@@ -871,7 +876,7 @@
                 &mut ColliderAabbMargin,
                 Ref<C>,
                 Option<&CollisionMargin>,
-                &ColliderTreeProxyKey,
+                Ref<ColliderTreeProxyKey>,
             ),
             Without<ColliderDisabled>,
         >,
@@ -927,9 +932,14 @@
         )| {
             // Skip if the collider's AABB can't have changed since the last physics tick.
             let collider_changed = collider.last_changed().is_newer_than(last_tick.0, this_run);
+            // A collider that joined a tree since the last tick is examined too. No AABB pass
+            // visits a collider while it is disabled, and enabling it again moves no pose or
+            // shape tick, so it would keep the AABB it was disabled with wherever it stands
+            // now; joining a tree writes its key.
             if !pos.last_changed().is_newer_than(last_tick.0, this_run)
                 && !rot.last_changed().is_newer_than(last_tick.0, this_run)
                 && !collider_changed
+                && !proxy_key.last_changed().is_newer_than(last_tick.0, this_run)
             {
                 return;
             }
```

## Notes

Running the pass last in `PhysicsStepSystems::Solver` puts it after `writeback_solver_bodies`
and `ColliderTreeSystems::EndOptimize` (both in `SolverSystems::Finalize`) and before anything
that removes a solver body in the step. Being in the set also orders it against every system
placed after the set; placed merely after the set, it is ambiguous with any such system that
writes a velocity or a pose, and a schedule that errors on ambiguity refuses to build.
`clear_moved_proxies` moves with the pass, so a proxy that joins a tree later in the step stays
marked moved for the next broad phase.

The window that remains, with or without this patch, comes from `update_moved_collider_aabbs`
accepting only ticks newer than `LastPhysicsTick`, which is set after `PhysicsStepSystems::Last`:
inside the schedule from `ColliderTreeSystems::UpdateAabbs` to the end of the step, a pose, shape
or tree-key write reaches the AABB only as the pose of an awake solver body written before
`update_solver_body_aabbs`. For a collider that pass skips (static, sleeping, on a disabled body,
standalone) the window is that whole span, and the joined-a-tree check below does not cover a
collider enabled inside it: each waits until its pose next changes.

What the move gives up against upstream's placement, both inside that span: a pose that a
collision-event handler or another system in `PhysicsStepSystems::Sleeping` or `Finalize` writes
to an awake body, which the pass at the end of the step caught; and a pose written mid-step to a
sleeping body, whose wake restored its index before the pass after `Finalize`.

A rejected form of the first fix kept the pass where it was and had `SolverBodies` record the
bodies removed since the solver prepared, with a second pass over them. It needed the list, a
serial pass, and static-tree arms for a body that left as static: 92 draft-diff lines against
this diff's 17.

The second fix works for a collider enabled outside that window. It keeps the proxy where
`add_to_tree_on` put it, with the stored `EnlargedAabb`, until the next step's
`update_moved_collider_aabbs`: the same delay a pose written between steps has. Recomputing inside `add_to_tree_on` (the TODO beside the `Add<C>` observer) would refresh the
tree at once, at the cost of making that function generic over the collider type.
