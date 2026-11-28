# The narrow phase runs the geometry query for pairs whose AABBs are out of its reach

Found in a game using avian3d. Local draft; not filed.

**Crate:** avian3d 0.8.0 (crates.io). **Status:** not sent. Source audit: not run against upstream
`main`; the diff is against the published crate (MEASURED: `patch --dry-run` applies it to a fresh
extraction of the cached archive).

## Defect

A contact pair lives while its two `EnlargedAabb`s overlap, and `update_contacts`
(`src/collision/narrow_phase/system_param.rs`) runs `contact_manifolds_with_context` for every
such pair that is not recycled. The enlarged AABB is the collider's `ColliderAabb` plus up to
0.05 length units (`ColliderAabbMargin`), kept until the tight AABB leaves it, so many pairs have
tight AABBs that no longer meet: the query runs, allocates its manifold list and returns nothing.

## Minimal repro

Two unit spheres, one static and one dynamic, 0.1 apart with no relative motion budget left to
recycle (or `recycle_distance: 0.0`): the pair stays in the contact graph and queries every step.
In a game's collapsing buildings, 20 to 22 % of the geometry queries were of such pairs (MEASURED,
found in a game using avian3d: 2.3 to 3.5 of 11.7 to 15.7 queries per awake body per step, four
runs, recycling off).

## Fix

Against the pristine crate (`a/` and `b/` are the crate root):

```diff
--- a/src/collision/narrow_phase/system_param.rs
+++ b/src/collision/narrow_phase/system_param.rs
@@ -28,6 +28,7 @@
     of: Option<Read<ColliderOf>>,
     shape: Read<C>,
     enlarged_aabb: Read<EnlargedAabb>,
+    aabb: Read<ColliderAabb>,
     position: Read<Position>,
     rotation: Read<Rotation>,
     transform: Option<Read<ColliderTransform>>,
@@ -805,16 +806,30 @@
                 // Compute the contact manifolds.
                 let context =
                     ColliderPairContext::new(collider1.entity, collider2.entity, collider_context);
-                collider1.shape.contact_manifolds_with_context(
-                    collider2.shape,
-                    collider1.position.0,
-                    *collider1.rotation,
-                    collider2.position.0,
-                    *collider2.rotation,
-                    max_contact_distance,
-                    &mut contacts.manifolds,
-                    context,
+                // AABBs farther apart than the query reaches are its miss. Each `ColliderAabb`
+                // holds its shape grown by the contact tolerance and the collider's own margin,
+                // so with one of them grown by the rest of the reach, disjoint AABBs put the
+                // shapes more than `max_contact_distance + contact_tolerance` apart.
+                // The spare tolerance covers the rounding of the AABBs and of the query.
+                let within_reach = collider1.aabb.intersects(
+                    &collider2
+                        .aabb
+                        .grow(Vector::splat(max_contact_distance - contact_tolerance)),
                 );
+                if within_reach {
+                    collider1.shape.contact_manifolds_with_context(
+                        collider2.shape,
+                        collider1.position.0,
+                        *collider1.rotation,
+                        collider2.position.0,
+                        *collider2.rotation,
+                        max_contact_distance,
+                        &mut contacts.manifolds,
+                        context,
+                    );
+                } else {
+                    contacts.manifolds.clear();
+                }
 
                 // Transform and prune contact data.
                 contacts.manifolds.retain_mut(|manifold| {
```

## Notes

Same contacts as without the test. `update_solver_body_aabbs` and `update_moved_collider_aabbs`
grow each `ColliderAabb` by `contact_tolerance + CollisionMargin` of the collider itself, and the
query's reach is `max_contact_distance`: both margins, a body's included, plus the speculative
distance, which is at least the tolerance. Two shapes within the reach have AABBs within it on
every axis. Growing one stored AABB by `max_contact_distance - contact_tolerance` therefore leaves
one tolerance (plus the colliders' own margins) between "disjoint" and "within reach", which
covers the rounding of the AABBs and of the query; `contact_manifolds_with_context` documents an
empty result beyond its prediction distance, and the else arm leaves the list as the query's miss
does. The growth reads the reach from the variable the query is given, so a margin a body lends
its colliders and a requested speculative distance are inside it though no AABB holds them.

Two things it relies on: a `CollisionMargin` is never negative, and each `ColliderAabb` holds the
current tolerance. A `contact_tolerance` changed at run time reaches a collider's AABB when its
pose or shape next changes; the broad phase pairs colliders under the same lag.

Check, in the game: with the query still run for every pair, 0 of 4,550,540 pairs the test would
have skipped kept a contact after pruning (MEASURED, three runs of collapsing buildings).
