//! AUTHORITY: the game's building model meeting bodies (game state only). A standing cell returns
//! a body no more than its bonds can ([`fracture::bound`]); pushed across its give it is freed at
//! rest and is ordinary rubble from the next step on. Nothing is predicted: the cells are the
//! ones the narrow phase found pressed.
//!
//! * [`bound_contacts`], inside the physics step after the narrow phase: groups each body's
//!   contact points on one construction into blocks of connected pressed cells, prices each
//!   ([`Standing::give`]) and bounds what its points return between them, wherever the body
//!   presses it, at that price (`avian_ext::reaction_bound`). A point the law does not move (an
//!   unbreakable element, a buried cell, a face slid along) stays rigid.
//! * [`release`], after the step: a block that returned all its bound was not enough to stop
//!   the body, which crossed [`crossed`] of its give; the share is remembered per cell while the
//!   block keeps yielding ([`Give`]) and the cells leave, at rest, when it reaches one.
//! * [`bound_belts`], before the step: the force a belt takes from standing matter is bounded by
//!   the same price; over it, the belt keeps what the cells can return and they leave.
//!
//! The support check hears none of this: a freed owner is queued for it as changed, and it reads
//! weights alone.

use avian3d::collision::contact_types::{ContactGraph, ContactId};
use avian3d::dynamics::integrator::VelocityIntegrationData;
use avian3d::dynamics::rigid_body::forces::AccumulatedLocalAcceleration;
use avian3d::dynamics::solver::schedule::SolverSystems;
use avian3d::prelude::{
    AngularVelocity, ComputedCenterOfMass, ComputedMass, ConstantForce, ConstantLinearAcceleration,
    ConstantLocalForce, ConstantLocalLinearAcceleration, Forces, Gravity, GravityScale,
    LinearVelocity, PhysicsSchedule, PhysicsStepSystems,
};

use super::fracture::bound::{Push, Standing, Way, Yield, crossed, reaction};
use super::*;
use crate::avian_ext::reaction_bound::{Bound, Bounds, Member};
use bevy::tasks::ParallelSlice;

/// One body's connected pressed cells on one construction, as the step priced them.
#[derive(Debug, Clone)]
pub(crate) struct Block {
    pub(crate) root: Entity,
    pub(crate) body: Entity,
    /// Ordered.
    pub(crate) cells: Vec<usize>,
    /// The share of its give crossed before this step.
    pub(crate) spent: f64,
    /// Each push's cheapest way to give, and its group of bounded points (`Bounds::group`).
    gives: Vec<(Yield, usize)>,
    /// Where the body presses, from its centre of mass at the step's start, m, world.
    arm: Vec3,
    /// The world's rotation into the owner's frame at the step's start.
    to_owner: Quat,
}

/// The blocks the last physics step bounded; [`release`] takes them.
#[derive(Resource, Default, Debug)]
pub(crate) struct Pressed {
    blocks: Vec<Block>,
    /// Cells a belt over-pushed before the step, by owner.
    belts: Vec<(Entity, Vec<usize>)>,
}

/// The share of its give each yielding cell has crossed, by owner and cell. A cell that stopped
/// its body, or is no longer pressed, is forgotten: its bonds held.
#[derive(Resource, Default, Debug)]
pub(crate) struct Give(BTreeMap<(Entity, usize), f64>);

/// One block's step, for the operator's instruments.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct BlockRow {
    pub(crate) body: Entity,
    pub(crate) construction: u16,
    pub(crate) cells: usize,
    pub(crate) way: Way,
    /// N and J ([`fracture::bound::Price`]).
    pub(crate) price: [f64; 2],
    /// The body's speed along the push as the step left it, m/s.
    pub(crate) closing: f64,
    /// The most the block returned along the push, N·s.
    pub(crate) bound: f64,
    /// It returned all its bound: the block did not stop the body.
    pub(crate) yielded: bool,
    /// The share of its give crossed after the step; one or more frees it.
    pub(crate) spent: f64,
    /// What its points returned along the push over the step, N·s.
    pub(crate) returned: f32,
}

/// The last step's account, for headless witnesses and operator instruments.
#[derive(Resource, Default, Debug)]
pub(crate) struct BoundAccount {
    pub(crate) blocks: Vec<BlockRow>,
    /// Freed cells: construction and chunk.
    pub(crate) freed: Vec<(u16, u16)>,
    /// Freed fragments shed to the players as decoration.
    pub(crate) shed: usize,
    /// Belt forces capped this tick: body, and the force taken back from it, N.
    pub(crate) capped_belts: Vec<(Entity, Vec3)>,
}

type Owners<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static Position,
        &'static Rotation,
        &'static mut Breakage,
    ),
    Without<Piece>,
>;

type Owned<'w, 's> = Query<
    'w,
    's,
    (
        &'static ColliderOf,
        Option<&'static Element>,
        Option<&'static ChunkCollider>,
    ),
>;

type Movers<'w, 's> = Query<
    'w,
    's,
    (
        &'static RigidBody,
        &'static Position,
        &'static Rotation,
        &'static LinearVelocity,
        &'static AngularVelocity,
        &'static ComputedCenterOfMass,
        Option<&'static VelocityIntegrationData>,
        Option<&'static AccumulatedLocalAcceleration>,
        Option<&'static GravityScale>,
        (
            &'static ComputedMass,
            Option<&'static ConstantForce>,
            Option<&'static ConstantLocalForce>,
            Option<&'static ConstantLinearAcceleration>,
            Option<&'static ConstantLocalLinearAcceleration>,
        ),
    ),
>;

/// The standing cell of `owner` a collider of it shows at world `point`: a chunk's own, or the
/// nearest of an unbroken element's. `None` for matter the law does not move: an unbreakable
/// element, a buried cell.
fn cell(
    owner: &Breakage,
    pose: &Transform,
    element: Option<&Element>,
    chunk: Option<&ChunkCollider>,
    point: Vec3,
) -> Option<usize> {
    let c = match (chunk, element) {
        (Some(chunk), _) => usize::from(chunk.0),
        (None, Some(element)) => nearest_chunk(
            &owner.kind.layout,
            usize::from(element.0),
            pose.rotation.inverse() * (point - pose.translation),
        )?,
        (None, None) => panic!("a standing owner's colliders are elements and chunks"),
    };
    (owner.live.state.standing(c) && !owner.live.structure.chunks[c].buried).then_some(c)
}

/// One contact point on a standing cell.
struct Touch {
    point: (u32, usize, usize),
    cell: usize,
    /// World.
    at: Vec3,
    /// Unit, from the body into the cell, in the owner's frame.
    into: DVec3,
}

/// The ways a body moving at `velocity` under `acceleration` (both in the owner's frame) pushes
/// the cells it meets at `touches`: along its travel, along the force that persists on it, and
/// along each face it presses. Each push's lean is the mean over the points that lean its way.
fn pushes(velocity: DVec3, acceleration: DVec3, touches: &[&Touch]) -> Vec<Push> {
    let mut ways: Vec<DVec3> = [velocity, acceleration]
        .into_iter()
        .filter_map(DVec3::try_normalize)
        .collect();
    for touch in touches {
        if !ways.contains(&touch.into) {
            ways.push(touch.into);
        }
    }
    ways.into_iter()
        .map(|along| {
            let leans = (touches.iter())
                .map(|touch| touch.into.dot(along))
                .filter(|&lean| lean > 0.0);
            let (sum, count) =
                leans.fold((0.0, 0.0), |(sum, count), lean| (sum + lean, count + 1.0));
            Push {
                along,
                closing: velocity.dot(along),
                lean: if count > 0.0 { sum / count } else { 0.0 },
            }
        })
        .collect()
}

/// One body's blocks on one owner, each with its gives' groups of points and their bounds in
/// the order of its gives.
type Priced = Vec<(Block, Vec<(Vec<Member>, Bound)>)>;

/// One owner as [`bound_contacts`] prices the bodies on it.
#[derive(Clone, Copy)]
struct Pricing<'a, 'w, 's> {
    root: Entity,
    standing: Standing<'a>,
    /// The world's rotation into the owner's frame.
    to_owner: Quat,
    movers: &'a Movers<'w, 's>,
    gravity: Vec3,
    give: &'a Give,
    dt: f64,
}

impl Pricing<'_, '_, '_> {
    /// Prices the cells `body` presses at `touches`. A block no way is open for gets no bound:
    /// its cells hold whatever presses them.
    fn price(&self, body: Entity, touches: &[Touch]) -> Priced {
        let Self {
            root,
            standing,
            to_owner,
            dt,
            ..
        } = *self;
        let (_, at, facing, linear, angular, centre, increment, local, scale, constant) =
            (self.movers.get(body)).expect("the body was read from this query");
        let (mass, force, local_force, steady, local_steady) = constant;
        // What Avian integrates before its contact solve: the forces applied to the body, its
        // constant ones and gravity (`pre_process_velocity_increments`).
        let acceleration = increment.map_or(Vec3::ZERO, |data| data.linear_increment)
            + force.map_or(Vec3::ZERO, |force| force.0 * mass.inverse())
            + steady.map_or(Vec3::ZERO, |steady| steady.0)
            + facing.0
                * (local.map_or(Vec3::ZERO, |local| local.linear)
                    + local_force.map_or(Vec3::ZERO, |force| force.0 * mass.inverse())
                    + local_steady.map_or(Vec3::ZERO, |steady| steady.0))
            + self.gravity * scale.map_or(1.0, |scale| scale.0);
        let mut cells: Vec<usize> = touches.iter().map(|touch| touch.cell).collect();
        cells.sort_unstable();
        cells.dedup();
        let mut blocks = Vec::new();
        for cells in standing.connected(&cells) {
            let on: Vec<&Touch> = (touches.iter())
                .filter(|touch| cells.binary_search(&touch.cell).is_ok())
                .collect();
            let met = on.iter().map(|touch| touch.at).sum::<Vec3>() / on.len() as f32;
            let arm = met - (at.0 + facing.0 * centre.0);
            let velocity = linear.0 + angular.0.cross(arm);
            let spent = (cells.iter())
                .map(|&c| self.give.0.get(&(root, c)).copied().unwrap_or(0.0))
                .sum::<f64>()
                / cells.len() as f64;
            let pushes = pushes(
                (to_owner * velocity).as_dvec3(),
                (to_owner * acceleration).as_dvec3(),
                &on,
            );
            // Every push bounds the points that lean its way, between them, at what the cells
            // hold and at the work they take before they let go (`Bounds::group`); a point in
            // several groups takes the least, which is the cheapest way at whatever speed the
            // solve finds the body closing.
            let gives: Vec<Yield> = (pushes.iter())
                .filter(|push| push.lean > 0.0)
                .filter_map(|&push| standing.give_along(&cells, push, spent, dt))
                .collect();
            if gives.is_empty() {
                continue;
            }
            let groups = (gives.iter())
                .map(|given| {
                    let members = (on.iter())
                        .filter_map(|touch| {
                            let lean = touch.into.dot(given.push.along) as f32;
                            let (contact, manifold, point) = touch.point;
                            (lean > 0.0).then_some(Member {
                                contact: ContactId(contact),
                                manifold,
                                point,
                                lean,
                            })
                        })
                        .collect();
                    let most = Bound {
                        impulse: (given.price.force * dt) as f32,
                        work: ((1.0 - spent) * given.price.work) as f32,
                    };
                    (members, most)
                })
                .collect();
            blocks.push((
                Block {
                    root,
                    body,
                    cells,
                    spent,
                    arm,
                    to_owner,
                    gives: gives.into_iter().map(|given| (given, usize::MAX)).collect(),
                },
                groups,
            ));
        }
        blocks
    }
}

/// `owner`'s structure as the law reads it. The gravity field it fills is a cache of the
/// state, so the caller passes the owner past change detection.
fn standing(owner: &mut Breakage) -> Standing<'_> {
    let live = &mut *owner.live;
    Standing {
        structure: &live.structure,
        state: &live.state,
        shapes: &owner.kind.layout.blocks,
        preload: live
            .gravity
            .get(&live.structure, &live.state, &live.assessment),
    }
}

#[derive(bevy::ecs::system::SystemParam)]
struct BoundContacts<'w, 's> {
    contacts: Res<'w, ContactGraph>,
    owned: Owned<'w, 's>,
    owners: Owners<'w, 's>,
    movers: Movers<'w, 's>,
    gravity: Res<'w, Gravity>,
    time: Res<'w, Time>,
    give: Res<'w, Give>,
    bounds: ResMut<'w, Bounds>,
    pressed: ResMut<'w, Pressed>,
    stages: Option<Res<'w, Stages>>,
}

/// Inside the physics step, between the narrow phase and the solver's constraints (module doc).
/// Each body's cells are priced from the owner's state and its own motion alone, so the bodies
/// on one owner are priced in parallel; their bounds are handed over in owner and body order.
fn bound_contacts(mut p: BoundContacts) {
    let _bound = Stages::start(p.stages.as_deref(), Stage::Bound);
    p.pressed.blocks.clear();
    let dt = f64::from(p.time.delta_secs());
    let mut touches = BTreeMap::<Entity, BTreeMap<Entity, Vec<Touch>>>::new();
    for pair in p.contacts.iter_active() {
        let sides = [
            (pair.collider1, pair.body2, -1.0),
            (pair.collider2, pair.body1, 1.0),
        ];
        for (collider, body, sign) in sides {
            let Some(body) = body else {
                continue; // A collider with no body presses nothing.
            };
            let Ok((of, element, chunk)) = p.owned.get(collider) else {
                continue; // Not construction matter.
            };
            let Ok((root, position, rotation, owner)) = p.owners.get(of.body) else {
                continue; // A piece or any other body: ordinary contact.
            };
            if !p.movers.get(body).is_ok_and(|mover| mover.0.is_dynamic()) {
                continue; // Only a dynamic body pushes.
            }
            let pose = owner.physics_frame((position, rotation));
            for (m, manifold) in pair.manifolds.iter().enumerate() {
                let into = (pose.rotation.inverse() * (sign * manifold.normal)).as_dvec3();
                for (n, point) in manifold.points.iter().enumerate() {
                    // Matter the law does not move answers rigidly.
                    if let Some(cell) = cell(owner, &pose, element, chunk, point.point) {
                        (touches.entry(root).or_default().entry(body).or_default()).push(Touch {
                            point: (pair.contact_id.0, m, n),
                            cell,
                            at: point.point,
                            into,
                        });
                    }
                }
            }
        }
    }
    for (root, bodies) in touches {
        let (_, position, rotation, mut owner) =
            (p.owners.get_mut(root)).expect("the owner was read from this query");
        let owner = owner.bypass_change_detection();
        let to_owner = owner.physics_frame((position, rotation)).rotation.inverse();
        let pricing = Pricing {
            root,
            standing: standing(owner),
            to_owner,
            movers: &p.movers,
            gravity: p.gravity.0,
            give: &p.give,
            dt,
        };
        let bodies: Vec<(Entity, Vec<Touch>)> = bodies.into_iter().collect();
        let priced = bodies.par_splat_map(ComputeTaskPool::get(), None, |_, bodies| {
            (bodies.iter())
                .map(|(body, touches)| pricing.price(*body, touches))
                .collect::<Vec<Priced>>()
        });
        for (mut block, groups) in priced.into_iter().flatten().flatten() {
            for ((_, group), (members, most)) in block.gives.iter_mut().zip(groups) {
                *group = p.bounds.group(members, most);
            }
            p.pressed.blocks.push(block);
        }
    }
}

/// `plug` (ordered) parted from `breakage`'s standing matter: the joints on its boundary
/// separate, and it and whatever it alone held up are fallen. Returns the fragments they leave
/// as. Internal joints stay, so a plug leaves as the pieces its own bonds make.
pub(super) fn cut(
    breakage: &mut Breakage,
    plug: &[usize],
    tick: u32,
    record: &mut Record,
) -> Vec<Vec<usize>> {
    let kind = &breakage.kind;
    let live = &mut *breakage.live;
    let inside = |c: usize| plug.binary_search(&c).is_ok();
    // A boundary joint has one side inside, so a plug's ordered cells meet it once.
    let boundary: Vec<usize> = (plug.iter())
        .flat_map(|&c| live.state.incident(c))
        .copied()
        .filter(|&j| {
            let joint = &live.structure.joints[j];
            let b = matches!(joint.b, Side::Chunk(b) if inside(b));
            inside(joint.a) != b && !live.state.separated[j]
        })
        .collect();
    for &j in &boundary {
        live.state.cracked[j] = true;
        live.state.separated[j] = true;
        let joint = &live.structure.joints[j];
        live.swapped[usize::from(kind.layout.element_of[joint.a])] = true;
        if let Side::Chunk(b) = joint.b {
            live.swapped[usize::from(kind.layout.element_of[b])] = true;
        }
        record.entries.push(Entry {
            construction: breakage.construction,
            event: Event::Separate(j as u16),
            tick,
        });
    }
    let mut fragments = crate::debris::crumble::fragments(&live.structure, &live.state, plug);
    for &c in plug {
        live.state.fate[c] = Fate::Fallen;
    }
    fragments.extend(fracture::loose_groups(&live.structure, &live.state));
    let fragments = crate::debris::crumble::merge_fragments(
        &live.structure,
        fragments,
        &boundary,
        &mut live.rubble_joints,
    );
    for &c in fragments.iter().flatten() {
        live.state.fate[c] = Fate::Fallen;
        live.swapped[usize::from(kind.layout.element_of[c])] = true;
    }
    fragments
}

/// `fragments` of `breakage` ([`cut`]) leaving at rest from `pose`: each routed by size
/// ([`crate::debris::crumble::route`]) to a prepared body or a decoration.
pub(super) fn depart(
    breakage: &mut Breakage,
    pose: Transform,
    fragments: Vec<Vec<usize>>,
    tick: u32,
    record: &mut Record,
    reusable: &ReusableColliders,
) -> (Vec<PreparedDeparture>, Vec<crate::debris::crumble::Shed>) {
    let rest = ReleasedMotion::at_rest(pose.translation);
    let (kept, sheds) = crate::debris::crumble::route(
        &breakage.live.structure,
        (breakage.construction, pose),
        fragments,
        |light| vec![rest; light.len()],
        tick,
        record,
    );
    // Root state changes only after every worker has finished reading it.
    let departures = ComputeTaskPool::get().scope(|scope| {
        let breakage = &*breakage;
        let kind = &breakage.kind;
        for group in kept {
            scope.spawn(async move {
                let launch = launch(kind, &breakage.live.structure, &group, &[]);
                prepare_group(
                    Departure {
                        kind: kind.clone(),
                        construction: breakage.construction,
                        root: pose,
                        live: breakage.live.group(&group),
                        group,
                        launch,
                        tick,
                    },
                    reusable,
                )
            });
        }
    });
    crate::debris::crumble::mark_dust(&mut breakage.live.state, &sheds);
    (departures, sheds)
}

#[derive(bevy::ecs::system::SystemParam)]
struct Release<'w, 's> {
    pressed: ResMut<'w, Pressed>,
    bounds: Res<'w, Bounds>,
    give: ResMut<'w, Give>,
    account: ResMut<'w, BoundAccount>,
    owners: Owners<'w, 's>,
    movers: Query<'w, 's, (&'static LinearVelocity, &'static AngularVelocity)>,
    children: Query<'w, 's, &'static Children>,
    chunk_entities: ChunkChildren<'w, 's>,
    fixed: Res<'w, Time<Fixed>>,
    clock: Option<Res<'w, ShotClock>>,
    record: ResMut<'w, Record>,
    queue: ResMut<'w, Queue>,
    shed: MessageWriter<'w, crate::debris::crumble::Shed>,
    commands: Commands<'w, 's>,
    stages: Option<Res<'w, Stages>>,
}

/// After the step: remember what yielded and free what crossed its give (module doc).
fn release(mut p: Release) {
    let _release = Stages::start(p.stages.as_deref(), Stage::Release);
    let dt = f64::from(p.fixed.timestep().as_secs_f32());
    let tick = p.clock.as_deref().map_or(0, |clock| clock.0.raw());
    let Pressed { blocks, belts } = std::mem::take(&mut *p.pressed);
    p.account.blocks.clear();
    p.account.freed.clear();
    p.account.shed = 0;
    // Only what yields this step is remembered ([`Give`]).
    p.give.0.clear();
    let mut leaving: BTreeMap<Entity, BTreeSet<usize>> = BTreeMap::new();
    for (root, cells) in belts {
        leaving.entry(root).or_default().extend(cells);
    }
    for block in blocks {
        // What the body crossed is how far the step moved it into the cells: its speed as the
        // solve left it, which a third body pushing it inside the step made. The way that gave
        // is the one that returns least at that speed.
        let velocity = (p.movers.get(block.body)).map_or(DVec3::ZERO, |(linear, angular)| {
            (block.to_owner * (linear.0 + angular.0.cross(block.arm))).as_dvec3()
        });
        let returned = |(given, _): &(Yield, usize)| {
            reaction(given.price, block.spent, velocity.dot(given.push.along), dt) / given.push.lean
        };
        let (given, group) = (block.gives.into_iter())
            .min_by(|a, b| returned(a).total_cmp(&returned(b)))
            .expect("a block holds a way to give");
        let closing = velocity.dot(given.push.along);
        // The points of the way that gave returned all its bound between them.
        let yielded = p.bounds.spent(group);
        let spent = if yielded {
            block.spent + crossed(given.price, closing, dt)
        } else {
            0.0
        };
        if let Ok((_, _, _, owner)) = p.owners.get(block.root) {
            p.account.blocks.push(BlockRow {
                body: block.body,
                construction: owner.construction,
                cells: block.cells.len(),
                way: given.way,
                price: [given.price.force, given.price.work],
                closing,
                bound: reaction(given.price, block.spent, closing, dt),
                yielded,
                spent,
                returned: p.bounds.returned(group),
            });
        }
        if spent >= 1.0 {
            leaving.entry(block.root).or_default().extend(given.leaves);
        } else if yielded {
            for c in block.cells {
                let share = p.give.0.entry((block.root, c)).or_insert(0.0_f64);
                *share = share.max(spent);
            }
        }
    }
    for (root, cells) in leaving {
        let Ok((_, position, rotation, mut owner)) = p.owners.get_mut(root) else {
            continue; // The owner left the world inside the step; its cells went with it.
        };
        let plug: Vec<usize> = (cells.into_iter())
            .filter(|&c| owner.live.state.standing(c))
            .collect();
        if plug.is_empty() {
            continue; // Another law freed them first.
        }
        let pose = owner.physics_frame((position, rotation));
        let construction = owner.construction;
        let fragments = cut(&mut owner, &plug, tick, &mut p.record);
        // A construction without children has no chunk collider to rehome.
        let reusable = reusable_colliders(p.children.get(root).ok(), &p.chunk_entities);
        let (departures, sheds) =
            depart(&mut owner, pose, fragments, tick, &mut p.record, &reusable);
        (p.account.freed).extend(plug.iter().map(|&c| (construction, c as u16)));
        p.account.shed += sheds.len();
        p.shed.write_batch(sheds);
        p.queue.0.push(RawDeposit {
            changed: vec![root],
            ..default()
        });
        for departure in departures {
            spawn_prepared(&mut p.commands, departure);
        }
    }
}

#[derive(bevy::ecs::system::SystemParam)]
struct BoundBelts<'w, 's> {
    transfers: Res<'w, ContactTransfers>,
    owned: Owned<'w, 's>,
    owners: Owners<'w, 's>,
    bodies: Query<'w, 's, (Forces, &'static ComputedCenterOfMass)>,
    fixed: Res<'w, Time<Fixed>>,
    pressed: ResMut<'w, Pressed>,
    account: ResMut<'w, BoundAccount>,
    stages: Option<Res<'w, Stages>>,
}

/// Before the step: each belt's force on standing matter against what that matter can return
/// (module doc). `StagedForce::force` is the force on the belt's body.
fn bound_belts(mut p: BoundBelts) {
    let _bound = Stages::start(p.stages.as_deref(), Stage::Bound);
    p.account.capped_belts.clear();
    p.pressed.belts.clear();
    let dt = f64::from(p.fixed.timestep().as_secs_f32());
    let mut staged = BTreeMap::<(Entity, Entity), Vec<(usize, Vec3, Vec3)>>::new();
    for force in p.transfers.staged() {
        let (_, element, chunk) = (p.owned.get(force.structure_collider))
            .expect("a staged belt force names a contact collider of its owner");
        let Ok((_, position, rotation, owner)) = p.owners.get(force.structure) else {
            continue; // A belt on a moving group pushes it through its body.
        };
        let pose = owner.physics_frame((position, rotation));
        if let Some(cell) = cell(owner, &pose, element, chunk, force.point) {
            staged
                .entry((force.structure, force.body))
                .or_default()
                .push((cell, force.point, force.force));
        }
    }
    for ((root, body), forces) in staged {
        let (_, position, rotation, mut owner) =
            (p.owners.get_mut(root)).expect("the owner was read from this query");
        let owner = owner.bypass_change_detection();
        let pose = owner.physics_frame((position, rotation));
        let (mut source, centre) =
            (p.bodies.get_mut(body)).expect("a belt's body is a dynamic body");
        let to_owner = pose.rotation.inverse();
        let mut cells: Vec<usize> = forces.iter().map(|&(cell, ..)| cell).collect();
        cells.sort_unstable();
        cells.dedup();
        let standing = standing(owner);
        for cells in standing.connected(&cells) {
            let on: Vec<_> = (forces.iter())
                .filter(|(cell, ..)| cells.binary_search(cell).is_ok())
                .collect();
            let total: Vec3 = on.iter().map(|&&(_, _, force)| force).sum();
            let Some(along) = (to_owner * -total).as_dvec3().try_normalize() else {
                continue; // The belt bears nothing on these cells.
            };
            let met = on.iter().map(|&&(_, at, _)| at).sum::<Vec3>() / on.len() as f32;
            let arm = met - (source.position() + source.rotation() * centre.0);
            let velocity = source.linear_velocity() + source.angular_velocity().cross(arm);
            let push = Push {
                along,
                closing: (to_owner * velocity).as_dvec3().dot(along),
                lean: 1.0,
            };
            let Some(given) = standing.give(&cells, &[push], 0.0, dt) else {
                continue; // No way is open: the cells bear the belt whatever it presses with.
            };
            let most = given.impulse / dt;
            let pressing = f64::from(total.length());
            if pressing <= most {
                continue; // The cells bear the belt.
            }
            // The belt keeps what the cells return; the rest it never had.
            let back = (1.0 - most / pressing) as f32;
            for &&(_, at, force) in &on {
                source.apply_force_at_point(-force * back, at);
            }
            p.account.capped_belts.push((body, -total * back));
            p.pressed.belts.push((root, given.leaves));
        }
    }
}

pub(super) fn plugin(app: &mut App) {
    app.init_resource::<Pressed>()
        .init_resource::<Give>()
        .init_resource::<BoundAccount>()
        .add_systems(
            FixedPostUpdate,
            (
                bound_belts
                    .in_set(BeltSet)
                    .before(avian3d::prelude::PhysicsSystems::Prepare),
                release.in_set(LawSet).before(apply_deposits),
            )
                .run_if(not(resource_exists::<ClientReplica>)),
        )
        .add_systems(
            PhysicsSchedule,
            bound_contacts
                .in_set(PhysicsStepSystems::Solver)
                .after(SolverSystems::PrepareSolverBodies)
                .before(SolverSystems::PrepareContactConstraints)
                .run_if(not(resource_exists::<ClientReplica>)),
        );
}

#[cfg(test)]
mod tests;
