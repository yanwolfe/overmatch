//! AUTHORITY: the game's building model meeting bodies (game state only). A standing cell returns
//! a body no more than its bonds can ([`fracture::bound`]); pushed across its give it is freed at
//! rest and is ordinary rubble from the next step on. Nothing is predicted: the cells are the
//! ones the narrow phase found pressed.
//!
//! * [`bound_contacts`], inside the physics step after the narrow phase: groups each body's
//!   contact points on one construction into blocks of connected pressed cells, prices each
//!   ([`Standing::give`]) and hands its points their share of the reaction as the solver's bound
//!   (`avian_ext::reaction_bound`). A point the law does not move (an unbreakable element, a
//!   buried cell, a face slid along) stays rigid.
//! * [`release`], after the step: a block one of whose points ended the step at its bound was not
//!   enough to stop the body, which crossed [`crossed`] of its give; the share is remembered per
//!   cell while the block keeps yielding ([`Give`]) and the cells leave when it reaches one.
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
use super::sweep_intake::{ReleasedMotion, cut_fragments, sweep_departure};
use super::*;
use crate::avian_ext::reaction_bound::{Bound, Bounds};
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
    /// Each push's cheapest way to give.
    gives: Vec<Yield>,
    /// Where the body presses, from its centre of mass at the step's start, m, world.
    arm: Vec3,
    /// The world's rotation into the owner's frame at the step's start.
    to_owner: Quat,
    /// Its bounded points: contact pair, manifold and point; the point's normal into the cell
    /// in the owner's frame; and the impulse it carried into this step.
    points: Vec<((u32, usize, usize), DVec3, f64)>,
}

/// Resolution: the share of a block's bound its points split equally, whatever each carried last
/// step, so a point that takes load this step answers within it; the rest follows last step's
/// load. A block has yielded when the points at their bound hold the rest between them: its
/// loaded points all returned what its bonds allow.
const EVEN_SHARE: f64 = 0.25;

/// Each point's share of a block's bound among the points leaning along a push (`leans` and
/// `loads` point for point): zero for a point that does not lean that way.
fn shares(leans: &[f64], loads: &[f64]) -> Vec<f64> {
    let leaning = leans.iter().filter(|&&lean| lean > 0.0).count() as f64;
    let carried: f64 = (leans.iter().zip(loads))
        .filter(|&(&lean, _)| lean > 0.0)
        .map(|(_, &load)| load)
        .sum();
    (leans.iter().zip(loads))
        .map(|(&lean, &load)| {
            if lean <= 0.0 {
                0.0
            } else if carried > 0.0 {
                EVEN_SHARE / leaning + (1.0 - EVEN_SHARE) * load / carried
            } else {
                1.0 / leaning
            }
        })
        .collect()
}

/// The blocks the last physics step bounded; [`release`] takes them.
#[derive(Resource, Default, Debug)]
pub(crate) struct Pressed {
    blocks: Vec<Block>,
    held: Vec<HeldRow>,
    /// Cells a belt over-pushed before the step, by owner.
    belts: Vec<(Entity, Vec<usize>)>,
}

/// The share of its give each yielding cell has crossed, by owner and cell. A cell that stopped
/// its body, or is no longer pressed, is forgotten: its bonds held.
#[derive(Resource, Default, Debug)]
pub(crate) struct Give(BTreeMap<(Entity, usize), f64>);

/// The ram instrument's relay arm (`net::ram::relay`): while present, a cell's crossed share is
/// never forgotten.
#[derive(Resource)]
pub(crate) struct KeepGive;

/// The ram instrument's relay arm (`net::ram::relay`): while present, a joint the push closes
/// resists a sideways shed with nothing ([`Standing::bare`]).
#[derive(Resource)]
pub(crate) struct BareShed;

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
    /// Its loaded points ended the step at their bounds: the block did not stop the body.
    pub(crate) yielded: bool,
    /// The share of its give crossed after the step; one or more frees it.
    pub(crate) spent: f64,
    /// The push, unit, in the owner's frame.
    pub(crate) along: DVec3,
    /// The push runs into the ground or buried matter.
    pub(crate) grounded: bool,
}

/// Why pressed standing matter answers rigidly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Held {
    /// A buried cell.
    Buried,
    /// An element that does not break.
    Unbreakable,
    /// No way to give is open along any push.
    NoWay,
}

/// Standing matter one body pressed that the step left rigid, for the operator's instruments.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct HeldRow {
    pub(crate) body: Entity,
    pub(crate) construction: u16,
    /// Cells for [`Held::NoWay`], contact points otherwise.
    pub(crate) cells: usize,
    pub(crate) why: Held,
}

/// The last step's account, for headless witnesses and operator instruments.
#[derive(Resource, Default, Debug)]
pub(crate) struct BoundAccount {
    pub(crate) blocks: Vec<BlockRow>,
    pub(crate) held: Vec<HeldRow>,
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
    /// The normal impulse the point carried into this step (the solver's warm start), N·s.
    load: f64,
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

/// One body's blocks on one owner, and the bounds of their points by contact and manifold.
type Priced = (Vec<Block>, Vec<((u32, usize), (usize, Bound))>, Vec<usize>);

/// Prices the cells `body` presses on `root` at `touches` ([`bound_contacts`]).
fn price(
    (root, body): (Entity, Entity),
    touches: &[Touch],
    standing: Standing,
    to_owner: Quat,
    movers: &Movers,
    gravity: Vec3,
    give: &Give,
    dt: f64,
) -> Priced {
    let (_, at, facing, linear, angular, centre, increment, local, scale, constant) =
        movers.get(body).expect("the body was read from this query");
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
        + gravity * scale.map_or(1.0, |scale| scale.0);
    let mut cells: Vec<usize> = touches.iter().map(|touch| touch.cell).collect();
    cells.sort_unstable();
    cells.dedup();
    let (mut blocks, mut limits, mut held) = (Vec::new(), Vec::new(), Vec::new());
    for cells in standing.connected(&cells) {
        let on: Vec<&Touch> = (touches.iter())
            .filter(|touch| cells.binary_search(&touch.cell).is_ok())
            .collect();
        let met = on.iter().map(|touch| touch.at).sum::<Vec3>() / on.len() as f32;
        let arm = met - (at.0 + facing.0 * centre.0);
        let velocity = linear.0 + angular.0.cross(arm);
        let spent = (cells.iter())
            .map(|&c| give.0.get(&(root, c)).copied().unwrap_or(0.0))
            .sum::<f64>()
            / cells.len() as f64;
        let pushes = pushes(
            (to_owner * velocity).as_dvec3(),
            (to_owner * acceleration).as_dvec3(),
            &on,
        );
        // Every push bounds the points that lean its way, each by its share ([`shares`]) of what
        // the cells hold and of the work they take before they let go; a point takes the least
        // of each, which is the cheapest way at whatever speed the solve finds the body closing:
        // min over ways of min(F dt, W / v) is min(least F dt, least W / v).
        let gives: Vec<Yield> = (pushes.iter())
            .filter(|push| push.lean > 0.0)
            .filter_map(|&push| standing.give_along(&cells, push, spent, dt))
            .collect();
        if gives.is_empty() {
            held.push(cells.len());
            continue; // No way is open: the cells hold whatever presses them.
        }
        let loads: Vec<f64> = on.iter().map(|touch| touch.load).collect();
        let mut most = vec![Bound::RIGID; on.len()];
        for given in &gives {
            let leans: Vec<f64> = (on.iter())
                .map(|touch| touch.into.dot(given.push.along))
                .collect();
            let shares = shares(&leans, &loads);
            let work = (1.0 - spent) * given.price.work;
            for ((most, lean), share) in most.iter_mut().zip(leans).zip(shares) {
                if lean > 0.0 {
                    let impulse = given.price.force * dt * share / lean;
                    most.impulse = most.impulse.min(impulse as f32);
                    most.work = most.work.min((work * share) as f32);
                }
            }
        }
        let mut points = Vec::new();
        for (touch, most) in on.iter().zip(most) {
            if most != Bound::RIGID {
                let (contact, manifold, point) = touch.point;
                limits.push(((contact, manifold), (point, most)));
                points.push((touch.point, touch.into, touch.load));
            }
        }
        blocks.push(Block {
            root,
            body,
            cells,
            spent,
            arm,
            to_owner,
            gives,
            points,
        });
    }
    (blocks, limits, held)
}

/// Inside the physics step, between the narrow phase and the solver's constraints (module doc).
/// Each body's cells are priced from the owner's state and its own motion alone, so the bodies
/// on one owner are priced in parallel; their bounds are handed over in owner and body order.
fn bound_contacts(
    contacts: Res<ContactGraph>,
    owned: Owned,
    mut owners: Owners,
    movers: Movers,
    gravity: Res<Gravity>,
    time: Res<Time>,
    give: Res<Give>,
    mut bounds: ResMut<Bounds>,
    mut pressed: ResMut<Pressed>,
    stages: Option<Res<Stages>>,
    bare: Option<Res<BareShed>>,
) {
    let _bound = Stages::start(stages.as_deref(), Stage::Bound);
    pressed.blocks.clear();
    pressed.held.clear();
    let mut rigid = BTreeMap::<(Entity, u16, Held), usize>::new();
    let dt = f64::from(time.delta_secs());
    let mut touches = BTreeMap::<Entity, BTreeMap<Entity, Vec<Touch>>>::new();
    let mut sizes = BTreeMap::new();
    for pair in contacts.iter_active() {
        let sides = [
            (pair.collider1, pair.body2, -1.0),
            (pair.collider2, pair.body1, 1.0),
        ];
        for (collider, body, sign) in sides {
            let Some(body) = body else {
                continue; // A collider with no body presses nothing.
            };
            let Ok((of, element, chunk)) = owned.get(collider) else {
                continue; // Not construction matter.
            };
            let Ok((root, position, rotation, owner)) = owners.get(of.body) else {
                continue; // A piece or any other body: ordinary contact.
            };
            if !movers.get(body).is_ok_and(|mover| mover.0.is_dynamic()) {
                continue; // Only a dynamic body pushes.
            }
            let pose = owner.physics_frame((position, rotation));
            for (m, manifold) in pair.manifolds.iter().enumerate() {
                sizes.insert((pair.contact_id.0, m), manifold.points.len());
                let into = (pose.rotation.inverse() * (sign * manifold.normal)).as_dvec3();
                for (p, point) in manifold.points.iter().enumerate() {
                    if let Some(cell) = cell(owner, &pose, element, chunk, point.point) {
                        (touches.entry(root).or_default().entry(body).or_default()).push(Touch {
                            point: (pair.contact_id.0, m, p),
                            cell,
                            at: point.point,
                            into,
                            load: f64::from(point.warm_start_normal_impulse),
                        });
                    } else {
                        let why = match (chunk, element) {
                            (None, Some(element))
                                if owner.kind.layout.chunks_of(usize::from(element.0)).count()
                                    == 0 =>
                            {
                                Held::Unbreakable
                            }
                            _ => Held::Buried,
                        };
                        *rigid.entry((body, owner.construction, why)).or_default() += 1;
                    }
                }
            }
        }
    }
    let mut limits = BTreeMap::<(u32, usize), Vec<(usize, Bound)>>::new();
    for (root, bodies) in touches {
        let (_, position, rotation, mut owner) = owners
            .get_mut(root)
            .expect("the owner was read from this query");
        let construction = owner.construction;
        // The gravity field is a cache of the state, not a change to it.
        let owner = owner.bypass_change_detection();
        let to_owner = owner.physics_frame((position, rotation)).rotation.inverse();
        let live = &mut *owner.live;
        let standing = Standing {
            structure: &live.structure,
            state: &live.state,
            shapes: &owner.kind.layout.blocks,
            preload: live
                .gravity
                .get(&live.structure, &live.state, &live.assessment),
            bare: bare.is_some(),
        };
        let bodies: Vec<(Entity, Vec<Touch>)> = bodies.into_iter().collect();
        let (movers, give) = (&movers, &*give);
        let priced = bodies.par_splat_map(ComputeTaskPool::get(), None, |_, bodies| {
            (bodies.iter())
                .map(|(body, touches)| {
                    let pair = (root, *body);
                    let priced = price(
                        pair, touches, standing, to_owner, movers, gravity.0, give, dt,
                    );
                    (*body, priced)
                })
                .collect::<Vec<(Entity, Priced)>>()
        });
        for (body, (blocks, points, held)) in priced.into_iter().flatten() {
            pressed.held.extend(held.into_iter().map(|cells| HeldRow {
                body,
                construction,
                cells,
                why: Held::NoWay,
            }));
            pressed.blocks.extend(blocks);
            for (manifold, point) in points {
                limits.entry(manifold).or_default().push(point);
            }
        }
    }
    pressed.held.extend(
        (rigid.into_iter()).map(|((body, construction, why), cells)| HeldRow {
            body,
            construction,
            cells,
            why,
        }),
    );
    for ((contact, manifold), points) in limits {
        let mut each = vec![Bound::RIGID; sizes[&(contact, manifold)]];
        for (point, most) in points {
            each[point] = most;
        }
        bounds.set(ContactId(contact), manifold, each);
    }
}

/// After the step: remember what yielded and free what crossed its give (module doc).
fn release(
    mut pressed: ResMut<Pressed>,
    bounds: Res<Bounds>,
    mut give: ResMut<Give>,
    mut account: ResMut<BoundAccount>,
    mut owners: Owners,
    movers: Query<(&LinearVelocity, &AngularVelocity)>,
    children: Query<&Children>,
    chunk_entities: ChunkChildren,
    fixed: Res<Time<Fixed>>,
    clock: Option<Res<ShotClock>>,
    mut record: ResMut<Record>,
    mut queue: ResMut<Queue>,
    mut shed: MessageWriter<crate::debris::crumble::Shed>,
    mut commands: Commands,
    stages: Option<Res<Stages>>,
    keep: Option<Res<KeepGive>>,
) {
    let _release = Stages::start(stages.as_deref(), Stage::Release);
    let dt = f64::from(fixed.timestep().as_secs_f32());
    let tick = clock.as_deref().map_or(0, |clock| clock.0.raw());
    let Pressed {
        blocks,
        held,
        belts,
    } = std::mem::take(&mut *pressed);
    account.blocks.clear();
    account.held = held;
    account.freed.clear();
    account.shed = 0;
    let mut kept = if keep.is_some() {
        give.0.clone()
    } else {
        BTreeMap::new()
    };
    let mut leaving: BTreeMap<Entity, BTreeSet<usize>> = BTreeMap::new();
    for (root, cells) in belts {
        leaving.entry(root).or_default().extend(cells);
    }
    for block in blocks {
        // What the body crossed is how far the step moved it into the cells: its speed as the
        // solve left it, which a third body pushing it inside the step made. The way that gave
        // is the one that returns least at that speed.
        let velocity = movers
            .get(block.body)
            .map_or(DVec3::ZERO, |(linear, angular)| {
                (block.to_owner * (linear.0 + angular.0.cross(block.arm))).as_dvec3()
            });
        let returned = |given: &Yield| {
            reaction(given.price, block.spent, velocity.dot(given.push.along), dt) / given.push.lean
        };
        let Yield {
            way,
            push,
            price,
            leaves,
            grounded,
            ..
        } = (block.gives.into_iter())
            .min_by(|a, b| returned(a).total_cmp(&returned(b)))
            .expect("a block holds a way to give");
        let closing = velocity.dot(push.along);
        let impulse = reaction(price, block.spent, closing, dt);
        let leans: Vec<f64> = (block.points.iter())
            .map(|(_, into, _)| into.dot(push.along))
            .collect();
        let loads: Vec<f64> = block.points.iter().map(|&(_, _, load)| load).collect();
        let at_bound: f64 = (block.points.iter())
            .zip(shares(&leans, &loads))
            .filter(|&(&((contact, manifold, point), ..), _)| {
                bounds
                    .held(ContactId(contact), manifold)
                    .is_some_and(|held| held[point])
            })
            .map(|(_, share)| share)
            .sum();
        let yielded = at_bound >= 1.0 - EVEN_SHARE;
        let spent = if yielded {
            block.spent + crossed(price, closing, dt)
        } else if keep.is_some() {
            block.spent
        } else {
            0.0
        };
        if let Ok((_, _, _, owner)) = owners.get(block.root) {
            account.blocks.push(BlockRow {
                body: block.body,
                construction: owner.construction,
                cells: block.cells.len(),
                way,
                price: [price.force, price.work],
                closing,
                bound: impulse,
                yielded,
                spent,
                along: push.along,
                grounded,
            });
        }
        if spent >= 1.0 {
            leaving.entry(block.root).or_default().extend(leaves);
        } else if yielded {
            for c in block.cells {
                let share = kept.entry((block.root, c)).or_insert(0.0_f64);
                *share = share.max(spent);
            }
        }
    }
    give.0 = kept;
    for (root, cells) in leaving {
        let Ok((_, position, rotation, mut owner)) = owners.get_mut(root) else {
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
        let cut = cut_fragments(&mut owner, plug, tick);
        // A construction without children has no chunk collider to rehome.
        let reusable = reusable_colliders(children.get(root).ok(), &chunk_entities);
        let (departures, sheds) = sweep_departure(
            &mut owner,
            pose,
            &cut,
            ReleasedMotion::at_rest(pose.translation),
            tick,
            &mut record,
            &reusable,
        );
        account
            .freed
            .extend(cut.plug.iter().map(|&c| (construction, c as u16)));
        account.shed += sheds.len();
        shed.write_batch(sheds);
        queue.0.push(RawDeposit {
            changed: vec![root],
            ..default()
        });
        for departure in departures {
            let piece = commands.spawn_empty().id();
            spawn_prepared_as(&mut commands, piece, departure);
        }
    }
}

/// Before the step: each belt's force on standing matter against what that matter can return
/// (module doc). `StagedForce::force` is the force on the belt's body.
fn bound_belts(
    transfers: Res<ContactTransfers>,
    owned: Owned,
    mut owners: Owners,
    mut bodies: Query<(Forces, &ComputedCenterOfMass)>,
    fixed: Res<Time<Fixed>>,
    mut pressed: ResMut<Pressed>,
    mut account: ResMut<BoundAccount>,
    stages: Option<Res<Stages>>,
    bare: Option<Res<BareShed>>,
) {
    let _bound = Stages::start(stages.as_deref(), Stage::Bound);
    account.capped_belts.clear();
    pressed.belts.clear();
    let dt = f64::from(fixed.timestep().as_secs_f32());
    let mut staged = BTreeMap::<(Entity, Entity), Vec<(usize, Vec3, Vec3)>>::new();
    for force in transfers.staged() {
        let (_, element, chunk) = owned
            .get(force.structure_collider)
            .expect("a staged belt force names a contact collider of its owner");
        let Ok((_, position, rotation, owner)) = owners.get(force.structure) else {
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
        let (_, position, rotation, mut owner) = owners
            .get_mut(root)
            .expect("the owner was read from this query");
        let owner = owner.bypass_change_detection();
        let pose = owner.physics_frame((position, rotation));
        let (mut source, centre) = bodies
            .get_mut(body)
            .expect("a belt's body is a dynamic body");
        let to_owner = pose.rotation.inverse();
        let mut cells: Vec<usize> = forces.iter().map(|&(cell, ..)| cell).collect();
        cells.sort_unstable();
        cells.dedup();
        let live = &mut *owner.live;
        let standing = Standing {
            structure: &live.structure,
            state: &live.state,
            shapes: &owner.kind.layout.blocks,
            preload: live
                .gravity
                .get(&live.structure, &live.state, &live.assessment),
            bare: bare.is_some(),
        };
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
                continue;
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
            account.capped_belts.push((body, -total * back));
            pressed.belts.push((root, given.leaves));
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
