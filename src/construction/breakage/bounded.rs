//! AUTHORITY: the game's building model meeting bodies (game state only). A standing cell returns
//! a body no more than its bonds can ([`fracture::bound`]), and its bonds take the work the body
//! does on it; a block one of whose ways of giving is spent ([`settle`]) is freed at rest and is
//! ordinary rubble from the next step on. Nothing is predicted: the cells are the ones the narrow
//! phase found pressed, and no rule reads which dynamic body presses. A static or kinematic body
//! is not read: the solver moves neither it nor a fixed owner, so their contact returns nothing
//! to bound.
//!
//! * [`bound_contacts`], inside the physics step after the narrow phase: groups each body's
//!   contact points on one construction into blocks of connected pressed cells, prices each
//!   ([`Standing::give_along`]) and bounds what its points return between them, wherever the body
//!   presses it, at that price (`avian_ext::reaction_bound`). A point the law does not move (an
//!   unbreakable element, a buried cell, a face slid along) stays rigid.
//! * [`bound_belts`], before the step: the force a belt takes from standing matter, which has no
//!   solver point, is bounded by the same price; the belt keeps what the cells can return.
//! * [`release`], after the step: counts the step's work on each cell ([`Live::work`]) and frees
//!   the blocks a way of which is spent ([`settle`]).
//!
//! One physics step runs between two [`release`]s: [`Pressed`] holds one step's points and group
//! indices, and `Bounds` holds a step's impulses and groups only until the next step starts.
//! [`bound_contacts`] asserts it.
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

use super::fracture::bound::{Push, Standing, Way, Yield, taken};
use super::*;
use crate::avian_ext::reaction_bound::{Bound, Bounds, Member};
use bevy::tasks::{ParallelSlice, ParallelSliceMut};

/// One body's connected pressed cells on one construction, as the step priced them.
#[derive(Debug, Clone)]
struct Block {
    body: Entity,
    /// Ordered.
    cells: Vec<usize>,
    /// Each push's cheapest way to give, and its group of bounded points (`Bounds::group`).
    gives: Vec<(Yield, usize)>,
}

/// The speed of a body moving at `linear` and turning at `angular` (world) into a cell along
/// unit `into`, at `arm` from its centre of mass, m/s.
fn speed_into(linear: Vec3, angular: Vec3, arm: Vec3, into: Vec3) -> f32 {
    (linear + angular.cross(arm)).dot(into)
}

/// What a point returned over the step.
#[derive(Debug, Clone, Copy)]
enum Returned {
    /// A solver point's normal impulse, read after the solve ([`Bounds::impulse`]): its contact
    /// pair, manifold and point.
    Solved(ContactId, usize, usize),
    /// What a belt's body kept of a staged force, N·s. The belt law has no solver point and
    /// stages its friction itself, so the force is its support and both tangent terms
    /// (`track::rubble::source_force`); [`bound_belts`] bounds the sum of a block's forces whole,
    /// and the work is counted along each whole force, where a solver point's is its normal
    /// impulse's alone.
    Staged(f32),
}

/// One body's press on one standing cell over a step: a solver contact point, or a staged belt
/// force.
#[derive(Debug, Clone, Copy)]
struct Point {
    body: Entity,
    cell: usize,
    /// Where it presses, from the body's centre of mass at the step's start, m, world.
    arm: Vec3,
    /// Unit, from the body into the cell, world.
    into: Vec3,
    /// The body's speed into the cell there at the step's start, before the step's forces, m/s.
    entering: f32,
    returned: Returned,
}

/// What pressed one owner over a step, in the order the step met it: a belt's forces before the
/// contacts, bodies in entity order.
#[derive(Default, Debug)]
struct Press {
    points: Vec<Point>,
    blocks: Vec<Block>,
    /// The cells (ordered) each belt bears on as one block, how they give to it, and whether
    /// the belt pressed harder than the most they return ([`Yield::impulse`]).
    belts: Vec<(Vec<usize>, Yield, bool)>,
}

/// What pressed each owner since the last [`release`], which takes it every tick.
#[derive(Resource, Default, Debug)]
pub(crate) struct Pressed(BTreeMap<Entity, Press>);

/// One block's step, for the operator's instruments: the way it parted by, or the way its work
/// reaches first. Work is kept per cell, never per way, and a step's impulse belongs to points
/// that several ways' groups share, so no way is the one that took the step's work: `closing`,
/// `bound` and `returned` are this way's, and it may have carried nothing this step.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct BlockRow {
    pub(crate) body: Entity,
    pub(crate) cells: usize,
    pub(crate) way: Way,
    /// N and J ([`fracture::bound::Price`]).
    pub(crate) price: [f64; 2],
    /// The body's speed along the push at the step's start, m/s.
    pub(crate) closing: f64,
    /// What the way's price returns along the push over a step at `closing`, N·s
    /// ([`fracture::bound::reaction`]); the solver is dealt the way's strength and its work left.
    pub(crate) bound: f64,
    /// The work its cells' bonds have taken after the step, J.
    pub(crate) taken: f64,
    /// What its points returned along the push over the step, N·s.
    pub(crate) returned: f32,
}

/// The last step's account, for headless witnesses and operator instruments.
#[derive(Resource, Default, Debug)]
pub(crate) struct BoundAccount {
    pub(crate) blocks: Vec<BlockRow>,
    /// Freed cells: construction and chunk.
    pub(crate) freed: Vec<(u16, u16)>,
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

/// A body the solver holds: its `SolverBodyIndex` requires the integration data.
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
        &'static VelocityIntegrationData,
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
    /// Unit, from the body into the cell, world.
    toward: Vec3,
    /// The same in the owner's frame.
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

/// One body's press on one owner as priced: every point on a standing cell, and each block of
/// connected cells with the ways it gives, each way's points and its bound. A block no way is
/// open for is not among them: its cells hold whatever presses them.
struct Priced {
    points: Vec<Point>,
    blocks: Vec<(Vec<usize>, Vec<(Yield, Vec<Member>, Bound)>)>,
}

/// One owner as [`bound_contacts`] prices the bodies on it.
#[derive(Clone, Copy)]
struct Pricing<'a, 'w, 's> {
    standing: Standing<'a>,
    /// The world's rotation into the owner's frame.
    to_owner: Quat,
    movers: &'a Movers<'w, 's>,
    gravity: Vec3,
    dt: f64,
}

impl Pricing<'_, '_, '_> {
    /// Prices the cells `body` presses at `touches`.
    fn price(&self, body: Entity, touches: &[Touch]) -> Priced {
        let Self {
            standing,
            to_owner,
            dt,
            ..
        } = *self;
        // The velocities are the step's start: Avian adds a step's forces to its solver bodies
        // inside the substep (`integrate_velocities`) and writes them back to these components
        // after the solve.
        let (_, at, facing, linear, angular, centre, increment, local, scale, constant) =
            (self.movers.get(body)).expect("the body was read from this query");
        let (mass, force, local_force, steady, local_steady) = constant;
        let centre = at.0 + facing.0 * centre.0;
        // What Avian integrates before its contact solve: the forces applied to the body, its
        // constant ones and gravity (`pre_process_velocity_increments`).
        let acceleration = increment.linear_increment
            + force.map_or(Vec3::ZERO, |force| force.0 * mass.inverse())
            + steady.map_or(Vec3::ZERO, |steady| steady.0)
            + facing.0
                * (local.map_or(Vec3::ZERO, |local| local.linear)
                    + local_force.map_or(Vec3::ZERO, |force| force.0 * mass.inverse())
                    + local_steady.map_or(Vec3::ZERO, |steady| steady.0))
            + self.gravity * scale.map_or(1.0, |scale| scale.0);
        // Every point on a standing cell is read, in a bounded block or not.
        let points = (touches.iter())
            .map(|touch| {
                let (contact, manifold, point) = touch.point;
                let arm = touch.at - centre;
                Point {
                    body,
                    cell: touch.cell,
                    arm,
                    into: touch.toward,
                    entering: speed_into(linear.0, angular.0, arm, touch.toward),
                    returned: Returned::Solved(ContactId(contact), manifold, point),
                }
            })
            .collect();
        let mut cells: Vec<usize> = touches.iter().map(|touch| touch.cell).collect();
        cells.sort_unstable();
        cells.dedup();
        let mut blocks = Vec::new();
        for cells in standing.connected(&cells) {
            let on: Vec<&Touch> = (touches.iter())
                .filter(|touch| cells.binary_search(&touch.cell).is_ok())
                .collect();
            let met = on.iter().map(|touch| touch.at).sum::<Vec3>() / on.len() as f32;
            let velocity = linear.0 + angular.0.cross(met - centre);
            let pushes = pushes(
                (to_owner * velocity).as_dvec3(),
                (to_owner * acceleration).as_dvec3(),
                &on,
            );
            // Every push bounds the points that lean its way, between them, at what the cells
            // hold and at the work their bonds have left (`Bounds::group`); a point in several
            // groups takes the least, which is the cheapest way at whatever speed the solve
            // finds the body closing.
            let done = taken(standing.work, &cells);
            let gives: Vec<_> = (pushes.iter())
                .filter_map(|&push| standing.give_along(&cells, push, dt))
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
                        work: given.price.left(done) as f32,
                    };
                    (given, members, most)
                })
                .collect();
            if !gives.is_empty() {
                blocks.push((cells, gives));
            }
        }
        Priced { points, blocks }
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
        work: &live.work,
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
    bounds: ResMut<'w, Bounds>,
    pressed: ResMut<'w, Pressed>,
    stages: Option<Res<'w, Stages>>,
}

/// Resolution: owners are priced as tasks from this many; one owner has nothing to split. Not
/// measured: a higher floor waits on a profile of a step that presses several owners (backlog
/// destruction-07).
const PARALLEL_OWNERS: usize = 2;

/// `work` on each of `owners`, in their order whichever of `pool`'s workers takes which. An
/// owner's work reads its own state and the step's immutable inputs alone, so each is a task.
fn per_owner<T: Send, R: Send + 'static>(
    pool: &TaskPool,
    mut owners: &mut [T],
    work: impl Fn(&mut T) -> R + Sync,
) -> Vec<R> {
    if owners.len() < PARALLEL_OWNERS {
        return owners.iter_mut().map(work).collect();
    }
    let done = owners.par_splat_map_mut(pool, None, |_, owners| {
        owners.iter_mut().map(&work).collect::<Vec<R>>()
    });
    done.into_iter().flatten().collect()
}

/// Inside the physics step, between the narrow phase and the solver's constraints (module doc).
/// Each body's cells are priced from the owner's state and its own motion alone, so owners
/// ([`per_owner`]) and the bodies on each are priced in parallel; their bounds are handed over
/// in owner and body order.
fn bound_contacts(mut p: BoundContacts) {
    let _bound = Stages::start(p.stages.as_deref(), Stage::Bound);
    assert!(
        (p.pressed.0.values()).all(|press| {
            let staged = |point: &Point| matches!(point.returned, Returned::Staged(_));
            press.blocks.is_empty() && press.points.iter().all(staged)
        }),
        "a second physics step before `release` took the first one's points: `Bounds` holds a \
         step's impulses and groups only until the next step starts"
    );
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
                let toward = sign * manifold.normal;
                let into = (pose.rotation.inverse() * toward).as_dvec3();
                for (n, point) in manifold.points.iter().enumerate() {
                    // Matter the law does not move answers rigidly.
                    if let Some(cell) = cell(owner, &pose, element, chunk, point.point) {
                        (touches.entry(root).or_default().entry(body).or_default()).push(Touch {
                            point: (pair.contact_id.0, m, n),
                            cell,
                            at: point.point,
                            toward,
                            into,
                        });
                    }
                }
            }
        }
    }
    let mut owners: Vec<_> = (p.owners.iter_many_unique_mut(touches.keys()))
        .map(|owner| owner.expect("the owner was read from this query"))
        .collect();
    let mut pressed: Vec<_> = (owners.iter_mut().zip(touches))
        .map(|((_, position, rotation, owner), (root, bodies))| {
            let owner = owner.bypass_change_detection();
            let to_owner = owner.physics_frame((position, rotation)).rotation.inverse();
            let bodies: Vec<(Entity, Vec<Touch>)> = bodies.into_iter().collect();
            (root, to_owner, owner, bodies)
        })
        .collect();
    let (movers, gravity) = (&p.movers, p.gravity.0);
    let pool = ComputeTaskPool::get();
    let priced = per_owner(pool, &mut pressed, |(_, to_owner, owner, bodies)| {
        let pricing = Pricing {
            standing: standing(owner),
            to_owner: *to_owner,
            movers,
            gravity,
            dt,
        };
        bodies.par_splat_map(pool, None, |_, bodies| {
            (bodies.iter())
                .map(|(body, touches)| (*body, pricing.price(*body, touches)))
                .collect::<Vec<(Entity, Priced)>>()
        })
    });
    for ((root, ..), priced) in pressed.iter().zip(priced) {
        let press = p.pressed.0.entry(*root).or_default();
        for (body, Priced { points, blocks }) in priced.into_iter().flatten() {
            for point in &points {
                if let Returned::Solved(contact, manifold, point) = point.returned {
                    p.bounds.read(contact, manifold, point);
                }
            }
            press.points.extend(points);
            press
                .blocks
                .extend(blocks.into_iter().map(|(cells, gives)| {
                    let gives = (gives.into_iter())
                        .map(|(given, members, most)| (given, p.bounds.group(members, most)))
                        .collect();
                    Block { body, cells, gives }
                }));
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
    entries: &mut Vec<Entry>,
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
        entries.push(Entry {
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
    for &c in fragments.iter().flatten() {
        live.state.fate[c] = Fate::Fallen;
        live.swapped[usize::from(kind.layout.element_of[c])] = true;
    }
    fragments
}

/// `fragments` of `breakage` ([`cut`]) leaving at rest from `pose`, each a prepared body.
pub(super) fn depart(
    breakage: &Breakage,
    pose: Transform,
    fragments: Vec<Vec<usize>>,
    tick: u32,
    reusable: &ReusableColliders,
) -> Vec<PreparedDeparture> {
    ComputeTaskPool::get().scope(|scope| {
        let kind = &breakage.kind;
        for group in fragments {
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
    })
}

#[derive(bevy::ecs::system::SystemParam)]
struct Release<'w, 's> {
    pressed: ResMut<'w, Pressed>,
    bounds: Res<'w, Bounds>,
    account: ResMut<'w, BoundAccount>,
    owners: Owners<'w, 's>,
    movers: Query<'w, 's, (&'static LinearVelocity, &'static AngularVelocity)>,
    children: Query<'w, 's, &'static Children>,
    chunk_entities: ChunkChildren<'w, 's>,
    clock: Option<Res<'w, ShotClock>>,
    record: ResMut<'w, Record>,
    queue: ResMut<'w, Queue>,
    commands: Commands<'w, 's>,
    stages: Option<Res<'w, Stages>>,
}

/// One owner's step settled. Each of `press`'s points adds to its cell in `work` (per chunk, J),
/// in their order, the impulse it returned times the body's speed into the cell there, the mean
/// of the step's start and end: under forces constant over a step a body's speed is linear in
/// time, so that is the work the impulse did, whatever else pushed the body. A body backing out
/// on that mean did none.
///
/// A block then leaves by the first of its ways, cheapest in work first, that is spent: its
/// cells' work has reached the way's; or the body took the last work the way dealt one of its
/// points and kept closing. The second is two readings: at the step's start the body was
/// closing on the block's mean pressed point fast enough to cross the give the way had left
/// within the step ([`Yield::crossing`]), and the step's last solve left a point of the way
/// holding an impulse at the work bound the way dealt it (`Bounds::limited`); a belt pressed
/// harder than the ration that crossing left it. It holds no tolerance, and it is one point at
/// the last substep, not a sum of the step's work. The solve rations a way's last work at its
/// own closing speed, which the count above does not read, so the count alone lands under the
/// way's work on every such step and never reaches it. The cells a block presses all leave with
/// it, so the work not counted has no cell left to be written on. Returns the contact blocks'
/// rows and the cells that leave.
fn settle(
    work: &mut [f64],
    press: &Press,
    bounds: &Bounds,
    movers: &Query<(&LinearVelocity, &AngularVelocity)>,
) -> (Vec<BlockRow>, BTreeSet<usize>) {
    for point in &press.points {
        let Ok((linear, angular)) = movers.get(point.body) else {
            continue; // The body left the world inside the tick: it advanced into nothing.
        };
        let leaving = speed_into(linear.0, angular.0, point.arm, point.into);
        let advance = 0.5 * (point.entering + leaving);
        let returned = match point.returned {
            Returned::Solved(contact, manifold, at) => bounds.impulse(contact, manifold, at),
            Returned::Staged(kept) => kept,
        };
        let joules = f64::from(returned) * f64::from(advance);
        assert!(
            joules.is_finite(),
            "work is kept for good, and body {} did {joules} J on cell {} (returned {returned} \
             N·s, advancing {advance} m/s)",
            point.body,
            point.cell
        );
        work[point.cell] += joules.max(0.0);
    }
    let mut leaving = BTreeSet::new();
    let mut rows = Vec::with_capacity(press.blocks.len());
    for block in &press.blocks {
        let taken = taken(work, &block.cells);
        let mut ways: Vec<&(Yield, usize)> = block.gives.iter().collect();
        ways.sort_by(|(a, _), (b, _)| a.price.work.total_cmp(&b.price.work));
        let parted = (ways.iter().copied()).find(|(given, group)| {
            given.price.spent(taken) || given.crossing && bounds.limited(*group)
        });
        let (given, group) = parted.unwrap_or_else(|| {
            (ways.first()).expect("`Pricing::price` drops a block no way is open for")
        });
        rows.push(BlockRow {
            body: block.body,
            cells: block.cells.len(),
            way: given.way,
            price: [given.price.force, given.price.work],
            closing: given.push.closing,
            bound: given.impulse,
            taken,
            returned: bounds.returned(*group),
        });
        if parted.is_some() {
            leaving.extend(&given.leaves);
        }
    }
    for (cells, given, pressed_past) in &press.belts {
        if given.price.spent(taken(work, cells)) || given.crossing && *pressed_past {
            leaving.extend(&given.leaves);
        }
    }
    (rows, leaving)
}

/// After the step: count the work it did on standing cells and free the blocks whose bonds are
/// spent (module doc). An owner's account and its cut read and write its own state, and read the
/// solve's results and the bodies' velocities, so owners are settled and cut in parallel
/// ([`per_owner`]); their record entries, deposits and departures are committed in owner order.
fn release(mut p: Release) {
    let _release = Stages::start(p.stages.as_deref(), Stage::Release);
    // The net protocol owns the shot clock; an authority composed without it (the offline
    // composition of `headless_test::headless_app_on`) records at tick zero, as its other laws do.
    let tick = p.clock.as_deref().map_or(0, |clock| clock.0.raw());
    p.account.blocks.clear();
    p.account.freed.clear();
    let pressed = std::mem::take(&mut p.pressed.0);
    let found: Vec<_> = p.owners.iter_many_unique_mut(pressed.keys()).collect();
    let mut owners: Vec<_> = (found.into_iter().zip(pressed))
        // An owner that left the world inside the step took its cells with it.
        .filter_map(|(owner, (root, press))| Some((root, owner.ok()?, press)))
        .collect();
    let (bounds, movers) = (&*p.bounds, &p.movers);
    let settled = per_owner(
        ComputeTaskPool::get(),
        &mut owners,
        |(_, (.., owner), press)| {
            // The work is no reader's change: what a view or the wire carries changes when cells
            // leave, in the cut.
            let work = &mut owner.bypass_change_detection().live.work;
            let (rows, leaving) = settle(work, press, bounds, movers);
            // Another law may have freed a cell first.
            let plug: Vec<usize> = (leaving.into_iter())
                .filter(|&c| owner.live.state.standing(c))
                .collect();
            let mut entries = Vec::new();
            let fragments = if plug.is_empty() {
                Vec::new()
            } else {
                cut(owner, &plug, tick, &mut entries)
            };
            (rows, plug, fragments, entries)
        },
    );
    for ((root, (_, position, rotation, owner), _), (rows, plug, fragments, entries)) in
        owners.into_iter().zip(settled)
    {
        p.account.blocks.extend(rows);
        if plug.is_empty() {
            continue; // Nothing parted.
        }
        p.record.entries.extend(entries);
        let pose = owner.physics_frame((position, rotation));
        // A construction without children has no chunk collider to rehome.
        let reusable = reusable_colliders(p.children.get(root).ok(), &p.chunk_entities);
        let departures = depart(&owner, pose, fragments, tick, &reusable);
        (p.account.freed).extend(plug.iter().map(|&c| (owner.construction, c as u16)));
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
    stages: Option<Res<'w, Stages>>,
}

/// One belt body's forces on one owner, as staged: the cell each bears on, where (world), and
/// the force on the body, N.
struct Belt {
    body: Entity,
    /// The body's centre of mass, m, and its linear and angular velocity, world.
    motion: [Vec3; 3],
    forces: Vec<(usize, Vec3, Vec3)>,
}

/// A belt's forces on one block of cells against what they return.
struct Borne {
    body: Entity,
    /// The body's centre of mass, m, world.
    centre: Vec3,
    /// The body's linear and angular velocity at the step's start, world.
    motion: [Vec3; 2],
    /// The cells, ordered, and how they give to the belt; no way when none is open, and the
    /// cells bear whatever the belt presses with.
    cells: Vec<usize>,
    given: Option<Yield>,
    /// The share of each force the body keeps: all of it up to the cells' bound.
    kept: f64,
    /// The cell each force bears on, where it is applied (world), and the force, N.
    forces: Vec<(usize, Vec3, Vec3)>,
}

/// The blocks of `standing` that `belt` presses, `to_owner` the world's rotation into the
/// owner's frame.
fn borne(standing: Standing, to_owner: Quat, belt: &Belt, dt: f64) -> Vec<Borne> {
    let [centre, linear, angular] = belt.motion;
    let mut cells: Vec<usize> = belt.forces.iter().map(|&(cell, ..)| cell).collect();
    cells.sort_unstable();
    cells.dedup();
    (standing.connected(&cells).into_iter())
        .map(|cells| {
            let forces: Vec<_> = (belt.forces.iter().copied())
                .filter(|(cell, ..)| cells.binary_search(cell).is_ok())
                .collect();
            let total: Vec3 = forces.iter().map(|&(_, _, force)| force).sum();
            let met = forces.iter().map(|&(_, at, _)| at).sum::<Vec3>() / forces.len() as f32;
            let velocity = linear + angular.cross(met - centre);
            // No push when the belt's forces on these cells sum to nothing.
            let given = (to_owner * -total)
                .as_dvec3()
                .try_normalize()
                .and_then(|along| {
                    let push = Push {
                        along,
                        closing: (to_owner * velocity).as_dvec3().dot(along),
                        lean: 1.0,
                    };
                    standing.give_along(&cells, push, dt)
                });
            let pressing = f64::from(total.length());
            let kept = (given.as_ref())
                .map(|given| given.impulse / dt)
                .filter(|&most| pressing > most)
                .map_or(1.0, |most| most / pressing);
            Borne {
                body: belt.body,
                centre,
                motion: [linear, angular],
                cells,
                given,
                kept,
                forces,
            }
        })
        .collect()
}

/// Before the step: each belt's force on standing matter against what that matter can return
/// (module doc). `StagedForce::force` is the force on the belt's body. Owners are priced in
/// parallel ([`per_owner`]); the forces go back to the bodies in owner and body order.
fn bound_belts(mut p: BoundBelts) {
    let _bound = Stages::start(p.stages.as_deref(), Stage::Bound);
    let dt = f64::from(p.fixed.timestep().as_secs_f32());
    let mut staged = BTreeMap::<Entity, BTreeMap<Entity, Vec<(usize, Vec3, Vec3)>>>::new();
    for force in p.transfers.staged() {
        let (_, element, chunk) = (p.owned.get(force.structure_collider))
            .expect("a staged belt force names a contact collider of its owner");
        let Ok((_, position, rotation, owner)) = p.owners.get(force.structure) else {
            continue; // A belt on a moving group pushes it through its body.
        };
        let pose = owner.physics_frame((position, rotation));
        if let Some(cell) = cell(owner, &pose, element, chunk, force.point) {
            ((staged.entry(force.structure).or_default())
                .entry(force.body)
                .or_default())
            .push((cell, force.point, force.force));
        }
    }
    let mut owners: Vec<_> = (p.owners.iter_many_unique_mut(staged.keys()))
        .map(|owner| owner.expect("the owner was read from this query"))
        .collect();
    let mut pressed: Vec<_> = (owners.iter_mut().zip(staged))
        .map(|((_, position, rotation, owner), (root, belts))| {
            let owner = owner.bypass_change_detection();
            let to_owner = owner.physics_frame((position, rotation)).rotation.inverse();
            let belts: Vec<Belt> = (belts.into_iter())
                .map(|(body, forces)| {
                    let (source, centre) =
                        (p.bodies.get(body)).expect("a belt's body is a dynamic body");
                    Belt {
                        body,
                        motion: [
                            source.position() + source.rotation() * centre.0,
                            source.linear_velocity(),
                            source.angular_velocity(),
                        ],
                        forces,
                    }
                })
                .collect();
            (root, to_owner, owner, belts)
        })
        .collect();
    let borne = per_owner(
        ComputeTaskPool::get(),
        &mut pressed,
        |(root, to_owner, owner, belts)| {
            let standing = standing(owner);
            let borne: Vec<Borne> = (belts.iter())
                .flat_map(|belt| borne(standing, *to_owner, belt, dt))
                .collect();
            (*root, borne)
        },
    );
    for (root, borne) in borne {
        let press = p.pressed.0.entry(root).or_default();
        for block in borne {
            let (mut source, _) =
                (p.bodies.get_mut(block.body)).expect("a belt's body is a dynamic body");
            for (cell, at, force) in block.forces {
                if block.kept < 1.0 {
                    // The belt keeps what the cells return; the rest it never had.
                    source.apply_force_at_point(-force * (1.0 - block.kept) as f32, at);
                }
                let Ok(onto) = Dir3::new(-force) else {
                    continue; // The belt bears nothing here.
                };
                let arm = at - block.centre;
                let [linear, angular] = block.motion;
                press.points.push(Point {
                    body: block.body,
                    cell,
                    arm,
                    into: *onto,
                    entering: speed_into(linear, angular, arm, *onto),
                    returned: Returned::Staged(
                        (f64::from(force.length()) * block.kept * dt) as f32,
                    ),
                });
            }
            if let Some(given) = block.given {
                press.belts.push((block.cells, given, block.kept < 1.0));
            }
        }
    }
}

pub(super) fn plugin(app: &mut App) {
    app.init_resource::<Pressed>()
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
