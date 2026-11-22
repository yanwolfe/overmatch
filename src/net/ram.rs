//! Tracked full-speed ram instrument for the game's environment destruction model, game state only.
//! `scripts/perf/collapse.py --ram` owns `OVERMATCH_RAM`. Default-feature servers derive the
//! run-up from their loaded drive data and resolve ground height at spawn, without view assets.
//!
//! CONTACT. A recipe states how the hull meets the instance ([`Contact`]): at full throttle from
//! a run-up long enough for governed speed, at a speed the scripted driver holds, or from rest
//! against it. The run-up is the free distance between the hull's nearest point and the face it
//! meets, so a deep construction costs no more of it than a thin one.
//!
//! HULL. Each row reads the ram's one tank after the tick's physics step, outside the tick clock
//! ([`HullRow`]): its motion, and what moved it along its forward over that step, from the step's
//! solver points ([`ContactTransfers`], held until the next step) and the bounded blocks'
//! account ([`BoundAccount`]). A body
//! retired between the step and the row has no kind left to read and counts as `retired`.

use avian3d::prelude::{LinearVelocity, Position, Rotation, Sleeping};
use bevy::prelude::*;
use bevy::tasks::{ComputeTaskPool, ParallelSlice, ParallelSliceMut};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use super::contact_census::{ContactTick, StepSnapshot};
use super::harness::{LeverError, env_text};
use crate::{
    command::ParsedTankCommand,
    construction::{
        Body, ConstructionKind,
        breakage::{Breakage, GRADE_M, Piece, bounded::BoundAccount},
        fracture::bound::Way,
    },
    contact_transfer::ContactTransfers,
    cost::{
        TickSample, Trace, phases,
        stages::{self, NamedCounts, NamedTimes},
    },
    debris::DebrisPiece,
    map::{InstanceRecord, MapManifest, Prototype},
    provenance::{Labelled, Provenance, with_shipped_policy},
    scatter::MapInstance,
    spec::TankSpec,
    tank::{Tank, TankTemplate},
    terrain_grid::{HeightGrid, Stance, spawn_pos},
    track::sim::TrackDrive,
};

mod structure;

const LEVER: &str = "OVERMATCH_RAM";

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub(super) struct Recipe {
    instance: String,
    heading_deg: f64,
    /// The approach line's offset from the instance's origin to the heading's right, m.
    lateral_m: f64,
    contact: Contact,
    /// No-contact limit on the server's ram capture clock (`tick * dt`), from tank spawn;
    /// includes client readiness and the harness lead-in before driving.
    approach_s: f64,
    deadline_s: f64,
    settle_s: f64,
    /// The structure reading's one in-place limit, m ([`structure`], IN PLACE): a `Unit`.
    in_place_m: Labelled<f64>,
}

/// How the hull meets the instance (module doc, CONTACT), and the speed entering first contact
/// that admits the run, m/s. A driven contact's `runup_margin` is the operator's margin on the
/// ideal constant-top-gear-force acceleration distance to its speed ([`top_gear`]).
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
enum Contact {
    /// Full throttle over the run-up to governed speed; admitted at `minimum_fraction` of it.
    Governed {
        minimum_fraction: f64,
        runup_margin: f64,
    },
    /// The scripted driver holds `speed_m_s` (`harness::Drive::Ram`, `hold_m_s`) over the run-up
    /// to that speed and through the contact; admitted within `within_m_s` of it.
    Held {
        speed_m_s: f64,
        within_m_s: f64,
        runup_margin: f64,
    },
    /// Full throttle from a standing hull whose nearest point is `gap_m` short of the instance;
    /// admitted at `at_most_m_s` or under.
    Rest { gap_m: f64, at_most_m_s: f64 },
}

impl Contact {
    fn valid(self) -> bool {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        let margin = |v: f64| v.is_finite() && v >= 1.0;
        match self {
            Self::Governed {
                minimum_fraction,
                runup_margin,
            } => 0.0 < minimum_fraction && minimum_fraction <= 1.0 && margin(runup_margin),
            Self::Held {
                speed_m_s,
                within_m_s,
                runup_margin,
            } => {
                positive(speed_m_s)
                    && positive(within_m_s)
                    && within_m_s < speed_m_s
                    && margin(runup_margin)
            }
            Self::Rest { gap_m, at_most_m_s } => positive(gap_m) && positive(at_most_m_s),
        }
    }
}

impl Recipe {
    fn parse(text: &str) -> Result<Self, LeverError> {
        let invalid = || {
            LeverError::invalid(
                LEVER,
                text,
                "JSON ram recipe: nonempty instance, finite heading_deg and lateral_m, contact as {\"Governed\": {\"minimum_fraction\": in (0,1], \"runup_margin\": >= 1}}, {\"Held\": {\"speed_m_s\": > 0, \"within_m_s\": in (0, speed_m_s), \"runup_margin\": >= 1}} or {\"Rest\": {\"gap_m\": > 0, \"at_most_m_s\": > 0}}, positive approach_s/deadline_s/settle_s with settle_s < deadline_s, in_place_m as [metres > 0, {\"Unit\": \"why this scale\"}]",
            )
        };
        let recipe: Self =
            with_shipped_policy(|| serde_json::from_str(text)).map_err(|_| invalid())?;
        if recipe.instance.is_empty()
            || !recipe.heading_deg.is_finite()
            || !recipe.lateral_m.is_finite()
            || !recipe.contact.valid()
            || [recipe.approach_s, recipe.deadline_s, recipe.settle_s]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.0)
            || recipe.settle_s >= recipe.deadline_s
            || !matches!(recipe.in_place_m.label, Provenance::Unit(_))
            || !(recipe.in_place_m.value.is_finite() && recipe.in_place_m.value > 0.0)
        {
            return Err(invalid());
        }
        Ok(recipe)
    }

    /// The approach's rotation: it turns -Z onto the heading.
    fn rotation(&self) -> Quat {
        Quat::from_rotation_y(self.heading_deg.rem_euclid(360.0).to_radians() as f32)
    }

    /// The instance and its construction in `manifest`; refused when it names none there.
    fn construction<'m>(
        &self,
        manifest: &'m MapManifest,
    ) -> Result<(&'m InstanceRecord, &'m ConstructionKind), LeverError> {
        let missing = || {
            LeverError::invalid(
                LEVER,
                &self.instance,
                "instance must name a construction in the active map",
            )
        };
        let instance = (manifest.instances.iter())
            .find(|i| i.id == self.instance)
            .ok_or_else(missing)?;
        let Prototype::Construction { asset } = &manifest.prototypes[&instance.prototype] else {
            return Err(missing());
        };
        let kind = (manifest.constructions.get(asset))
            .expect("map load resolved every construction its instances name");
        Ok((instance, kind))
    }
}

#[derive(Serialize)]
struct Plan {
    spawn: [f32; 3],
    rotation: [f32; 4],
    /// DERIVED: how far the hull's nearest point stands from the face it meets, along the
    /// heading, m (module doc, CONTACT).
    runup_m: f32,
    governed_m_s: f32,
    /// The least and, where the contact bounds it, the greatest speed entering first contact
    /// that admits the run, m/s.
    admitted_m_s: (f64, Option<f64>),
    /// DERIVED: where the hull's root has carried its rear past the construction, along the
    /// heading from the map origin, m: the construction's far bound plus the tank's stance reach.
    clear_m: f32,
    panel: structure::Panel,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
enum End {
    Rest,
    Deadline,
    /// The target or the ram's one tank went, or another tank joined.
    WorldChanged,
    ApproachTimeout,
}

#[derive(Resource)]
pub(super) struct Ram {
    recipe: Recipe,
    plan: Option<Plan>,
    started: bool,
    /// The last row's hull speeds; none before the capture's first row.
    last: Option<Speeds>,
    impact: Option<Value>,
    impact_tick: Option<u64>,
    resting_s: f64,
    done: Option<End>,
    /// The struck construction once the capture ends at rest or its deadline.
    structure: Option<structure::Reading>,
    target: Option<Entity>,
    pub(super) trace: Trace<RamWork>,
}

#[derive(Serialize)]
pub(super) struct RamWork {
    work: NamedCounts,
    contacts: ContactTick,
    hull: HullRow,
}

/// A row's hull speeds, m/s.
#[derive(Clone, Copy)]
struct Speeds {
    forward: f32,
    horizontal: f32,
}

/// The ram's tank after one tick's physics step, and what moved it along its forward over that
/// step. The belts' implicit support and traction (`contact_transfer`, IMPLICIT CONTACTS) and
/// gravity are no channel, so the channels do not sum to the hull's momentum change.
#[derive(Serialize)]
pub(super) struct HullRow {
    position_m: [f32; 3],
    velocity_m_s: [f32; 3],
    /// Along the hull's forward.
    forward_m_s: f32,
    /// This row's `forward_m_s` less the last row's; none on the capture's first row.
    forward_change_m_s: Option<f32>,
    /// The hull's position along the recipe heading from the map origin, m.
    along_m: f32,
    /// The parsed throttle and steer the step drove with.
    drive: [f32; 2],
    /// Belt surface speeds, left and right.
    belt_m_s: [f32; 2],
    solver_ns: Channels,
    /// Every block of standing cells the step bounded (`construction::breakage::bounded`).
    blocks: Vec<BlockReadout>,
    /// Cells the step's over-pushed blocks freed.
    freed: usize,
}

/// One bounded block of standing cells over one step.
#[derive(Serialize)]
struct BlockReadout {
    /// The ram's tank presses it; a loose piece otherwise.
    hull: bool,
    /// The pressing body's entity index.
    body: u32,
    cells: usize,
    /// The way it gives soonest at the speed it was crossed.
    way: &'static str,
    force_n: f32,
    work_j: f32,
    /// The pressing body's speed along the push after the step.
    closing_m_s: f32,
    /// The most it returned along the push over the step, N s.
    bound_ns: f32,
    /// It returned all its bound.
    yielded: bool,
    /// The share of its give crossed; one frees it.
    spent: f32,
    /// What its points returned along the push over the step, N s.
    returned_ns: f32,
}

/// Solver momentum into the hull along its forward over one step, N s, by the other body.
#[derive(Serialize, Default, Clone, Copy, Debug)]
struct Channels {
    /// A standing construction.
    standing: f32,
    /// A debris piece.
    piece: f32,
    /// The ground or any other body.
    other: f32,
    /// A body retired between the step and the row (module doc, HULL).
    retired: f32,
}

impl HullRow {
    /// The solver keeps every state finite; a reading that is not is a bug.
    fn assert_finite(&self, tick: u64) {
        let s = self.solver_ns;
        let fields: [(&str, &[f32]); 8] = [
            ("position_m", &self.position_m),
            ("velocity_m_s", &self.velocity_m_s),
            ("forward_m_s", &[self.forward_m_s]),
            ("forward_change_m_s", self.forward_change_m_s.as_slice()),
            ("along_m", &[self.along_m]),
            ("drive", &self.drive),
            ("belt_m_s", &self.belt_m_s),
            ("solver_ns", &[s.standing, s.piece, s.other, s.retired]),
        ];
        for (field, values) in fields {
            assert!(
                values.iter().all(|v| v.is_finite()),
                "ram row {tick}: hull.{field} is not finite: {values:?}"
            );
        }
    }
}

pub(super) fn install(app: &mut App, stats_on: bool) -> Result<(), LeverError> {
    let Some(text) = env_text(LEVER)? else {
        return Ok(());
    };
    let recipe = Recipe::parse(&text)?;
    if super::harness::script().is_some_and(|script| script.spawn_pose.is_some()) {
        return Err(LeverError::invalid(
            LEVER,
            text,
            "one spawn recipe; unset the harness spawn_pose when using OVERMATCH_RAM",
        ));
    }
    if !stats_on {
        return Err(LeverError::invalid(
            LEVER,
            text,
            "set OVERMATCH_SERVER_STATS_S for the ram instrument",
        ));
    }
    install_recipe(app, recipe);
    Ok(())
}

/// The construction instance a scripted client's frames approach (`dev_capture`'s approach
/// anchor) and the approach's rotation, from the recipe `collapse.py --ram` sets on both
/// processes; refused unless it names a construction in this client's own map.
pub(super) fn target(manifest: &MapManifest) -> Result<(String, Quat), LeverError> {
    let text = env_text(LEVER)?.ok_or_else(|| {
        LeverError::invalid(
            LEVER,
            "",
            "the ram recipe whose instance a scripted client's capture frames approach",
        )
    })?;
    let recipe = Recipe::parse(&text)?;
    recipe.construction(manifest)?;
    let heading = recipe.rotation();
    Ok((recipe.instance, heading))
}

fn install_recipe(app: &mut App, recipe: Recipe) {
    app.insert_resource(Ram::new(recipe));
    app.add_systems(PostStartup, prepare);
}

/// The tank's governed speed, m/s, and its ideal top-gear acceleration, m/s²:
/// a = T_peak * top_reduction / (r * mass). DERIVED ideal run-up to a speed v: d = v²/(2a), which
/// excludes terrain and rolling losses and, below top gear, overstates the distance; the recipe's
/// margin buys distance, and measured first-contact speed is the admission gate. It never
/// substitutes an estimated impact speed.
fn top_gear(spec: &TankSpec) -> (f32, f32) {
    let drive = spec
        .track
        .nominal_transmission_params(&spec.roadwheels, spec.mass)
        .expect("spawn uses an admitted tank transmission");
    let top = drive
        .gears_fwd
        .last()
        .expect("transmission admission requires forward gears");
    let acceleration =
        drive.torque_at(drive.peak_torque_rpm) * top / drive.sprocket_radius / spec.mass;
    (drive.governed_speed(false), acceleration)
}

impl Recipe {
    /// The run's plan for a tank of `spec` whose collision construction, posed at a spawn turn
    /// about its root, has the points `points` gives.
    fn plan(
        &self,
        spec: &TankSpec,
        points: impl Fn(Quat) -> Vec<Vec3>,
        manifest: &MapManifest,
        height: Option<&HeightGrid>,
    ) -> Result<Plan, LeverError> {
        let invalid = |why| LeverError::invalid(LEVER, &self.instance, why);
        let (instance, kind) = self.construction(manifest)?;
        let (governed_m_s, acceleration) = top_gear(spec);
        let governed = f64::from(governed_m_s);
        let to_speed = |margin: f64, speed: f64| {
            (margin * speed * speed / (2.0 * f64::from(acceleration))) as f32
        };
        let (runup_m, admitted_m_s) = match self.contact {
            Contact::Governed {
                minimum_fraction,
                runup_margin,
            } => (
                to_speed(runup_margin, governed),
                (governed * minimum_fraction, None),
            ),
            Contact::Held {
                speed_m_s,
                within_m_s,
                runup_margin,
            } => {
                if speed_m_s >= governed {
                    return Err(invalid(
                        "contact.Held.speed_m_s under the tank's governed speed; Governed is the full-speed contact",
                    ));
                }
                let admitted = (speed_m_s - within_m_s, Some(speed_m_s + within_m_s));
                (to_speed(runup_margin, speed_m_s), admitted)
            }
            Contact::Rest { gap_m, at_most_m_s } => (gap_m as f32, (0.0, Some(at_most_m_s))),
        };
        let rotation = self.rotation();
        let direction = rotation * Vec3::NEG_Z;
        let points = points(rotation);
        let stance = Stance::enclosing(points.iter().copied());
        let pose = Transform::from_isometry(instance.pose.at_height(0.0));
        let (min, max) = kind.bounds();
        let corners = (0..8)
            .map(|k| Vec3::select(BVec3::new(k & 1 > 0, k & 2 > 0, k & 4 > 0), max, min))
            .map(|corner| pose.transform_point(corner));
        let (far, around) = corners.fold((f32::NEG_INFINITY, 0.0_f32), |(far, around), corner| {
            let from_origin = (corner.xz() - instance.pose.xz()).length();
            (far.max(corner.dot(direction)), around.max(from_origin))
        });
        // Each point of the tank on the approach line, in the construction's frame, its root
        // standing at its stance over its own ground and `back` short of the line's foot: far
        // enough that no point has entered the construction.
        let foot = (rotation * Vec3::X) * self.lateral_m as f32;
        let back = around + self.lateral_m.abs() as f32 + stance.reach_m;
        let turn = pose.rotation.inverse();
        let lift = Vec3::Y * (GRADE_M + stance.depth_m);
        let origins = (points.iter()).map(|&point| turn * (point + foot - direction * back) + lift);
        let (panel, ahead_m) =
            structure::Panel::met(kind, origins, turn * direction).map_err(|why| {
                let value = format!(
                    "instance {} heading_deg {} lateral_m {}",
                    self.instance, self.heading_deg, self.lateral_m
                );
                LeverError::invalid(LEVER, value, why)
            })?;
        let xz = instance.pose.xz() + (foot - direction * (back - ahead_m + runup_m)).xz();
        let limit = super::spawn_map::spawn_limit(manifest.extent.half_extent());
        if !xz.is_finite() || xz.abs().max_element() > limit {
            return Err(invalid(
                "derived run-up is outside spawn bounds; choose another approach or instance",
            ));
        }
        let spawn = spawn_pos(height, xz, stance);
        Ok(Plan {
            spawn: spawn.to_array(),
            rotation: rotation.to_array(),
            runup_m,
            governed_m_s,
            admitted_m_s,
            clear_m: far + stance.reach_m,
            panel,
        })
    }
}

/// All synchronous tank/map publishers have flushed before PostStartup; no client is admitted yet.
pub(super) fn prepare(
    mut ram: ResMut<Ram>,
    template: Res<TankTemplate>,
    manifest: Res<MapManifest>,
    height: Option<Res<HeightGrid>>,
) {
    ram.plan = Some(
        ram.recipe
            .plan(
                template.spec(),
                |turn| template.collision_points(turn).collect(),
                &manifest,
                height.as_deref(),
            )
            .unwrap_or_else(super::harness::refuse_start),
    );
    println!(
        "ram_plan {}",
        json!({"recipe": ram.recipe, "plan": ram.plan})
    );
}

impl Ram {
    /// The capture is armed and unfinished.
    pub(super) fn recording(&self) -> bool {
        self.started && self.done.is_none()
    }

    fn new(recipe: Recipe) -> Self {
        Ram {
            recipe,
            plan: None,
            started: false,
            last: None,
            impact: None,
            impact_tick: None,
            resting_s: 0.0,
            done: None,
            structure: None,
            target: None,
            trace: Trace::default(),
        }
    }

    pub(super) fn pose(&mut self) -> (Vec3, Quat) {
        let plan = self
            .plan
            .as_ref()
            .expect("PostStartup checks the ram plan before the server admits clients");
        self.started = true;
        (plan.spawn.into(), Quat::from_array(plan.rotation))
    }

    pub(super) fn json(&mut self) -> Value {
        let (ticks, sends) = self.trace.drain();
        json!({"recipe": self.recipe, "plan": self.plan, "impact": self.impact,
            "done": self.done, "structure": self.structure, "ticks": ticks, "sends": sends})
    }
}

/// What a row reads of the ram's tank (module doc, HULL).
#[derive(bevy::ecs::system::SystemParam)]
pub(super) struct Hull<'w, 's> {
    tanks: Query<'w, 's, Entity, With<Tank>>,
    states: Query<
        'w,
        's,
        (
            &'static Position,
            &'static Rotation,
            &'static LinearVelocity,
            &'static ParsedTankCommand,
            &'static TrackDrive,
        ),
    >,
    kinds: Query<'w, 's, (Has<Body>, Has<DebrisPiece>)>,
    contacts: Res<'w, ContactTransfers>,
    bound: Res<'w, BoundAccount>,
}

impl Hull<'_, '_> {
    /// `tank` now, along unit `heading`, after a row of `last` speeds.
    fn row(&self, tank: Entity, heading: Vec3, last: Option<Speeds>) -> HullRow {
        let (position, rotation, velocity, drive, belts) = self
            .states
            .get(tank)
            .expect("a simulated tank has a pose, a velocity, a parsed command and its belts");
        let forward = rotation.0 * Vec3::NEG_Z;
        let forward_m_s = velocity.0.dot(forward);
        // Summed in the solver's order, which no worker count decides.
        let mut solver_ns = Channels::default();
        for point in self.contacts.solver_groups().flatten() {
            let (body, into_hull) = if point.other == tank {
                (point.reference, point.impulse)
            } else if point.reference == tank {
                (point.other, -point.impulse)
            } else {
                continue; // This constraint moved no momentum into the hull.
            };
            let channel = match self.kinds.get(body) {
                Ok((true, _)) => &mut solver_ns.standing,
                Ok((_, true)) => &mut solver_ns.piece,
                Ok(_) => &mut solver_ns.other,
                Err(_) => &mut solver_ns.retired,
            };
            *channel += into_hull.dot(forward);
        }
        HullRow {
            position_m: position.0.to_array(),
            velocity_m_s: velocity.0.to_array(),
            forward_m_s,
            forward_change_m_s: last.map(|last| forward_m_s - last.forward),
            along_m: position.0.dot(heading),
            drive: [drive.throttle, drive.steer],
            belt_m_s: belts.sides.map(|side| side.speed),
            solver_ns,
            blocks: (self.bound.blocks.iter())
                .map(|block| BlockReadout {
                    hull: block.body == tank,
                    body: block.body.index_u32(),
                    cells: block.cells,
                    way: match block.way {
                        Way::ShearOut => "shear_out",
                        Way::Shed => "shed",
                    },
                    force_n: block.price[0] as f32,
                    work_j: block.price[1] as f32,
                    closing_m_s: block.closing as f32,
                    bound_ns: block.bound as f32,
                    yielded: block.yielded,
                    spent: block.spent as f32,
                    returned_ns: block.returned,
                })
                .collect(),
            freed: self.bound.freed.len(),
        }
    }
}

#[derive(bevy::ecs::system::SystemParam)]
pub(super) struct Census<'w, 's> {
    ram: Option<ResMut<'w, Ram>>,
    hull: Hull<'w, 's>,
    pieces: Query<
        'w,
        's,
        (),
        (
            With<Piece>,
            With<Breakage>,
            Without<Sleeping>,
            Without<crate::debris::freeze::Frozen>,
        ),
    >,
    constructions: Query<'w, 's, (Entity, &'static MapInstance, &'static Breakage), With<Body>>,
    poses: Query<'w, 's, (&'static Position, &'static Rotation)>,
    loose: Query<
        'w,
        's,
        (
            &'static Piece,
            &'static Breakage,
            &'static Position,
            &'static Rotation,
            &'static LinearVelocity,
        ),
    >,
    avian: AvianDiagnostics<'w, 's>,
    capture: ResMut<'w, StepSnapshot>,
}

/// Avian's step diagnostics and the contact state they describe.
#[derive(bevy::ecs::system::SystemParam)]
pub(super) struct AvianDiagnostics<'w, 's> {
    solver: Res<'w, avian3d::dynamics::solver::SolverDiagnostics>,
    visits: Res<'w, crate::avian_ext::solver_cost::ContactVisits>,
    collision: Res<'w, avian3d::collision::CollisionDiagnostics>,
    tree: Res<'w, avian3d::collider_tree::ColliderTreeDiagnostics>,
    graph: Res<'w, avian3d::prelude::ContactGraph>,
    constraints: Res<'w, avian3d::dynamics::solver::constraint_graph::ConstraintGraph>,
    reduction: Res<'w, crate::avian_ext::contact_reduction::ReductionCounts>,
    bodies: Query<'w, 's, &'static avian3d::prelude::RigidBody>,
    degrees: Local<'s, bevy::ecs::entity::EntityHashMap<u32>>,
    workers: Local<'s, Vec<AvianContactWork>>,
}

/// Reusable census output owned by one compute-worker batch.
#[derive(Default)]
struct AvianContactWork {
    touching_pairs: u32,
    points: u32,
    degrees: bevy::ecs::entity::EntityHashMap<u32>,
}

impl AvianDiagnostics<'_, '_> {
    /// One collector snapshot of active contacts and prepared constraints, outside the tick clock.
    fn step(&mut self) -> crate::cost::AvianStep {
        use avian3d::dynamics::solver::constraint_graph::COLOR_OVERFLOW_INDEX;
        let us = |d: std::time::Duration| d.as_secs_f64() * 1.0e6;
        let (s, c) = (&*self.solver, &*self.collision);
        let colors = &self.constraints.colors;
        let graph = &*self.graph;
        let pairs = graph.active_pairs();
        let bodies = &self.bodies;
        let pool = ComputeTaskPool::get();
        let workers = pool.thread_num();
        self.workers.resize_with(workers, AvianContactWork::default);
        // Each batch owns disjoint ranges of both populations and its incidence map.
        self.workers.par_chunk_map_mut(pool, 1, |worker, work| {
            let work = &mut work[0];
            work.touching_pairs = 0;
            work.points = 0;
            work.degrees.clear();
            for pair in &pairs[pairs.len() * worker / workers..pairs.len() * (worker + 1) / workers]
            {
                if pair.is_touching() {
                    work.touching_pairs += 1;
                    work.points += pair
                        .manifolds
                        .iter()
                        .map(|m| m.points.len() as u32)
                        .sum::<u32>();
                }
            }
            for color in colors {
                let constraints = &color.contact_constraints;
                for constraint in &constraints[constraints.len() * worker / workers
                    ..constraints.len() * (worker + 1) / workers]
                {
                    let Some((_, pair)) = graph.get_by_id(constraint.contact_id) else {
                        // Retirement removes pairs before this post-step census, so the
                        // degree excludes their surviving dynamic partners on retirement ticks.
                        // Solver indices cannot recover them: retirement swap-removes bodies
                        // (`avian3d` 0.8.0 `solver_body/plugin.rs`, `remove_solver_body_inner`).
                        continue;
                    };
                    for body in [pair.body1, pair.body2].map(|body| {
                        body.expect("a prepared constraint retains both rigid-body owners")
                    }) {
                        // Retired bodies can leave the world after solving and before collection.
                        if bodies
                            .get(body)
                            .is_ok_and(avian3d::prelude::RigidBody::is_dynamic)
                        {
                            *work.degrees.entry(body).or_default() += 1;
                        }
                    }
                }
            }
        });
        let (mut touching_pairs, mut points) = (0, 0);
        self.degrees.clear();
        // Incidence for a body can span batches; only the keyed reduction is serial.
        for work in &*self.workers {
            touching_pairs += work.touching_pairs;
            points += work.points;
            for (&body, &degree) in &work.degrees {
                *self.degrees.entry(body).or_default() += degree;
            }
        }
        crate::cost::AvianStep {
            collider_tree_update_us: us(self.tree.update),
            collider_tree_optimize_us: us(self.tree.optimize),
            broad_phase_us: us(c.broad_phase),
            narrow_phase_us: us(c.narrow_phase),
            prepare_constraints_us: us(s.prepare_constraints),
            update_velocity_increments_us: us(s.update_velocity_increments),
            integrate_velocities_us: us(s.integrate_velocities),
            warm_start_us: us(s.warm_start),
            solve_constraints_us: us(s.solve_constraints),
            integrate_positions_us: us(s.integrate_positions),
            relax_velocities_us: us(s.relax_velocities),
            apply_restitution_us: us(s.apply_restitution),
            finalize_us: us(s.finalize),
            store_impulses_us: us(s.store_impulses),
            swept_ccd_us: us(s.swept_ccd),
            broad_phase_candidates: c.broad_phase_candidates,
            pairs: c.contact_count,
            touching_pairs,
            points,
            constraints: s.contact_constraint_count,
            colors: colors
                .iter()
                .filter(|color| !color.contact_constraints.is_empty())
                .count() as u32,
            overflow_constraints: colors[COLOR_OVERFLOW_INDEX].contact_constraints.len() as u32,
            serial_contact_visits: self.visits.serial,
            parallel_contact_visits: self.visits.parallel,
            // No live dynamic endpoint means zero incidence.
            max_dynamic_manifold_degree: self.degrees.values().copied().max().unwrap_or(0),
            stationary_non_touching_pairs: c.stationary_non_touching_pairs,
            stationary_non_touching_queries: c.stationary_non_touching_queries,
            recycled_pairs: c.recycled_count,
            reduction_groups: self.reduction.groups,
            reduction_landing_whole: self.reduction.landing_whole,
            reduction_ceiling_whole: self.reduction.ceiling_whole,
            reduction_overlap_whole: self.reduction.overlap_whole,
            reduction_warm_points_removed: self.reduction.warm_removed,
            reduction_cold_points_kept: self.reduction.cold_kept,
        }
    }
}

impl Census<'_, '_> {
    /// Whether this tick will reach a row: the census is taken only then.
    pub(super) fn recording(&self) -> bool {
        self.ram.as_deref().is_some_and(Ram::recording)
    }

    pub(super) fn contact_tick(&mut self) -> ContactTick {
        super::contact_census::finish(&mut self.capture)
    }

    pub(super) fn contact_points(&self) -> usize {
        self.hull.contacts.groups().map(|points| points.len()).sum()
    }

    /// This tick's sample, recorded by the ram and the scheduled collapse alike.
    pub(super) fn sample(
        &mut self,
        tick_us: f64,
        physics_us: f64,
        sleep_us: f64,
        stages: ([f64; stages::COUNT], NamedTimes<{ stages::COUNT }>),
        phases: &phases::Phases,
        dt: f64,
    ) -> TickSample {
        TickSample {
            tick_us,
            physics_us,
            sleep_us,
            stages: NamedTimes::new(&stages::NAMES, stages.0),
            phases_us: phases.times(),
            phases_cpu_us: phases.cpu_times(),
            stages_cpu_us: stages.1,
            tick_cpu_us: stages
                .1
                .values
                .iter()
                .chain(&phases.cpu_times().values)
                .sum(),
            detail: NamedTimes::new(&stages::DETAIL_NAMES, [0.0; stages::DETAIL_COUNT]),
            awake: self.pieces.iter().len(),
            tanks: self.hull.tanks.iter().len(),
            dt_s: dt,
            avian: self.avian.step(),
        }
    }

    /// Records a tick of an armed, unfinished capture, returning its row's `collector_us`.
    pub(super) fn record(
        &mut self,
        sample: TickSample,
        work: NamedCounts,
        contacts: ContactTick,
    ) -> Option<&mut f64> {
        let ram = self.ram.as_deref_mut().filter(|ram| ram.recording())?;
        let target = if let Some(target) = ram.target {
            if !self.constructions.contains(target) {
                ram.done = Some(End::WorldChanged);
                return None;
            }
            target
        } else {
            let target = self
                .constructions
                .iter()
                .find(|(_, id, ..)| id.id == ram.recipe.instance)
                .expect("the admitted ram construction exists throughout its capture")
                .0;
            ram.target = Some(target);
            target
        };
        let Ok(tank) = self.hull.tanks.single() else {
            ram.done = Some(End::WorldChanged);
            return None;
        };
        let plan = ram.plan.as_ref().expect("a started capture has its plan");
        let hull = self.hull.row(
            tank,
            Quat::from_array(plan.rotation) * Vec3::NEG_Z,
            ram.last,
        );
        hull.assert_finite(ram.trace.next_tick());
        let entering = ram.last.replace(Speeds {
            forward: hull.forward_m_s,
            horizontal: Vec2::new(hull.velocity_m_s[0], hull.velocity_m_s[2]).length(),
        });
        let row = ram.trace.push(
            sample,
            RamWork {
                work,
                contacts,
                hull,
            },
        );
        let tick = row.tick;
        if ram.impact_tick.is_none() {
            // First contact is the first solver point that pushed the tank off the target: a
            // speculative point inside the contact margin that returned nothing is no touch.
            let groups: Vec<_> = self.hull.contacts.groups().collect();
            let solver = groups
                .par_splat_map(ComputeTaskPool::get(), None, |_, groups| {
                    groups
                        .iter()
                        .flat_map(|points| points.iter())
                        .filter(|point| {
                            point.reference == target
                                && point.other == tank
                                && point.impulse != Vec3::ZERO
                        })
                        .map(|point| point.closing_speed)
                        .reduce(f32::max)
                })
                .into_iter()
                .flatten();
            if let Some(normal_speed) = solver.reduce(f32::max) {
                ram.impact_tick = Some(tick);
                // The speed entering the contact step: the last row's, none on the first row.
                ram.impact = Some(json!({"tick": tick,
                    "speed_m_s": entering.map(|speeds| speeds.horizontal),
                    "normal_speed_m_s": normal_speed}));
            }
        }
        if let Some(impact_tick) = ram.impact_tick {
            let dt = sample.dt_s;
            ram.resting_s = if sample.awake == 0 {
                ram.resting_s + dt
            } else {
                0.0
            };
            if ram.resting_s >= ram.recipe.settle_s {
                ram.done = Some(End::Rest);
            } else if (tick - impact_tick) as f64 * dt >= ram.recipe.deadline_s {
                ram.done = Some(End::Deadline);
            }
        } else if tick as f64 * sample.dt_s >= ram.recipe.approach_s {
            ram.done = Some(End::ApproachTimeout);
        }
        if matches!(ram.done, Some(End::Rest | End::Deadline)) {
            let (.., root) =
                (self.constructions.get(target)).expect("the target was checked this tick");
            let (position, rotation) =
                (self.poses.get(target)).expect("a construction's standing body has a pose");
            let pieces = (self.loose.iter())
                .filter(|(piece, ..)| {
                    piece.construction == root.construction && piece.retired.is_none()
                })
                .map(|(piece, breakage, position, rotation, velocity)| {
                    let frame = breakage.physics_frame((position, rotation));
                    (piece, breakage, frame, velocity.0.length())
                });
            let built = root.physics_frame((position, rotation));
            let in_place_m = ram.recipe.in_place_m.value as f32;
            ram.structure = Some(structure::read(
                &plan.panel,
                in_place_m,
                (root, built),
                pieces,
            ));
        }
        Some(&mut row.collector_us)
    }
}

/// The recipe `collapse.py` hands the server for the shipped pass `scripts/perf/ram.json` at its
/// contact `name`: the pass's fields with that one of its `contacts` as `contact`.
#[cfg(test)]
fn shipped_recipe(name: &str) -> Recipe {
    let mut pass: Value = serde_json::from_str(include_str!("../../scripts/perf/ram.json"))
        .expect("the shipped pass is JSON");
    let fields = pass.as_object_mut().expect("a pass is an object");
    let mut contacts = fields.remove("contacts").expect("a pass states contacts");
    fields.insert("contact".into(), contacts[name].take());
    Recipe::parse(&pass.to_string()).expect("the shipped pass makes an admitted recipe")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Operator typos must refuse the run, rather than produce a different workload or reading;
    /// the in-place limit is a labelled `Unit`, never a bare number or another job.
    #[test]
    fn recipes_reject_unknown_fields_and_invalid_domains() {
        let valid = shipped_recipe("governed");
        let value = serde_json::to_value(&valid).unwrap();
        for (field, bad) in [
            ("instance", json!("")),
            ("lateral_m", json!("1")),
            ("runup_margin", json!(2.0)),
            ("contact", json!("Rest")),
            (
                "contact",
                json!({"Governed": {"minimum_fraction": 1.1, "runup_margin": 2.0}}),
            ),
            (
                "contact",
                json!({"Governed": {"minimum_fraction": 0.9, "runup_margin": 0.5}}),
            ),
            (
                "contact",
                json!({"Held": {"speed_m_s": 2.0, "within_m_s": 2.0, "runup_margin": 2.0}}),
            ),
            (
                "contact",
                json!({"Held": {"speed_m_s": 2.0, "within_m_s": 0.5}}),
            ),
            (
                "contact",
                json!({"Rest": {"gap_m": 0.0, "at_most_m_s": 0.5}}),
            ),
            ("settle_s", json!(0)),
            ("deadline_s", json!(-1)),
            ("approach_s", json!("60")),
            ("drive_s", json!(18)),
            ("typo", json!(1)),
            ("in_place_m", json!(0.1)),
            ("in_place_m", json!([0.0, {"Unit": "a scale"}])),
            ("in_place_m", json!([0.1, {"Taste": "a look"}])),
        ] {
            let mut changed = value.clone();
            changed[field] = bad;
            assert!(Recipe::parse(&changed.to_string()).is_err(), "{field}");
        }
        assert_eq!(
            Recipe::parse(&serde_json::to_string(&valid).unwrap()).unwrap(),
            valid
        );
    }

    /// Fixture: a tank-sized box's corners about a root 0.6 m over its lowest point.
    fn fixture_points(_: Quat) -> Vec<Vec3> {
        (0..8)
            .map(|k| {
                let pick = |bit, low, high| if k & bit > 0 { high } else { low };
                Vec3::new(pick(1, -1.7, 1.7), pick(2, -0.6, 2.0), pick(4, -3.5, 3.5))
            })
            .collect()
    }

    fn fixture_spec() -> TankSpec {
        ron::from_str(include_str!("../../assets/tiger_1/tiger_1.tank.ron")).unwrap()
    }

    /// Operators get an unusable route refusal before a join can reach the spawn path.
    #[test]
    fn route_admission_refuses_missing_nonconstruction_and_unreachable_instances() {
        let manifest = crate::map::tests::construction_fixture_manifest();
        let mut recipe = shipped_recipe("governed");
        let spec = fixture_spec();
        for instance in ["missing", "house_0001"] {
            // Fixture: absent and graybox targets.
            recipe.instance = instance.into();
            let error = recipe
                .plan(&spec, fixture_points, &manifest, None)
                .err()
                .expect("unusable target refused")
                .to_string();
            assert!(
                error.contains(LEVER) && error.contains("instance"),
                "{error}"
            );
        }
        recipe.instance = "brick_0001".into(); // Fixture construction from the manifest owner.
        assert!(recipe.plan(&spec, fixture_points, &manifest, None).is_ok());
        // Fixture: finite input whose derived run-up cannot fit.
        recipe.contact = Contact::Governed {
            minimum_fraction: 0.95,
            runup_margin: f64::MAX,
        };
        let error = recipe
            .plan(&spec, fixture_points, &manifest, None)
            .err()
            .expect("unreachable route refused")
            .to_string();
        assert!(error.contains("spawn bounds"), "{error}");
    }

    /// WHERE THE RUN STARTS: the hull's nearest point stands the contact's free distance from the
    /// face it meets, however deep the construction is behind that face, and the approach line
    /// lies `lateral_m` to the heading's right of the instance's origin. A pass from rest relies on
    /// the first (its gap is centimetres) and a pass at a wall junction on the second. A mutant
    /// laying the distance from the instance's origin, or the offset to the left, fails; so does a
    /// held speed the tank cannot reach passing as a slower full-throttle run.
    #[test]
    fn the_run_up_is_free_distance_to_the_met_face_on_the_offset_line() {
        let manifest = crate::map::tests::construction_fixture_manifest();
        let spec = fixture_spec();
        let mut recipe = shipped_recipe("governed");
        recipe.instance = "brick_0001".into(); // Fixture construction from the manifest owner.
        // Fixture: a gap, a heading and an offset inside the fixture box's half-width.
        let (gap_m, lateral_m) = (0.25, 0.5);
        recipe.heading_deg = 20.0;
        recipe.lateral_m = lateral_m;
        recipe.contact = Contact::Rest {
            gap_m,
            at_most_m_s: 1.0,
        };
        let turned = |turn: Quat| (fixture_points(turn).into_iter().map(|p| turn * p)).collect();
        let plan = recipe.plan(&spec, turned, &manifest, None).unwrap();
        let (instance, _) = recipe.construction(&manifest).unwrap();
        let pose = Transform::from_isometry(instance.pose.at_height(0.0));
        let direction = recipe.rotation() * Vec3::NEG_Z;
        let spawn = Vec3::from_array(plan.spawn);
        let panel = serde_json::to_value(&plan.panel).unwrap();
        let at: Vec3 = serde_json::from_value(panel["at"].clone()).unwrap();
        let nearest = (fixture_points(Quat::IDENTITY).iter())
            .map(|&point| (spawn + recipe.rotation() * point).dot(direction))
            .fold(f32::NEG_INFINITY, f32::max);
        let free = pose.transform_point(at).dot(direction) - nearest;
        assert!((free - gap_m as f32).abs() < 1e-3, "{free}");
        assert_eq!(plan.runup_m, gap_m as f32);
        let right = (spawn - pose.translation).dot(recipe.rotation() * Vec3::X);
        assert!((right - lateral_m as f32).abs() < 1e-4, "{right}");
        recipe.contact = Contact::Held {
            speed_m_s: f64::from(plan.governed_m_s),
            within_m_s: 0.5,
            runup_margin: 1.0,
        };
        let error = recipe.plan(&spec, turned, &manifest, None).err();
        let error = error
            .expect("an unreachable held speed is refused")
            .to_string();
        assert!(error.contains("Held.speed_m_s"), "{error}");
    }

    /// The production installer checks and retains the route without a client, while sampling
    /// waits for spawn. Starting capture at admission would publish a zero-tank prefix to the runner.
    #[test]
    fn startup_retains_the_plan_without_starting_capture() {
        let mut app = App::new();
        let mut recipe = shipped_recipe("governed");
        recipe.instance = "brick_0001".into(); // Fixture construction in the manifest below.
        // Route admission reads the tank's construction for its spawn stance; no rig is spawned.
        let (blueprint, template) = crate::tank::shipped_construction();
        app.insert_resource(crate::map::tests::construction_fixture_manifest())
            .insert_resource(blueprint)
            .insert_resource(template)
            .init_resource::<ContactTransfers>()
            .init_resource::<BoundAccount>();
        install_recipe(&mut app, recipe);
        app.world_mut().run_schedule(PostStartup);
        let ram = app.world().resource::<Ram>();
        assert!(ram.plan.is_some());
        assert!(!ram.started);
        assert!(!record(app.world_mut()));
        let mut ram = app.world_mut().resource_mut::<Ram>();
        let planned = ram.plan.as_ref().unwrap().spawn;
        assert_eq!(ram.pose().0.to_array(), planned);
        assert!(ram.started);
    }

    /// A Battle reset must invalidate an armed observation, rather than certify a quiet new
    /// world as the old construction resting. The runner relies on that refusal.
    #[test]
    fn removed_target_refuses_the_capture() {
        let mut world = World::new();
        let target = world.spawn_empty().id();
        world.despawn(target);
        world.init_resource::<ContactTransfers>();
        world.init_resource::<BoundAccount>();
        let mut ram = armed();
        ram.target = Some(target);
        world.insert_resource(ram);
        assert!(!record(&mut world));
        assert_eq!(world.resource::<Ram>().done, Some(End::WorldChanged));
    }

    /// Runs the census once on an idle tick (fixture: one-second step); whether it captured.
    fn record(world: &mut World) -> bool {
        use bevy::ecs::system::RunSystemOnce;
        ComputeTaskPool::get_or_init(bevy::tasks::TaskPool::default);
        world.init_resource::<avian3d::dynamics::solver::SolverDiagnostics>();
        world.init_resource::<crate::avian_ext::solver_cost::ContactVisits>();
        world.init_resource::<avian3d::collision::CollisionDiagnostics>();
        world.init_resource::<avian3d::collider_tree::ColliderTreeDiagnostics>();
        world.init_resource::<avian3d::prelude::ContactGraph>();
        world.init_resource::<avian3d::dynamics::solver::constraint_graph::ConstraintGraph>();
        world.init_resource::<crate::avian_ext::contact_reduction::ReductionCounts>();
        world.init_resource::<avian3d::prelude::PhysicsLengthUnit>();
        world.init_resource::<avian3d::prelude::TimeToSleep>();
        world.init_resource::<StepSnapshot>();
        world
            .run_system_once(|mut census: Census| {
                let cpu = NamedTimes::new(&stages::NAMES, [0.0; stages::COUNT]);
                let sample = census.sample(
                    0.0,
                    0.0,
                    0.0,
                    ([0.0; stages::COUNT], cpu),
                    &phases::Phases::default(),
                    1.0,
                );
                census
                    .record(
                        sample,
                        crate::cost::stages::Stages::default().take_counts(),
                        ContactTick::default(),
                    )
                    .is_some()
            })
            .unwrap()
    }

    /// A started capture on the shipped recipe; tests set any timing they exercise.
    fn armed() -> Ram {
        let mut ram = Ram::new(shipped_recipe("governed"));
        ram.plan = Some(Plan {
            spawn: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            runup_m: 1.0,
            governed_m_s: 1.0,
            admitted_m_s: (1.0, None),
            clear_m: 1.0,
            panel: structure::tests::column_panel(),
        });
        ram.started = true;
        ram
    }

    /// A tank at rest at the origin, moving at `velocity`.
    fn spawn_tank(world: &mut World, velocity: Vec3) -> Entity {
        world
            .spawn((
                Tank,
                Position::default(),
                Rotation::default(),
                LinearVelocity(velocity),
            ))
            .id()
    }

    /// Fixture: one solver point between `reference` and `other` that closed at `closing` m/s
    /// and returned `impulse` N s along +Z.
    fn touch(
        reference: Entity,
        other: Entity,
        closing: f32,
        impulse: f32,
    ) -> crate::contact_transfer::ContactTransfer {
        crate::contact_transfer::ContactTransfer {
            reference,
            reference_collider: reference,
            other,
            other_collider: other,
            point: Vec3::ZERO,
            normal: Vec3::Z,
            impulse: Vec3::Z * impulse,
            effective_mass: 1.0,
            closing_speed: closing,
            friction: 0.0,
        }
    }

    /// Fixture: the armed recipe's target as construction 3, beside the resources a row reads.
    fn spawn_target(world: &mut World) -> Entity {
        world.init_resource::<BoundAccount>();
        let instance = world.resource::<Ram>().recipe.instance.clone();
        world
            .spawn((
                Body,
                MapInstance {
                    id: instance,
                    ordinal: 0,
                },
                Breakage::new(crate::construction::tests::column_construction(), 3),
            ))
            .id()
    }

    /// The admission gate reads the speed entering first contact, the last row's: the tank's
    /// first solver point on the target that returned an impulse sets the impact tick, at that
    /// point's closing speed. Another body's touch, the tank's touch on another construction and
    /// a speculative point that returned nothing are not the ram's impact. Mutants: the contact
    /// row's own speed (what the contact left), any point in the contact margin.
    #[test]
    fn the_tanks_first_impulse_on_the_target_sets_the_impact_tick() {
        let mut world = World::new();
        world.insert_resource(armed());
        let tank = spawn_tank(&mut world, Vec3::NEG_Z * 10.0); // Fixture: the run-up's speed.
        let target = spawn_target(&mut world);
        let [other, elsewhere] = [(); 2].map(|()| world.spawn_empty().id());
        // Fixture: closing speeds, m/s, of which none is the tank's impulse on the target.
        world.insert_resource(ContactTransfers::fixture(
            vec![vec![
                touch(target, tank, 13.0, 0.0),
                touch(target, other, 12.0, 1.0),
                touch(elsewhere, tank, 11.0, 1.0),
            ]],
            1.0,
        ));
        assert!(record(&mut world));
        assert_eq!(world.resource::<Ram>().impact_tick, None);
        // Fixture: the contact step leaves 7 m/s.
        world.get_mut::<LinearVelocity>(tank).unwrap().0 = Vec3::NEG_Z * 7.0;
        world.insert_resource(ContactTransfers::fixture(
            vec![vec![touch(target, tank, 9.5, 1.0)]],
            1.0,
        ));
        assert!(record(&mut world));
        let ram = world.resource::<Ram>();
        assert_eq!(ram.impact_tick, Some(2));
        assert_eq!(
            ram.impact,
            Some(json!({"tick": 2, "speed_m_s": 10.0, "normal_speed_m_s": 9.5}))
        );
    }

    /// The runner must receive a named refusal at the approach limit. Contact on the limit
    /// still opens the observation; a mutant that waits or times out after impact fails here.
    #[test]
    fn approach_watchdog_ends_only_an_uncontacted_capture() {
        for contact_at_limit in [false, true] {
            let mut world = World::new();
            world.init_resource::<ContactTransfers>();
            let mut ram = armed();
            ram.recipe.approach_s = 2.0; // Fixture: two one-second census ticks.
            world.insert_resource(ram);
            let tank = spawn_tank(&mut world, Vec3::ZERO);
            let target = spawn_target(&mut world);
            assert!(record(&mut world));
            assert_eq!(world.resource::<Ram>().done, None);
            if contact_at_limit {
                // Fixture: first touch arrives exactly at the approach limit.
                world.insert_resource(ContactTransfers::fixture(
                    vec![vec![touch(target, tank, 1.0, 1.0)]],
                    1.0,
                ));
            }
            assert!(record(&mut world));
            if contact_at_limit {
                assert_eq!(world.resource::<Ram>().impact_tick, Some(2));
                assert_ne!(world.resource::<Ram>().done, Some(End::ApproachTimeout));
            } else {
                assert_eq!(
                    world.resource_mut::<Ram>().json()["done"],
                    "approach_timeout"
                );
                assert!(
                    !record(&mut world),
                    "a refused capture records no coasting ticks"
                );
            }
        }
    }

    /// A second vehicle changes the run-up through its data, without a vehicle-specific route:
    /// twice the mass at one drive is half the ideal acceleration, so twice the distance to any
    /// speed, at the same governed speed.
    #[test]
    fn the_ideal_acceleration_follows_the_vehicles_mass() {
        let mut spec = fixture_spec();
        let (speed, acceleration) = top_gear(&spec);
        spec.mass *= 2.0; // Fixture: otherwise identical heavier vehicle.
        assert_eq!(top_gear(&spec), (speed, acceleration / 2.0));
    }
}
