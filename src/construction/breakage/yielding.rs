//! AUTHORITY: loose pieces of the game's building model meeting bodies (game state only). A piece
//! meets everything as an ordinary body: it is pushed, stacks, bears a hull and falls on things.
//! Its contacts are a spring in place of Avian's rigid stop: each point of a contact with a piece
//! on either side takes the stiffest critically damped spring the solver resolves at that point's
//! effective mass ([`piece_spring`]), so a piece squeezed in a chain passes on a force that rises
//! with its overlap over substeps, and the overlap closes as room opens, without a bounce.
//!
//! Masonry is far stiffer than this (a cell's modulus over its size is of the order of 1e9 N/m),
//! so the spring's stiffness states nothing of the material: it is the solver's resolution.
//!
//! A belt's run on a piece keeps its links' law (`track::soft_run`), which states that contact
//! whole. A standing cell's bound on a piece (`bounded`) clamps the same point's spring.

use avian3d::collision::contact_types::ContactGraph;
use avian3d::dynamics::solver::schedule::SolverSystems;
use avian3d::prelude::{PhysicsSchedule, PhysicsStepSystems, RigidBody, Substeps};

use super::*;
use crate::avian_ext::compliance::{Laws, Tuned};

/// Substeps one period of a piece's contact spring spans. `Resolution`: the soft constraint is an
/// implicit step, stable at any frequency (a free overlap shrinks by `1 / (1 + 2ζωh + ω²h²)` a
/// substep `h`), but a spring sampled under four times a period answers as a damper whatever its
/// stiffness; four is also the bound Avian holds its own contact spring to
/// (`update_contact_softness`, `0.25 / h`).
const SUBSTEPS_PER_PERIOD: f32 = 4.0;

/// The spring of a loose piece's contact points at substep `h` s: [`SUBSTEPS_PER_PERIOD`] substeps
/// a period, critically damped. DERIVED at the shipped 64 Hz and 6 substeps: 96 Hz, so a 300 kg
/// piece's point is 1.1e8 N/m at most (`m (2π 96)²`) and gives 9 mm under 1 MN.
fn piece_spring(h: f32) -> Tuned {
    Tuned {
        hertz: (SUBSTEPS_PER_PERIOD * h).recip(),
        damping_ratio: 1.0,
    }
}

/// Inside the physics step, before Avian prepares the step's contact constraints: every manifold
/// of an awake pair with a loose piece on either side takes the piece's spring.
fn soften_pieces(
    contacts: Res<ContactGraph>,
    bodies: Query<&RigidBody, With<Piece>>,
    substep: Res<Time<Substeps>>,
    mut laws: ResMut<Laws>,
    standing: Query<(), (With<Breakage>, Without<Piece>)>,
    dynamic: Query<&RigidBody, Without<Piece>>,
) {
    // TEMP DIAG: which contacts of a piece are soft: p piece, h other dynamic, s standing, g ground.
    let scope = std::env::var("OVERMATCH_TMP_PIECE_SCOPE").unwrap_or("phsg".into());
    let mut spring = piece_spring(substep.delta_secs());
    // TEMP DIAG
    if let Some(hz) = std::env::var("OVERMATCH_TMP_PIECE_HZ")
        .ok()
        .and_then(|v| v.parse::<f32>().ok())
    {
        if hz <= 0.0 {
            return;
        }
        spring.hertz = hz;
    }
    if let Some(z) = std::env::var("OVERMATCH_TMP_PIECE_ZETA")
        .ok()
        .and_then(|v| v.parse::<f32>().ok())
    {
        spring.damping_ratio = z;
    }
    let loose = |body: Option<Entity>| {
        body.is_some_and(|body| bodies.get(body).is_ok_and(RigidBody::is_dynamic))
    };
    for pair in contacts.iter_active() {
        let (a, b) = (loose(pair.body1), loose(pair.body2));
        let other = if a { pair.body2 } else { pair.body1 };
        let kind = if a && b {
            'p'
        } else if other.is_some_and(|o| standing.contains(o)) {
            's'
        } else if other.is_some_and(|o| dynamic.get(o).is_ok_and(RigidBody::is_dynamic)) {
            'h'
        } else {
            'g'
        };
        if (a || b) && scope.contains(kind) {
            for manifold in 0..pair.manifolds.len() {
                laws.tune(pair.contact_id, manifold, spring);
            }
        }
    }
}

pub(super) fn plugin(app: &mut App) {
    app.add_systems(
        PhysicsSchedule,
        soften_pieces
            .in_set(PhysicsStepSystems::Solver)
            .after(SolverSystems::PrepareSolverBodies)
            .after(crate::track::soft_run::RunLaws)
            .before(SolverSystems::PrepareContactConstraints)
            .run_if(not(resource_exists::<ClientReplica>)),
    );
    // TEMP DIAG
    if std::env::var("OVERMATCH_TMP_DIAG").is_ok() {
        app.add_systems(
            FixedPostUpdate,
            diag.after(avian3d::prelude::PhysicsSystems::StepSimulation)
                .run_if(not(resource_exists::<ClientReplica>)),
        );
    }
}

// TEMP DIAG (not committed)
fn diag(
    contacts: Res<ContactGraph>,
    pieces: Query<
        (
            Entity,
            &Position,
            &avian3d::prelude::LinearVelocity,
            &avian3d::prelude::AngularVelocity,
            Has<avian3d::prelude::Sleeping>,
        ),
        With<Piece>,
    >,
    hulls: Query<
        (
            &Position,
            &avian3d::prelude::LinearVelocity,
            &avian3d::prelude::ComputedMass,
            &RigidBody,
        ),
        Without<Piece>,
    >,
    mut tick: Local<u32>,
    spatial: avian3d::prelude::SpatialQuery,
    mut seen: Local<bevy::platform::collections::HashMap<Entity, (f32, bool)>>,
    info_of: Query<&Piece>,
) {
    *tick += 1;
    for (e, at, v, _, _) in &pieces {
        let filter = avian3d::prelude::SpatialQueryFilter::from_mask(Layer::Terrain);
        let entry = seen.entry(e).or_insert_with(|| {
            let ground = spatial
                .cast_ray(at.0 + Vec3::Y * 60.0, Dir3::NEG_Y, 200.0, true, &filter)
                .map_or(f32::NAN, |hit| at.0.y + 60.0 - hit.distance);
            let cells = info_of.get(e).map_or(0, |p| p.chunks.len());
            info!(
                "BORN tick={} piece={:?} cells={} y={} ground={} vy={}",
                *tick, e, cells, at.0.y, ground, v.0.y
            );
            (ground, false)
        });
        if !entry.1 && at.0.y < entry.0 - 1.5 {
            entry.1 = true;
            info!(
                "FELL tick={} piece={:?} y={} ground={} v={:?}",
                *tick, e, at.0.y, entry.0, v.0
            );
        }
    }
    if *tick % 4 != 0 {
        return;
    }
    // deepest overlap by kind (0 piece-piece, 1 piece-fixed, 2 piece-driven) and counts over 2 cm and 5 cm
    let mut deep = [0.0_f32; 3];
    let mut over2 = [0u32; 3];
    let mut over5 = [0u32; 3];
    let mut pairs = [0u32; 3];
    for pair in contacts
        .active_pairs()
        .iter()
        .chain(contacts.sleeping_pairs())
    {
        let (Some(a), Some(b)) = (pair.body1, pair.body2) else {
            continue;
        };
        let kind = |e: Entity| {
            if pieces.contains(e) {
                0
            } else if hulls.get(e).is_ok_and(|h| h.3.is_dynamic()) {
                2
            } else {
                1
            }
        };
        let (ka, kb) = (kind(a), kind(b));
        if ka != 0 && kb != 0 {
            continue;
        }
        let k = ka.max(kb);
        let pen = pair
            .manifolds
            .iter()
            .flat_map(|m| &m.points)
            .map(|p| p.penetration)
            .fold(f32::NEG_INFINITY, f32::max);
        if pen <= 0.0 {
            continue;
        }
        pairs[k] += 1;
        deep[k] = deep[k].max(pen);
        if pen > 0.02 {
            over2[k] += 1;
        }
        if pen > 0.05 {
            over5[k] += 1;
        }
    }
    let (mut n, mut awake, mut fast, mut low, mut spin) = (0, 0, 0.0_f32, f32::INFINITY, 0.0_f32);
    for (_, at, v, w, asleep) in &pieces {
        n += 1;
        if !asleep {
            awake += 1;
            fast = fast.max(v.0.length());
            spin = spin.max(w.0.length());
        }
        low = low.min(at.0.y);
    }
    for (at, v, mass, body) in &hulls {
        if body.is_dynamic() && mass.value() > 10_000.0 {
            info!(
                "WHERE tick={} x={} y={} z={} vx={} vz={} vy={}",
                *tick, at.0.x, at.0.y, at.0.z, v.0.x, v.0.z, v.0.y
            );
        }
    }
    info!(
        "SOFT tick={} pieces={} awake={} fastest={} spin={} lowest_y={} deep={:?} over2={:?} over5={:?} pairs={:?}",
        *tick, n, awake, fast, spin, low, deep, over2, over5, pairs
    );
}
