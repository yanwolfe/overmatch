//! The ram instrument's relay reading, for the game's building model (game state only): what
//! passes a hull's push through loose pieces into standing matter. `OVERMATCH_RELAY` on the
//! server, a comma list, read once at install; its user is the relay measurement of
//! `lane/bounded`, which deletes it when it reports.
//!
//! * always: `RELAY born`, each new piece (its centre against the ground under it, its volume),
//!   `RELAY birth`, its deepest overlap with a driven body after its first step, `RELAY freed`,
//!   the cells the bound law freed and what pressed them, and `RELAY hull` every fourth tick.
//! * `verdicts`: pressed pieces are judged read-only (`debris::crumble::PressVerdicts`).
//! * `keep`: a cell's crossed give is never forgotten (`bounded::KeepGive`).
//! * `pushout=<m/s>`: Avian's `max_overlap_solve_speed`.

use avian3d::dynamics::solver::SolverConfig;
use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;

use crate::Layer;
use crate::construction::breakage::bounded::{BoundAccount, KeepGive};
use crate::construction::breakage::{Breakage, Piece};
use crate::debris::crumble::PressVerdicts;

const LEVER: &str = "OVERMATCH_RELAY";

pub(super) fn install(app: &mut App) {
    let Ok(text) = std::env::var(LEVER) else {
        return;
    };
    for word in text.split(',') {
        match word.split_once('=') {
            _ if word == "log" => {}
            _ if word == "verdicts" => {
                app.insert_resource(PressVerdicts);
            }
            _ if word == "keep" => {
                app.insert_resource(KeepGive);
            }
            Some(("pushout", speed)) => {
                let speed: f32 = speed
                    .parse()
                    .unwrap_or_else(|_| panic!("{LEVER}: pushout={speed} is not a speed in m/s"));
                app.world_mut()
                    .resource_mut::<SolverConfig>()
                    .max_overlap_solve_speed = speed;
            }
            _ => panic!("{LEVER}: unknown word {word:?}; use log, verdicts, keep, pushout=<m/s>"),
        }
    }
    app.add_systems(
        FixedPostUpdate,
        read.after(PhysicsSystems::StepSimulation)
            .after(crate::construction::breakage::LawSet),
    );
}

type Pieces<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static Piece,
        &'static Breakage,
        &'static Position,
        &'static Rotation,
        &'static ComputedCenterOfMass,
        &'static LinearVelocity,
    ),
>;

fn read(
    contacts: Res<ContactGraph>,
    pieces: Pieces,
    driven: Query<(&RigidBody, &Position, &LinearVelocity, &ComputedMass), Without<Piece>>,
    account: Res<BoundAccount>,
    spatial: SpatialQuery,
    mut seen: Local<HashMap<Entity, u32>>,
    mut tick: Local<u32>,
) {
    *tick += 1;
    let tick = *tick;
    let hull = driven
        .iter()
        .find(|(body, _, _, mass)| body.is_dynamic() && mass.value() > 10_000.0);
    let speed = hull.map_or(0.0, |(_, _, velocity, _)| velocity.0.length());
    if tick % 4 == 0
        && let Some((_, at, velocity, _)) = hull
    {
        info!(
            "RELAY hull tick={tick} at={:?} v={:?} pieces={}",
            at.0,
            velocity.0,
            pieces.iter().len()
        );
    }
    for (entity, piece, breakage, at, rotation, centre, velocity) in &pieces {
        let age = seen.entry(entity).or_insert(0);
        *age += 1;
        // A second after birth, with its mass properties computed: where it is now.
        if *age == 64 {
            let centre = at.0 + rotation.0 * centre.0;
            let filter = SpatialQueryFilter::from_mask(Layer::Terrain);
            let ground = spatial
                .cast_ray(centre + Vec3::Y * 60.0, Dir3::NEG_Y, 200.0, true, &filter)
                .map_or(f32::NAN, |hit| centre.y + 60.0 - hit.distance);
            info!(
                "RELAY later tick={tick} piece={entity:?} over_ground={} vy={}",
                centre.y - ground,
                velocity.0.y
            );
        }
        if *age == 1 {
            let centre = at.0 + rotation.0 * centre.0;
            let filter = SpatialQueryFilter::from_mask(Layer::Terrain);
            let ground = spatial
                .cast_ray(centre + Vec3::Y * 60.0, Dir3::NEG_Y, 200.0, true, &filter)
                .map_or(f32::NAN, |hit| centre.y + 60.0 - hit.distance);
            let structure = &breakage.live.structure;
            let volume: f32 = (piece.chunks.iter())
                .map(|&c| structure.chunks[usize::from(c)].volume)
                .sum();
            let buried = (piece.chunks.iter())
                .filter(|&&c| structure.chunks[usize::from(c)].buried)
                .count();
            info!(
                "RELAY born tick={tick} piece={entity:?} con={} cells={} buried={buried} vol={volume} over_ground={} v={} hull_v={speed} chunks={:?}",
                piece.construction,
                piece.chunks.len(),
                centre.y - ground,
                velocity.0.length(),
                piece.chunks,
            );
        }
    }
    // A piece's first contacts are its second sighting's: it was spawned after its first step.
    let mut deepest: HashMap<Entity, f32> = HashMap::default();
    for pair in contacts.iter_active() {
        let (Some(a), Some(b)) = (pair.body1, pair.body2) else {
            continue;
        };
        for (piece, other) in [(a, b), (b, a)] {
            if seen.get(&piece) == Some(&2)
                && driven.get(other).is_ok_and(|(body, ..)| body.is_dynamic())
            {
                let deep = (pair.manifolds.iter())
                    .flat_map(|manifold| &manifold.points)
                    .map(|point| point.penetration)
                    .fold(0.0, f32::max);
                let most = deepest.entry(piece).or_insert(0.0);
                *most = most.max(deep);
            }
        }
    }
    for (piece, deep) in deepest {
        info!("RELAY birth tick={tick} piece={piece:?} in_hull_m={deep} hull_v={speed}");
    }
    if !account.freed.is_empty() {
        let rows: Vec<String> = (account.blocks.iter())
            .filter(|row| row.spent >= 1.0)
            .map(|row| {
                let (kind, moving) = match pieces.get(row.body) {
                    Ok(piece) => ("piece", piece.6.0.length()),
                    Err(_) => (
                        "hull",
                        driven.get(row.body).map_or(0.0, |body| body.2.0.length()),
                    ),
                };
                format!("{kind}:{moving}:{}:{:?}", row.cells, row.way)
            })
            .collect();
        info!(
            "RELAY freed tick={tick} cells={} belts={} by=[{}]",
            account.freed.len(),
            account.capped_belts.len(),
            rows.join(",")
        );
    }
}
