//! The ram instrument's relay reading, for the game's building model (game state only): what
//! passes a hull's push through loose pieces into standing matter. `OVERMATCH_RELAY` on the
//! server, a comma list, read once at install; its user is the relay measurement of
//! `lane/bounded`, which deletes it when it reports.
//!
//! * always: `RELAY born`, each new piece (its centre against the height grid under it, its
//!   volume), `RELAY birth`, its deepest overlap with a driven body after its first step, `RELAY
//!   sunk`, a piece whose centre first lies under the grid, `RELAY freed`, the cells the bound law
//!   freed and what pressed them, `RELAY held`, the blocks no way was open for, `RELAY hull` every
//!   fourth tick, `RELAY rest` every second (the deepest overlap by pair kind), and `RELAY site`
//!   once (each construction's chunks against the grid, and the cells the site unburied that no
//!   ground joint holds).
//! * `verdicts`: pressed pieces are judged read-only (`debris::crumble::PressVerdicts`).
//! * `keep`: a cell's crossed give is never forgotten (`bounded::KeepGive`).
//! * `bare`: a joint the push closes resists a sideways shed with nothing (`bounded::BareShed`).
//! * `pushout=<m/s>`: Avian's `max_overlap_solve_speed`.

use avian3d::dynamics::solver::SolverConfig;
use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;

use crate::construction::breakage::bounded::{BareShed, BoundAccount, KeepGive};
use crate::construction::breakage::{Breakage, Piece};
use crate::construction::fracture::Side;
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
            _ if word == "bare" => {
                app.insert_resource(BareShed);
            }
            Some(("pushout", speed)) => {
                let speed: f32 = speed
                    .parse()
                    .unwrap_or_else(|_| panic!("{LEVER}: pushout={speed} is not a speed in m/s"));
                app.world_mut()
                    .resource_mut::<SolverConfig>()
                    .max_overlap_solve_speed = speed;
            }
            _ => panic!(
                "{LEVER}: unknown word {word:?}; use log, verdicts, keep, bare, pushout=<m/s>"
            ),
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
    grid: Option<Res<crate::terrain_grid::HeightGrid>>,
    standing: Query<(&Breakage, &Position, &Rotation), Without<Piece>>,
    mut seen: Local<HashMap<Entity, u32>>,
    mut sunk: Local<bevy::platform::collections::HashSet<Entity>>,
    mut tick: Local<u32>,
) {
    *tick += 1;
    let tick = *tick;
    let hull = driven
        .iter()
        .find(|(body, _, _, mass)| body.is_dynamic() && mass.value() > 10_000.0);
    let speed = hull.map_or(0.0, |(_, _, velocity, _)| velocity.0.length());
    if tick.is_multiple_of(4)
        && let Some((_, at, velocity, _)) = hull
    {
        info!(
            "RELAY hull tick={tick} at={:?} v={:?} pieces={}",
            at.0,
            velocity.0,
            pieces.iter().len()
        );
    }
    // The height grid is the ground: a ray on `Layer::Terrain` meets standing elements first.
    let ground = |at: Vec3| (grid.as_deref()).map_or(f32::NAN, |grid| grid.height_at(at.x, at.z));
    if tick == 1 {
        for (owner, position, rotation) in &standing {
            let pose = owner.physics_frame((position, rotation));
            let chunks = &owner.live.structure.chunks;
            let under = |c: usize| {
                (owner.kind.layout.blocks[c].points.iter()).all(|&point| {
                    let world = pose.transform_point(point);
                    world.y <= ground(world)
                })
            };
            let unburied_under = (0..chunks.len())
                .filter(|&c| !chunks[c].buried && under(c))
                .count();
            let buried_above = (0..chunks.len())
                .filter(|&c| chunks[c].buried && !under(c))
                .count();
            // Cells the kind buries on level ground and this site does not, and those of them no
            // ground joint holds.
            let joints = &owner.live.structure.joints;
            let grounded = |c: usize| {
                (joints.iter()).any(|joint| joint.a == c && matches!(joint.b, Side::Ground))
            };
            let lost: Vec<usize> = (0..chunks.len())
                .filter(|&c| owner.kind.structure.chunks[c].buried && !chunks[c].buried)
                .collect();
            let gained = (0..chunks.len())
                .filter(|&c| !owner.kind.structure.chunks[c].buried && chunks[c].buried)
                .count();
            let unheld: Vec<usize> = lost.iter().copied().filter(|&c| !grounded(c)).collect();
            // A cell is carried from below when a joint ties it to a cell whose centre is lower.
            let lower = |c: usize| {
                joints.iter().any(|joint| match joint.b {
                    Side::Chunk(b) if joint.a == c => chunks[b].centroid.y < chunks[c].centroid.y,
                    Side::Chunk(b) if b == c => chunks[joint.a].centroid.y < chunks[c].centroid.y,
                    _ => false,
                })
            };
            let base_under = |c: usize| {
                (owner.kind.layout.blocks[c].points.iter()).any(|&point| {
                    let world = pose.transform_point(point);
                    world.y <= ground(world)
                })
            };
            let none: Vec<usize> = (0..chunks.len())
                .filter(|&c| !chunks[c].buried && !grounded(c) && !lower(c))
                .collect();
            let air: Vec<f32> = (joints.iter())
                .filter(|joint| matches!(joint.b, Side::Ground) && !chunks[joint.a].buried)
                .map(|joint| {
                    let world = pose.transform_point(joint.centroid);
                    world.y - ground(world)
                })
                .filter(|&gap| gap > 0.0)
                .collect();
            let loose = crate::construction::fracture::loose_groups(
                &owner.live.structure,
                &owner.live.state,
            );
            if unburied_under + buried_above + lost.len() + gained + none.len() + loose.len() > 0 {
                info!(
                    "RELAY site con={} chunks={} buried={} level_buried={} lost_buried={} lost_and_no_ground_joint={} of_those_on_a_lower_cell={} of_those_base_under_ground={} gained_buried={gained} ground_joints={} anchors_over_ground={} highest_anchor_over_ground={} none_of_three={} none_sample={:?} loose_groups={} unburied_under_ground={unburied_under} buried_above_ground={buried_above} root_over_ground={}",
                    owner.construction,
                    chunks.len(),
                    chunks.iter().filter(|chunk| chunk.buried).count(),
                    owner
                        .kind
                        .structure
                        .chunks
                        .iter()
                        .filter(|chunk| chunk.buried)
                        .count(),
                    lost.len(),
                    unheld.len(),
                    unheld.iter().filter(|&&c| lower(c)).count(),
                    unheld.iter().filter(|&&c| base_under(c)).count(),
                    joints
                        .iter()
                        .filter(|joint| matches!(joint.b, Side::Ground))
                        .count(),
                    air.len(),
                    air.iter().copied().fold(0.0, f32::max),
                    none.len(),
                    &none[..none.len().min(6)],
                    loose.len(),
                    pose.translation.y - ground(pose.translation),
                );
            }
        }
    }
    for (entity, piece, breakage, at, rotation, centre, velocity) in &pieces {
        let age = seen.entry(entity).or_insert(0);
        *age += 1;
        let middle = at.0 + rotation.0 * centre.0;
        if *age > 1 && middle.y < ground(middle) - 0.3 && sunk.insert(entity) {
            info!(
                "RELAY sunk tick={tick} piece={entity:?} age={} cells={} under_m={} v={:?} hull_at={:?}",
                *age,
                piece.chunks.len(),
                ground(middle) - middle.y,
                velocity.0,
                hull.map(|(_, at, ..)| at.0 - middle),
            );
        }
        // A second after birth, with its mass properties computed: where it is now.
        if *age == 64 {
            let centre = at.0 + rotation.0 * centre.0;
            let ground = ground(centre);
            info!(
                "RELAY later tick={tick} piece={entity:?} over_ground={} vy={}",
                centre.y - ground,
                velocity.0.y
            );
        }
        if *age == 1 {
            let centre = at.0 + rotation.0 * centre.0;
            let ground = ground(centre);
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
    if tick.is_multiple_of(64) {
        // Deepest overlap by pair kind: piece on piece, on standing matter, on the ground, on a
        // driven body; and a driven body on the ground.
        let kind = |body: Entity| {
            if pieces.contains(body) {
                0
            } else if standing.contains(body) {
                1
            } else if driven.get(body).is_ok_and(|(body, ..)| body.is_dynamic()) {
                3
            } else {
                2
            }
        };
        let mut deep = [0.0_f32; 5];
        for pair in contacts.iter_active() {
            let (Some(a), Some(b)) = (pair.body1, pair.body2) else {
                continue;
            };
            let (a, b) = (kind(a).min(kind(b)), kind(a).max(kind(b)));
            let slot = match (a, b) {
                (0, 0) => 0,
                (0, 1) => 1,
                (0, 2) => 2,
                (0, 3) => 3,
                (2, 3) => 4,
                _ => continue,
            };
            let most = (pair.manifolds.iter())
                .flat_map(|manifold| &manifold.points)
                .map(|point| point.penetration)
                .fold(0.0, f32::max);
            deep[slot] = deep[slot].max(most);
        }
        let speeds: Vec<f32> = pieces.iter().map(|piece| piece.6.0.length()).collect();
        info!(
            "RELAY rest tick={tick} pieces={} moving={} fastest={} deep_piece_piece={} deep_piece_standing={} deep_piece_ground={} deep_piece_driven={} deep_driven_ground={} under_ground={}",
            speeds.len(),
            speeds.iter().filter(|&&speed| speed > 0.05).count(),
            speeds.iter().copied().fold(0.0, f32::max),
            deep[0],
            deep[1],
            deep[2],
            deep[3],
            deep[4],
            sunk.len(),
        );
    }
    for held in &account.held {
        if tick.is_multiple_of(4) {
            info!(
                "RELAY held tick={tick} body={:?} piece={} cells={} why={:?}",
                held.body,
                pieces.contains(held.body),
                held.cells,
                held.why
            );
        }
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
