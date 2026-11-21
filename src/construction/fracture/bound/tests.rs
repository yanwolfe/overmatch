use std::sync::Arc;

use bevy::math::{DVec3, Vec3};

use super::*;
use crate::construction::ConstructionKind;
use crate::construction::fracture::parts::Gravity;
use crate::construction::tests::wall_construction;

/// Fixture: a brick wall fixture standing intact under its own weight.
struct Wall {
    kind: Arc<ConstructionKind>,
    state: State,
    preload: Vec<f64>,
    faces: Faces,
}

impl Wall {
    fn new(size: Vec3, cells: [usize; 3]) -> Self {
        // The gravity solve runs its parts on the compute pool.
        bevy::tasks::ComputeTaskPool::get_or_init(bevy::tasks::TaskPool::default);
        let kind = wall_construction(size, cells, "Brick");
        let state = State::new(&kind.structure);
        let preload = Gravity::default()
            .get(&kind.structure, &state, &kind.equilibrium)
            .to_vec();
        let faces = Faces::new(&kind.layout.blocks, &kind.structure);
        Self {
            kind,
            state,
            preload,
            faces,
        }
    }

    fn standing(&self) -> Standing<'_> {
        Standing {
            structure: &self.kind.structure,
            state: &self.state,
            shapes: &self.kind.layout.blocks,
            faces: &self.faces,
            preload: &self.preload,
        }
    }

    /// The cells whose centroids lie inside the box `lo..hi`, ordered.
    fn within(&self, lo: Vec3, hi: Vec3) -> Vec<usize> {
        (0..self.kind.structure.chunks.len())
            .filter(|&c| {
                let at = self.kind.structure.chunks[c].centroid;
                at.cmpge(lo).all() && at.cmple(hi).all()
            })
            .collect()
    }
}

/// Fixture: a wall 4 m long and 2.4 m high, and a patch of its face 1 m wide and 0.8 m high.
const WALL: Vec3 = Vec3::new(4.0, 2.4, 0.25);
const PATCH: (Vec3, Vec3) = (Vec3::new(-0.5, 0.8, -1.0), Vec3::new(0.5, 1.6, 1.0));
const THROUGH: DVec3 = DVec3::Z;

fn close(a: f64, b: f64, share: f64) -> bool {
    (a - b).abs() <= share * a.abs().max(b.abs())
}

/// Invariant: the price of pushing a patch of wall out is its boundary's, whatever the cut
/// inside it, so a wall is no stronger the finer it is baked. Relied on by every body's bound
/// (`breakage::bounded`). Every term is a sum over the same boundary: areas, and the gravity
/// preload those areas carry. Mutant: each
/// pressed cell's own joints summed (the fine cut prices several times the coarse).
#[test]
fn joints_between_cells_pressed_together_carry_nothing() {
    let prices: Vec<Price> = [[16, 12, 1], [32, 24, 1]]
        .into_iter()
        .map(|cells| {
            let wall = Wall::new(WALL, cells);
            let pressed = wall.within(PATCH.0, PATCH.1);
            assert_eq!(pressed.len(), cells[0] / 4 * (cells[1] / 3));
            let standing = wall.standing();
            let plug = standing.plug(&pressed, THROUGH);
            assert_eq!(plug.cells, pressed, "a single leaf has nothing behind");
            standing.shear_out(&plug, THROUGH).unwrap()
        })
        .collect();
    assert!(
        close(prices[0].force, prices[1].force, 1e-6)
            && close(prices[0].work, prices[1].work, 1e-6),
        "two cuts, one price: {prices:?}"
    );
}

/// Invariant: a cell with wall behind it leaves with that wall through the free face beyond, so
/// two leaves bonded together price as one wall of their thickness. Relied on by a body meeting
/// a wall more than one cell thick. Mutant: the front layer priced alone (about half).
#[test]
fn a_backed_block_leaves_through_the_free_face_behind_it() {
    let thick = WALL.with_z(0.5);
    let price = |layers: usize| {
        let wall = Wall::new(thick, [16, 12, layers]);
        // The face the push enters by: the layer at the low side of the thickness.
        let front = wall.within(PATCH.0, PATCH.1.with_z(0.5 / layers as f32 - 0.25));
        let standing = wall.standing();
        let plug = standing.plug(&front, THROUGH);
        assert!(!plug.grounded);
        assert_eq!(plug.cells.len(), front.len() * layers);
        standing.shear_out(&plug, THROUGH).unwrap()
    };
    let (two, one) = (price(2), price(1));
    assert!(
        close(two.force, one.force, 0.02) && close(two.work, one.work, 0.02),
        "two layers price as one: {two:?} against {one:?}"
    );
}

/// Invariant: a pressed cell with standing wall on every other face must be compacted, and one
/// with a side open (or giving way with it) splits, each at its material row's own energy.
/// Relied on by a thick mass stopping what a thin wall cannot. Mutant: the two swapped.
#[test]
fn a_boxed_in_cell_costs_its_compaction_and_an_open_one_its_splitting() {
    let wall = Wall::new(Vec3::splat(1.5), [3, 3, 3]);
    let standing = wall.standing();
    let material = wall.kind.structure.chunks[0].material;
    let (strength, volume) = (material.compressive_strength, 0.125);
    let work = |pressed: Vec<usize>| {
        let plug = standing.plug(&pressed, THROUGH);
        assert!(plug.cells.len() > pressed.len(), "the mass backs them");
        standing.crush(&pressed, &plug, THROUGH).unwrap().work / pressed.len() as f64
    };
    // The middle cell of the entered face, and that whole face.
    let boxed = work(vec![4]);
    let open = work((0..9).collect());
    let compaction = strength * f64::from(material.compaction.lock_up) * volume;
    let splitting = strength * strength / (2.0 * material.young) * volume;
    assert!(
        close(boxed, compaction, 1e-6) && close(open, splitting, 1e-6),
        "boxed in {boxed} J against {compaction}; open {open} J against {splitting}"
    );
    assert!(
        compaction > 10.0 * splitting,
        "the rows make boxing in dear"
    );
}

/// Invariant: a cell returns its strength for no longer than a body takes to cross its give, so
/// what a fast body loses to it is `W / v` whatever the step, and what a slow one loses is
/// `F dt`. Relied on by the tick-rate independence of a ram. Mutant: `F dt` alone (the fast
/// body's loss halves with the step). Fixture price and speeds.
#[test]
fn a_cell_returns_its_strength_for_no_longer_than_its_give() {
    let price = Price {
        force: 36_000.0,
        work: 400.0,
    };
    let give = price.work / price.force;
    let (dt, fast, slow) = (1.0 / 64.0, 10.0, 0.1);
    assert!(fast * dt / 2.0 > give && slow * dt < give);
    for step in [dt, dt / 2.0] {
        assert!(close(
            reaction(price, 0.0, fast, step),
            price.work / fast,
            1e-12
        ));
        assert!(close(
            reaction(price, 0.0, slow, step),
            price.force * step,
            1e-12
        ));
        assert_eq!(reaction(price, 0.0, 0.0, step), price.force * step);
    }
    // Give already crossed is not returned again.
    assert!(close(
        reaction(price, 0.75, fast, dt),
        0.25 * price.work / fast,
        1e-12
    ));
    assert!(close(crossed(price, slow, dt), slow * dt / give, 1e-12));
}

/// Invariant: a push that runs into the ground shears nothing out, and a face slid along is not
/// bounded, so a body resting on a wall is borne at the wall's crushing strength and a body
/// passing along it is not held by it. Relied on by rubble at rest and by a hull grazing a wall.
/// Mutant: the ground treated as a free face (the wall top prices at its seams).
#[test]
fn a_push_into_the_ground_crushes_or_holds() {
    let wall = Wall::new(WALL, [16, 12, 1]);
    let standing = wall.standing();
    let top = wall.within(Vec3::new(-0.5, 2.2, -1.0), Vec3::new(0.5, 2.4, 1.0));
    let down = Push {
        along: DVec3::NEG_Y,
        closing: 0.0,
        lean: 1.0,
    };
    let plug = standing.plug(&top, down.along);
    assert!(plug.grounded && plug.cells.len() == top.len() * 12);
    assert_eq!(standing.shear_out(&plug, down.along), None);
    let dt = 1.0 / 64.0;
    let given = standing.give(&top, &[down], 0.0, dt).unwrap();
    let material = wall.kind.structure.chunks[0].material;
    assert_eq!(given.way, Way::Crush);
    assert!(close(
        given.price.force,
        material.compressive_strength * 1.0 * 0.25,
        1e-5
    ));
    let along_the_face = Push {
        along: DVec3::X,
        closing: 5.0,
        lean: 0.0,
    };
    assert_eq!(standing.give(&top, &[along_the_face], 0.0, dt), None);
}
