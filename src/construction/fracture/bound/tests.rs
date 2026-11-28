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
    work: Vec<f64>,
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
        Self {
            work: vec![0.0; kind.structure.chunks.len()],
            kind,
            state,
            preload,
        }
    }

    fn standing(&self) -> Standing<'_> {
        Standing {
            structure: &self.kind.structure,
            state: &self.state,
            shapes: &self.kind.layout.blocks,
            preload: &self.preload,
            work: &self.work,
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
            let (price, leaves) = standing.shear_out(&pressed, THROUGH).unwrap();
            assert_eq!(leaves, pressed, "a single leaf has nothing behind");
            price
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
        let (price, leaves) = standing.shear_out(&front, THROUGH).unwrap();
        assert_eq!(leaves.len(), front.len() * layers);
        price
    };
    let (two, one) = (price(2), price(1));
    assert!(
        // One boundary summed in two orders.
        close(two.force, one.force, 1e-6) && close(two.work, one.work, 1e-6),
        "two layers price as one: {two:?} against {one:?}"
    );
}

/// Invariant: a cell returns a belt its strength for no longer than the belt takes to do the work
/// its bonds have left, so what a fast belt loses to it is `W / v` whatever the step, what a slow
/// one loses is `F dt`, and work already taken is not returned again. Relied on by the
/// tick-rate independence of a ram. Mutants: `F dt` alone (the fast body's loss halves with the
/// step); the work taken ignored (worked bonds return a fast body the whole `W / v`). Fixture
/// price and speeds.
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
    assert!(close(
        reaction(price, 0.75 * price.work, fast, dt),
        0.25 * price.work / fast,
        1e-12
    ));
    // Spent bonds return a closing body nothing, and hold one that is not closing.
    assert_eq!(reaction(price, 2.0 * price.work, slow, dt), 0.0);
    assert_eq!(reaction(price, price.work, 0.0, dt), price.force * dt);
}

/// Invariant: a leaf pressed along its own length sheds the cells pressed, through its face, at
/// a price that is theirs alone: no dearer for the wall that stands behind them along the push.
/// Relied on by a body, or a piece a body pins, meeting a wall end-on (`breakage::bounded`).
/// Mutants: the plug behind the cells priced (the longer wall dearer), or the cells shed along
/// the wall (more than the pressed leave). Fixture: two walls, one twice the other's length.
#[test]
fn a_leaf_pressed_along_its_length_sheds_the_pressed_cells_through_its_face() {
    let end_on = DVec3::X;
    let shed = |length: f32, cells: usize| {
        let wall = Wall::new(WALL.with_x(length), [cells, 12, 1]);
        let pressed = wall.within(
            Vec3::new(-length / 2.0, 0.8, -1.0),
            Vec3::new(-length / 2.0 + 0.25, 1.6, 1.0),
        );
        assert_eq!(pressed.len(), 4, "the end column's middle cells");
        let standing = wall.standing();
        let (through, plug) = standing.shear_out(&pressed, end_on).unwrap();
        assert_eq!(plug.len(), pressed.len() * cells, "their courses back them");
        let (price, leaves) = standing.shed(&pressed, end_on).unwrap();
        assert_eq!(leaves, pressed);
        assert!(price.force < through.force, "{price:?} against {through:?}");
        price
    };
    let (short, long) = (shed(4.0, 16), shed(8.0, 32));
    assert!(
        close(short.force, long.force, 1e-6) && close(short.work, long.work, 1e-6),
        "one price whatever stands behind: {short:?} against {long:?}"
    );
}

/// Invariant: a push that runs into the ground shears nothing out; the cells it presses can
/// only slide off sideways, and the joint the push closes holds them by its friction under the
/// push itself, so they return at least what their joints hold over `1 − μ` (Coulomb; `μ` is
/// the substance pair's). A face slid along is not bounded. Relied on by rubble and a hull
/// borne on a wall, and by a hull grazing one. Mutants: the ground treated as a free face (the
/// wall top prices at its seams), the closed joint carrying nothing (the top is shed at its
/// head joints' price).
#[test]
fn a_push_into_the_ground_sheds_sideways_against_its_own_friction() {
    let wall = Wall::new(WALL, [16, 12, 1]);
    let top = wall.within(Vec3::new(-0.5, 2.2, -1.0), Vec3::new(0.5, 2.4, 1.0));
    let down = Push {
        along: DVec3::NEG_Y,
        closing: 0.0,
        lean: 1.0,
    };
    let material = wall.kind.structure.chunks[0].material;
    let friction = f64::from(crate::substances::pair_friction(
        material.friction,
        material.friction,
    ));
    assert!(friction < 1.0, "fixture: the pair slides");
    let standing = wall.standing();
    let given = {
        assert_eq!(standing.shear_out(&top, down.along), None);
        let along_the_face = Push {
            along: DVec3::X,
            closing: 5.0,
            lean: 0.0,
        };
        assert_eq!(standing.give_along(&top, along_the_face), None);
        standing.give_along(&top, down).unwrap()
    };
    assert_eq!((given.way, &given.leaves), (Way::Shed, &top));
    // What slides when the cells leave through either face, from the fixture's geometry: the
    // bed joint under them and the head joint at each end, each at its cohesion at least.
    let cell = WALL / Vec3::new(16.0, 12.0, 1.0);
    let sliding = f64::from((PATCH.1.x - PATCH.0.x) * WALL.z + 2.0 * cell.y * WALL.z);
    let cohesion = (wall.kind.structure.joints.iter())
        .find_map(|joint| joint.seam())
        .map(|seam| f64::from(seam.shear.cohesion))
        .unwrap();
    let beside = cohesion * sliding;
    assert!(
        given.price.force * (1.0 - friction) >= beside * (1.0 - 1e-6),
        "{:?} against the joints beside alone, {beside} N, at friction {friction}",
        given.price
    );
}
