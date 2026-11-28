//! The game's building model (game state only): what a standing cell returns to a body that
//! presses it. Bonds hold up to a force and let go after a work, so a cell answers with at most
//! `F` ([`Price::force`]) until the bodies pressing it have done `W` on it ([`Standing::work`]);
//! its bonds are then spent and it is loose rubble. Nothing here predicts a path: the cells are the
//! ones the narrow phase found pressed. The design narrative is
//! `.agents/scratch/strength-limited-contact-design-2026-11-21.md`, "As built".
//!
//! Pure functions of one structure's state ([`Standing`]); `breakage::bounded` hands the
//! result to the solver's contacts and frees what gave.

use bevy::math::DVec3;

use super::bondsolve::{joint_normal, pair_friction};
use super::{Block, Joint, Side, State, Structure};

/// A neighbour is behind a cell along a push when the push closes their joint more than it
/// slides along it. DERIVED: cos 45°, where the two modes of one rigid translation are equal.
const BEHIND: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// Resolution: sideways directions within about 8 degrees (cos) are priced as one side to shed
/// through, since each side priced costs a plug and a boundary sum. The faces of one wall plane
/// differ by rounding only, far inside it; the angle itself is not measured.
const SAME_SIDE: f64 = 0.99;

/// The chunk across `joint` from its side `c`; none across a ground joint.
fn across(joint: &Joint, c: usize) -> Option<usize> {
    if joint.a != c {
        return Some(joint.a);
    }
    match joint.b {
        Side::Chunk(b) => Some(b),
        Side::Ground => None,
    }
}

/// What one way of giving holds, N, and the work it takes before it lets go, J.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Price {
    pub force: f64,
    pub work: f64,
}

impl Price {
    /// Whether bonds that have taken `taken` J are spent: the work has reached theirs. The
    /// remainder rounding leaves: the design narrative, "What rounding leaves".
    pub(crate) fn spent(&self, taken: f64) -> bool {
        taken >= self.work
    }

    /// The work the bonds still take, J, having taken `taken`.
    pub(crate) fn left(&self, taken: f64) -> f64 {
        (self.work - taken).max(0.0)
    }
}

/// The work the bonds of `cells` have taken, J, `work` their structure's per chunk
/// ([`Standing::work`]). Work is one scalar per cell, whoever did it and along whichever way:
/// every way a block gives is charged with all of it, so a way may part on work done along
/// another, or by another body. That is a simplification; its measured cost is in the design
/// narrative (the module doc).
pub(crate) fn taken(work: &[f64], cells: &[usize]) -> f64 {
    cells.iter().map(|&c| work[c]).sum()
}

/// The give cells priced at `price` have left, m, their bonds having taken `taken` J: the
/// distance over which they still return their strength.
fn give_left(price: Price, taken: f64) -> f64 {
    price.left(taken) / price.force
}

/// Whether a belt closing at `closing` m/s on cells priced at `price` crosses the give they have
/// left ([`give_left`]) within a step of `dt` s, if it keeps that speed. Cells that hold nothing
/// have no give to cross. A belt's force is staged before the step, so its start speed is the
/// only one its bound can read; a solver point's work is bounded inside the solve
/// (`avian_ext::reaction_bound`) and reads no speed here.
pub(crate) fn crosses(price: Price, taken: f64, closing: f64, dt: f64) -> bool {
    price.force > 0.0 && closing * dt > give_left(price, taken)
}

/// The impulse cells priced at `price` return over a step of `dt` s to a belt closing on them at
/// `closing` m/s, N·s, their bonds having taken `taken` J already: their strength for the step,
/// or for the time the body takes to do the work left, whichever is shorter ([`crosses`]). The
/// second holds no `dt`, which is what makes a fast body's loss independent of the step.
pub(crate) fn reaction(price: Price, taken: f64, closing: f64, dt: f64) -> f64 {
    if price.force <= 0.0 {
        return 0.0; // Nothing holds it.
    }
    price.force
        * if crosses(price, taken, closing, dt) {
            give_left(price, taken) / closing
        } else {
            dt
        }
}

/// How pressed cells give.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Way {
    /// The pressed cells and the matter behind them part from the rest at their boundary and
    /// leave through the free face behind.
    ShearOut,
    /// The pressed cells, and the matter beside them, leave sideways through a free face the
    /// push runs along: a leaf pressed along its own plane sheds its pressed layer.
    Shed,
}

/// One direction a body pushes pressed cells in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Push {
    /// Unit, in the structure's frame.
    pub along: DVec3,
    /// The body's speed along it where it presses, at the step's start, m/s.
    pub closing: f64,
    /// The contact normal's part along it: the share of a normal impulse that moves the cells
    /// this way. A face slid along has none, and no bound binds it.
    pub lean: f64,
}

/// One way pressed cells give to a body along one push.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Yield {
    pub way: Way,
    pub push: Push,
    pub price: Price,
    /// The cells that leave if it gives, ordered.
    pub leaves: Vec<usize>,
}

/// How pressed cells give to a belt: the way, the most they return along the push this step,
/// N·s ([`reaction`]), and whether the belt crosses the give they have left within the step
/// ([`crosses`]).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Rationed {
    pub given: Yield,
    pub impulse: f64,
    pub crossing: bool,
}

/// The matter one push moves.
enum Plug {
    /// The pressed cells and every cell behind them along the push, ordered.
    Free(Vec<usize>),
    /// The push runs into the ground or buried matter: nothing leaves that way, and no cells
    /// are named.
    Grounded,
}

/// One structure as the law reads it: its graph and live state, its chunk shapes, its gravity
/// preload per joint (N, tension positive; `parts::Gravity`), and the work each chunk's bonds
/// have taken from the bodies that pressed it, J.
#[derive(Clone, Copy)]
pub(crate) struct Standing<'a> {
    pub structure: &'a Structure,
    pub state: &'a State,
    pub shapes: &'a [Block],
    pub preload: &'a [f64],
    pub work: &'a [f64],
}

impl Standing<'_> {
    /// Whether `joint` (index `j`) still presses its two sides together.
    fn touching(&self, j: usize, joint: &Joint) -> bool {
        !self.state.separated[j] && self.state.live(joint)
    }

    /// `joint`'s unit normal toward chunk `c`, one of its sides.
    fn toward(&self, joint: &Joint, c: usize) -> DVec3 {
        let toward_a = joint_normal(self.structure, joint);
        if joint.a == c { toward_a } else { -toward_a }
    }

    /// `cells` split into the ordered sets its touching joints connect: each is pressed as one
    /// block.
    pub(crate) fn connected(&self, cells: &[usize]) -> Vec<Vec<usize>> {
        let mut left: std::collections::BTreeSet<usize> = cells.iter().copied().collect();
        let mut blocks = Vec::new();
        while let Some(first) = left.pop_first() {
            let mut block = vec![first];
            let mut open = vec![first];
            while let Some(c) = open.pop() {
                for &j in &self.state.topology.incident[c] {
                    let joint = &self.structure.joints[j];
                    if let Some(other) = across(joint, c)
                        && self.touching(j, joint)
                        && left.remove(&other)
                    {
                        block.push(other);
                        open.push(other);
                    }
                }
            }
            block.sort_unstable();
            blocks.push(block);
        }
        blocks
    }

    /// The matter a push `along` on `pressed` moves: joints between cells pressed together carry
    /// nothing, so the cells behind go with them, up to a free face. The walk ends where it
    /// meets the ground.
    fn plug(&self, pressed: &[usize], along: DVec3) -> Plug {
        let topology = &self.state.topology;
        let mut inside = vec![false; self.structure.chunks.len()];
        let mut open: Vec<usize> = pressed.to_vec();
        for &c in pressed {
            assert!(
                self.state.standing(c) && !self.structure.chunks[c].buried,
                "a body presses standing matter above grade"
            );
            inside[c] = true;
        }
        while let Some(c) = open.pop() {
            for &j in &topology.incident[c] {
                let joint = &self.structure.joints[j];
                if !self.touching(j, joint) || -self.toward(joint, c).dot(along) <= BEHIND {
                    continue;
                }
                match across(joint, c) {
                    Some(d) if self.structure.chunks[d].buried => return Plug::Grounded,
                    Some(d) if !inside[d] => {
                        inside[d] = true;
                        open.push(d);
                    }
                    Some(_) => {}
                    None => return Plug::Grounded,
                }
            }
        }
        Plug::Free((0..inside.len()).filter(|&c| inside[c]).collect())
    }

    /// What the plug behind `pressed` holds against leaving `along`, and its cells, which leave:
    /// summed over the joints on its boundary, each opens at its tensile strength and slides at
    /// its cohesion plus the friction of its gravity preload (`bondsolve::JointStrength`), and
    /// spends its fracture energy. `None` when the push runs into the ground.
    pub(crate) fn shear_out(&self, pressed: &[usize], along: DVec3) -> Option<(Price, Vec<usize>)> {
        match self.plug(pressed, along) {
            Plug::Free(cells) => Some((self.boundary(&cells, along, None).0, cells)),
            Plug::Grounded => None,
        }
    }

    /// What `cells` (ordered) hold against leaving along `slide`, summed over the joints on
    /// their boundary, and of those joints the ones a push along `push` closes: their area, m²,
    /// and that area weighted by their friction.
    fn boundary(&self, cells: &[usize], slide: DVec3, push: Option<DVec3>) -> (Price, f64, f64) {
        let reach =
            |c: &usize| (self.shapes[*c].points.iter()).map(|point| point.as_dvec3().dot(slide));
        let (near, far) = (cells.iter().flat_map(reach))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), at| {
                (lo.min(at), hi.max(at))
            });
        let depth = far - near;
        let mut price = Price {
            force: 0.0,
            work: 0.0,
        };
        let (mut closed, mut rough) = (0.0, 0.0);
        for &c in cells {
            for &j in &self.state.topology.incident[c] {
                let joint = &self.structure.joints[j];
                let internal = across(joint, c).is_some_and(|d| cells.binary_search(&d).is_ok());
                if internal || !self.touching(j, joint) {
                    continue;
                }
                let normal = self.toward(joint, c);
                let area = f64::from(joint.area);
                if push.is_some_and(|push| -normal.dot(push) > BEHIND) {
                    closed += area;
                    rough += area * pair_friction(self.structure, joint);
                }
                let cracked = self.state.cracked[j];
                // No bearing term: a boundary joint the slide closes is slid along, and the
                // closing part is borne by the matter behind it.
                price.force += super::bondsolve::JointStrength::of(self.structure, joint, cracked)
                    .translation_resistance(area, normal, slide, self.preload[j], 0.0);
                let opening = normal.dot(slide).max(0.0);
                let sliding = normal.cross(slide).length();
                if let Some(seam) = joint.seam().filter(|_| !cracked) {
                    price.work += area
                        * (f64::from(seam.fracture_energy) * opening
                            + f64::from(seam.shear.energy) * sliding);
                }
                // StandIn for the grip of standing matter on a loosened plug, which the solver
                // does not carry (a static neighbour bears on nothing): the preload's friction
                // over the plug's depth along the slide.
                price.work += pair_friction(self.structure, joint)
                    * (-self.preload[j]).max(0.0)
                    * sliding
                    * depth;
                assert!(
                    price.work.is_finite() && price.work >= 0.0,
                    "a joint's bonds take a finite work: joint {j} of cell {c} brings the \
                     boundary's to {} J",
                    price.work
                );
            }
        }
        (price, closed, rough)
    }

    /// What `pressed` hold against being shed sideways by a push `along`, and the cells that
    /// leave: over the directions the push slides a pressed cell's face along, the cheapest
    /// whose matter can move (its plug is not grounded). The joints the push closes rub under
    /// the push itself, at their area-weighted friction (the push bears on them evenly), which
    /// raises the price (the StandIn below). `None` when no side is open.
    pub(crate) fn shed(&self, pressed: &[usize], along: DVec3) -> Option<(Price, Vec<usize>)> {
        let mut sides: Vec<DVec3> = Vec::new();
        for &c in pressed {
            for &(normal, _) in &self.shapes[c].planes {
                let normal = normal.as_dvec3();
                if normal.dot(along).abs() >= BEHIND {
                    continue; // The push closes or opens this face more than it slides along it.
                }
                let side = (normal - along * normal.dot(along)).normalize();
                if !sides.iter().any(|known| known.dot(side) > SAME_SIDE) {
                    sides.push(side);
                }
            }
        }
        let mut least: Option<(Price, Vec<usize>)> = None;
        for side in sides {
            let Plug::Free(cells) = self.plug(pressed, side) else {
                continue; // Its matter cannot move this way.
            };
            let (mut price, closed, rough) = self.boundary(&cells, side, Some(along));
            if closed > 0.0 {
                let friction = rough / closed;
                // StandIn for the sideways share of a push, the missing law (how much of a push
                // `P` on a block turns sideways, `κ P`, by its geometry and what backs it): here
                // `κ = 1`, the whole push, which the closed joints rub at `μ P`, so the bonds'
                // `R` is passed at `P = R + μ P`, `R / (1 - μ)`. Past `μ = 1` this way is shut.
                if friction >= 1.0 {
                    continue;
                }
                price.force /= 1.0 - friction;
            }
            if least
                .as_ref()
                .is_none_or(|(least, _)| price.force < least.force)
            {
                least = Some((price, cells));
            }
        }
        least
    }

    /// The ways open for `pressed` (ordered) along `push`, shearing out first. None when the
    /// push leans no part of a normal impulse its way ([`Push::lean`]).
    fn ways(&self, pressed: &[usize], push: Push) -> impl Iterator<Item = Yield> {
        let open = (push.lean > 0.0).then(|| {
            [
                (Way::ShearOut, self.shear_out(pressed, push.along)),
                (Way::Shed, self.shed(pressed, push.along)),
            ]
        });
        (open.into_iter().flatten()).filter_map(move |(way, parting)| {
            parting.map(|(price, leaves)| Yield {
                way,
                push,
                price,
                leaves,
            })
        })
    }

    /// How `pressed` (ordered) give to a body pushing them along `push`: the way that holds
    /// least, the first a rising force moves; the first of equals. `None` when no way is open
    /// along it.
    pub(crate) fn give_along(&self, pressed: &[usize], push: Push) -> Option<Yield> {
        (self.ways(pressed, push)).min_by(|a, b| a.price.force.total_cmp(&b.price.force))
    }

    /// How `pressed` (ordered) give to a belt pushing them along `push`: the way that returns
    /// the belt least this step ([`reaction`]), after the work their bonds have taken
    /// ([`taken`]); the first of equals. `None` when no way is open along it.
    pub(crate) fn give_to_belt(&self, pressed: &[usize], push: Push, dt: f64) -> Option<Rationed> {
        let taken = taken(self.work, pressed);
        (self.ways(pressed, push))
            .map(|given| Rationed {
                impulse: reaction(given.price, taken, push.closing, dt),
                crossing: crosses(given.price, taken, push.closing, dt),
                given,
            })
            .min_by(|a, b| a.impulse.total_cmp(&b.impulse))
    }
}

#[cfg(test)]
mod tests;
