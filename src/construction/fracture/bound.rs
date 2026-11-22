//! The game's building model (game state only): what a standing cell returns to a body that
//! presses it. Bonds hold up to a force and let go after a work, so a cell answers with at most
//! `F` for no longer than the body takes to cross its give `W / F` ([`reaction`]); a cell pushed
//! past that is loose rubble. Nothing here predicts a path: the cells are the ones the narrow
//! phase found pressed. The design narrative is
//! `.agents/docs/design/construction-destruction.md`, "Bodies in contact".
//!
//! Pure functions of one structure's state ([`Standing`]); `breakage::bounded` hands the
//! result to the solver's contacts and frees what gave.

use bevy::math::{DVec3, Vec3};

use super::bondsolve::{joint_normal, pair_friction};
use super::{Block, Joint, Side, State, Structure};

/// A neighbour is behind a cell along a push when the push closes their joint more than it
/// slides along it. DERIVED: cos 45°, where the two modes of one rigid translation are equal.
const BEHIND: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// Resolution: two faces' sideways directions within about 8 degrees are one side to shed
/// through (cos); baked faces of one wall plane differ by rounding only.
const SAME_SIDE: f64 = 0.99;

fn chunk(side: Side) -> Option<usize> {
    match side {
        Side::Chunk(c) => Some(c),
        Side::Ground => None,
    }
}

/// What one way of giving holds, N, and the work it takes before it lets go, J. Its give is
/// `work / force`, m, never authored.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Price {
    pub force: f64,
    pub work: f64,
}

/// The impulse a cell priced at `price` returns over a step of `dt` s to a body closing on it at
/// `closing` m/s, N·s, `spent` of its give already crossed: its strength for the step, or for the
/// time the body takes to cross the give left, whichever is shorter. The second holds no `dt`,
/// which is what makes a fast body's loss independent of the step.
pub(crate) fn reaction(price: Price, spent: f64, closing: f64, dt: f64) -> f64 {
    if price.force <= 0.0 {
        return 0.0; // Nothing holds it.
    }
    let left = (1.0 - spent) * price.work / price.force;
    price.force
        * if closing * dt > left {
            left / closing
        } else {
            dt
        }
}

/// The share of a price's give a body closing at `closing` m/s crosses in `dt` s.
pub(crate) fn crossed(price: Price, closing: f64, dt: f64) -> f64 {
    if price.work <= 0.0 {
        return 1.0; // No give: it lets go at once.
    }
    closing.max(0.0) * dt * price.force / price.work
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

/// How pressed cells give to a body: the cheapest way and direction.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Yield {
    pub way: Way,
    pub push: Push,
    pub price: Price,
    /// The most the cells return along the push this step, N·s ([`reaction`]).
    pub impulse: f64,
    /// The cells that leave if it gives, ordered.
    pub leaves: Vec<usize>,
    /// The push runs into the ground or buried matter ([`Plug::grounded`]).
    pub grounded: bool,
}

/// The matter one push moves: the pressed cells and every cell behind them along it.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Plug {
    /// Ordered.
    pub cells: Vec<usize>,
    /// The push runs into the ground or buried matter: nothing leaves that way.
    pub grounded: bool,
}

/// One structure as the law reads it: its graph and live state, its chunk shapes, and its gravity
/// preload per joint (N, tension positive; `parts::Gravity`).
#[derive(Clone, Copy)]
pub(crate) struct Standing<'a> {
    pub structure: &'a Structure,
    pub state: &'a State,
    pub shapes: &'a [Block],
    pub preload: &'a [f64],
    /// The ram instrument's arm (`net::ram::relay`, `bare`): a joint the push closes resists a
    /// sideways shed with nothing.
    pub bare: bool,
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
                    let other = if joint.a == c {
                        chunk(joint.b)
                    } else {
                        Some(joint.a)
                    };
                    if let Some(other) = other
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
    /// nothing, so the cells behind go with them, up to a free face.
    pub(crate) fn plug(&self, pressed: &[usize], along: DVec3) -> Plug {
        let topology = &self.state.topology;
        let mut inside = vec![false; self.structure.chunks.len()];
        let mut grounded = false;
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
                let other = if joint.a == c {
                    chunk(joint.b)
                } else {
                    Some(joint.a)
                };
                match other {
                    Some(d) if self.structure.chunks[d].buried => grounded = true,
                    Some(d) if !inside[d] => {
                        inside[d] = true;
                        open.push(d);
                    }
                    Some(_) => {}
                    None => grounded = true,
                }
            }
        }
        Plug {
            cells: (0..inside.len()).filter(|&c| inside[c]).collect(),
            grounded,
        }
    }

    /// What `plug` holds against leaving `along`, summed over the joints on its boundary: each
    /// opens at its tensile strength and slides at its cohesion plus the friction of its gravity
    /// preload (`bondsolve::JointStrength`), and spends its fracture energy. `None` when the push
    /// runs into the ground.
    pub(crate) fn shear_out(&self, plug: &Plug, along: DVec3) -> Option<Price> {
        (!plug.grounded).then(|| self.boundary(&plug.cells, along, None).0)
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
                let across = if joint.a == c {
                    chunk(joint.b)
                } else {
                    Some(joint.a)
                };
                let internal = across.is_some_and(|d| cells.binary_search(&d).is_ok());
                if internal || !self.touching(j, joint) {
                    continue;
                }
                let normal = self.toward(joint, c);
                let area = f64::from(joint.area);
                if push.is_some_and(|push| -normal.dot(push) > BEHIND) {
                    if self.bare {
                        continue;
                    }
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
            }
        }
        (price, closed, rough)
    }

    /// What `pressed` hold against being shed sideways by a push `along`, and the cells that
    /// leave: over the directions the push slides a pressed cell's face along, the cheapest
    /// whose matter can move (its plug is not grounded). The joints the push closes slide under
    /// the push itself, so their friction grows with the reaction `P`: sliding starts at
    /// `P = R + μ P`, `R` the boundary's resistance at its gravity preload and `μ` the closed
    /// joints' friction, area-weighted (the push bears on them evenly): `P = R / (1 − μ)`
    /// (exact, Coulomb). Closed joints at `μ ≥ 1` never slide. `None` when no side is open.
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
            let plug = self.plug(pressed, side);
            if plug.grounded {
                continue;
            }
            let (mut price, closed, rough) = self.boundary(&plug.cells, side, Some(along));
            if closed > 0.0 {
                let friction = rough / closed;
                if friction >= 1.0 {
                    continue;
                }
                price.force /= 1.0 - friction;
            }
            if least
                .as_ref()
                .is_none_or(|(least, _)| price.force < least.force)
            {
                least = Some((price, plug.cells));
            }
        }
        least
    }

    /// How `pressed` (ordered) give to a body pushing them along `push`, `spent` of their give
    /// crossed already: the way that returns the body least this step. `None` when no way is
    /// open along it.
    pub(crate) fn give_along(
        &self,
        pressed: &[usize],
        push: Push,
        spent: f64,
        dt: f64,
    ) -> Option<Yield> {
        let plug = self.plug(pressed, push.along);
        let grounded = plug.grounded;
        let ways = [
            (self.shear_out(&plug, push.along)).map(|price| (Way::ShearOut, price, plug.cells)),
            (self.shed(pressed, push.along)).map(|(price, leaves)| (Way::Shed, price, leaves)),
        ];
        let mut least: Option<Yield> = None;
        for (way, price, leaves) in ways.into_iter().flatten() {
            let impulse = reaction(price, spent, push.closing, dt);
            if least.as_ref().is_none_or(|least| impulse < least.impulse) {
                least = Some(Yield {
                    way,
                    push,
                    price,
                    impulse,
                    leaves,
                    grounded,
                });
            }
        }
        least
    }

    /// [`Self::give_along`] the push of `pushes` that returns the body's contact least: the
    /// cheapest way and direction. `None` when no push moves them (every lean is nil or away).
    pub(crate) fn give(
        &self,
        pressed: &[usize],
        pushes: &[Push],
        spent: f64,
        dt: f64,
    ) -> Option<Yield> {
        (pushes.iter())
            .filter(|push| push.lean > 0.0)
            .filter_map(|&push| self.give_along(pressed, push, spent, dt))
            .min_by(|a, b| (a.impulse / a.push.lean).total_cmp(&(b.impulse / b.push.lean)))
    }
}

#[cfg(test)]
mod tests;
