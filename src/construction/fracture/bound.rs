//! The game's building model (game state only): what a standing cell returns to a body that
//! presses it. Bonds hold up to a force and let go after a work, so a cell answers with at most
//! `F` for no longer than the body takes to cross its give `W / F` ([`reaction`]); a cell pushed
//! past that is loose rubble. Nothing here predicts a path: the cells are the ones the narrow
//! phase found pressed. The design narrative is
//! `.agents/docs/design/construction-destruction.md`, "Bodies in contact".
//!
//! Pure functions of one structure's state ([`Standing`]); `breakage::bounded` hands the
//! result to the solver's contacts and frees what gave.

#![cfg_attr(
    not(test),
    expect(dead_code, reason = "the law's wiring lands in the next change")
)]

use bevy::math::{DVec3, Vec3};

use super::bondsolve::{joint_normal, pair_friction};
use super::{Block, Joint, Side, State, Structure, joint_face, silhouette};

/// A neighbour is behind a cell along a push when the push closes their joint more than it
/// slides along it. DERIVED: cos 45°, where the two modes of one rigid translation are equal.
const BEHIND: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// Per chunk, per face of its shape: the face and the joints lying on it, baked at admission.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Faces(Vec<Vec<Face>>);

#[derive(Debug, Clone, PartialEq)]
struct Face {
    /// Outward unit normal and area, m².
    plane: (Vec3, f32),
    joints: Vec<usize>,
}

impl Faces {
    pub(crate) fn new(shapes: &[Block], structure: &Structure) -> Self {
        let mut faces: Vec<Vec<Face>> = (shapes.iter())
            .map(|shape| {
                (shape.face_areas().into_iter())
                    .map(|plane| Face {
                        plane,
                        joints: Vec::new(),
                    })
                    .collect()
            })
            .collect();
        for (j, joint) in structure.joints.iter().enumerate() {
            let sides = [Some(joint.a), chunk(joint.b)];
            for c in sides.into_iter().flatten() {
                // A ground anchor may lie on no face (soil bearing a chunk's side): it covers none.
                if let Some(face) = joint_face(&shapes[c], joint) {
                    faces[c][face].joints.push(j);
                }
            }
        }
        Self(faces)
    }
}

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
    /// The pressed cells are crushed against what backs them.
    Crush,
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
}

/// The matter one push moves: the pressed cells and every cell behind them along it.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Plug {
    /// Ordered.
    pub cells: Vec<usize>,
    /// The push runs into the ground or buried matter: nothing leaves that way.
    pub grounded: bool,
}

/// One structure as the law reads it: its graph and live state, its chunk shapes and their faces,
/// and its gravity preload per joint (N, tension positive; `parts::Gravity`).
#[derive(Clone, Copy)]
pub(crate) struct Standing<'a> {
    pub structure: &'a Structure,
    pub state: &'a State,
    pub shapes: &'a [Block],
    pub faces: &'a Faces,
    pub preload: &'a [f64],
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
        if plug.grounded {
            return None;
        }
        let reach =
            |c: &usize| (self.shapes[*c].points.iter()).map(|point| point.as_dvec3().dot(along));
        let (near, far) = (plug.cells.iter().flat_map(reach))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), at| {
                (lo.min(at), hi.max(at))
            });
        let depth = far - near;
        let mut price = Price {
            force: 0.0,
            work: 0.0,
        };
        for &c in &plug.cells {
            for &j in &self.state.topology.incident[c] {
                let joint = &self.structure.joints[j];
                let across = if joint.a == c {
                    chunk(joint.b)
                } else {
                    Some(joint.a)
                };
                let internal = across.is_some_and(|d| plug.cells.binary_search(&d).is_ok());
                if internal || !self.touching(j, joint) {
                    continue;
                }
                let normal = self.toward(joint, c);
                let area = f64::from(joint.area);
                let cracked = self.state.cracked[j];
                // No bearing term: a boundary joint the push closes is slid along, and the
                // closing part is borne by the matter behind it.
                price.force += super::bondsolve::JointStrength::of(self.structure, joint, cracked)
                    .translation_resistance(area, normal, along, self.preload[j], 0.0);
                let opening = normal.dot(along).max(0.0);
                let sliding = normal.cross(along).length();
                if let Some(seam) = joint.seam().filter(|_| !cracked) {
                    price.work += area
                        * (f64::from(seam.fracture_energy) * opening
                            + f64::from(seam.shear.energy) * sliding);
                }
                // StandIn for the grip of standing matter on a loosened plug, which the solver
                // does not carry (a static neighbour bears on nothing): the preload's friction
                // over the plug's depth along the push.
                price.work += pair_friction(self.structure, joint)
                    * (-self.preload[j]).max(0.0)
                    * sliding
                    * depth;
            }
        }
        Some(price)
    }

    /// What `pressed` holds against being crushed `along` where they stand, backed by the rest of
    /// `plug`: their compressive strength over the face they show the push. A cell with a free
    /// side splits, for the elastic energy at crushing, `σ_c² / 2E` per volume (DERIVED; the
    /// compressive fracture energy is `Unknown`, and a measured value would replace the form); a
    /// cell boxed in must be compacted to lock-up, `σ_c ε_v` per volume (the blast law's
    /// [`super::Compaction`] facts). `None` when nothing backs them.
    pub(crate) fn crush(&self, pressed: &[usize], plug: &Plug, along: DVec3) -> Option<Price> {
        if !plug.grounded && plug.cells.len() == pressed.len() {
            return None;
        }
        let mut price = Price {
            force: 0.0,
            work: 0.0,
        };
        for &c in pressed {
            let cell = &self.structure.chunks[c];
            let strength = cell.material.bearing(along);
            let planes: Vec<_> = self.faces.0[c].iter().map(|face| face.plane).collect();
            price.force += strength * f64::from(silhouette(&planes, along.as_vec3()));
            let per_volume = if self.boxed_in(pressed, c, along) {
                strength * f64::from(cell.material.compaction.lock_up)
            } else {
                strength * strength / (2.0 * cell.material.young)
            };
            price.work += per_volume * f64::from(cell.volume);
        }
        Some(price)
    }

    /// Whether pressed cell `c` has nowhere to go: every face but the one the push enters by is
    /// covered, over most of its area, by standing matter that is not itself pressed (a pressed
    /// neighbour is giving way too and confines nothing).
    fn boxed_in(&self, pressed: &[usize], c: usize, along: DVec3) -> bool {
        self.faces.0[c].iter().all(|face| {
            let (normal, area) = face.plane;
            if -normal.as_dvec3().dot(along) > BEHIND {
                return true; // The face the push enters by.
            }
            let covered: f32 = (face.joints.iter())
                .map(|&j| (j, &self.structure.joints[j]))
                .filter(|&(j, joint)| {
                    let across = if joint.a == c {
                        chunk(joint.b)
                    } else {
                        Some(joint.a)
                    };
                    self.touching(j, joint)
                        && across.is_none_or(|d| pressed.binary_search(&d).is_err())
                })
                .map(|(_, joint)| joint.area)
                .sum();
            covered > 0.5 * area
        })
    }

    /// How `pressed` (ordered) give to a body pushing them each of `pushes`' ways, `spent` of
    /// their give crossed already: the way and direction that return the body least this step.
    /// `None` when no push moves them (every lean is nil or away).
    pub(crate) fn give(
        &self,
        pressed: &[usize],
        pushes: &[Push],
        spent: f64,
        dt: f64,
    ) -> Option<Yield> {
        let mut least: Option<(f64, Yield)> = None;
        for &push in pushes {
            if push.lean <= 0.0 {
                continue;
            }
            let plug = self.plug(pressed, push.along);
            let ways = [
                (Way::ShearOut, self.shear_out(&plug, push.along)),
                (Way::Crush, self.crush(pressed, &plug, push.along)),
            ];
            for (way, price) in ways {
                let Some(price) = price else {
                    continue; // The way is closed this direction.
                };
                let impulse = reaction(price, spent, push.closing, dt);
                let normal = impulse / push.lean;
                if least.as_ref().is_none_or(|(least, _)| normal < *least) {
                    let leaves = match way {
                        Way::ShearOut => plug.cells.clone(),
                        Way::Crush => pressed.to_vec(),
                    };
                    least = Some((
                        normal,
                        Yield {
                            way,
                            push,
                            price,
                            impulse,
                            leaves,
                        },
                    ));
                }
            }
        }
        least.map(|(_, given)| given)
    }
}

#[cfg(test)]
mod tests;
