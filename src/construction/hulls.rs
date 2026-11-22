//! Each chunk's collision hulls, built once when its kind is admitted (the game's environment
//! destruction): a standing or departing chunk clones them, and a collapse builds none.

use avian3d::dynamics::rigid_body::mass_properties::bevy_heavy::{
    ComputeMassProperties3d, MassProperties3d,
};
use avian3d::prelude::*;
use bevy::prelude::*;

use super::fracture::{Block, Side, Structure};

#[derive(Debug, Clone)]
pub(crate) struct Hulls {
    /// Per chunk: its contact hull about its centroid ([`super::breakage::contact_collider`]),
    /// what a freed chunk meets other bodies with.
    contact: Vec<Collider>,
    /// Per chunk: its occupied hull about its centroid and that hull's mass properties at its
    /// material's density.
    occupied: Vec<(Collider, MassProperties3d)>,
}

/// Two kinds' hulls are one when their occupied masses are: both are functions of the kind's
/// blocks and structure, which a kind compares beside them.
impl PartialEq for Hulls {
    fn eq(&self, other: &Self) -> bool {
        let masses = |hulls: &Self| -> Vec<MassProperties3d> {
            hulls.occupied.iter().map(|(_, props)| *props).collect()
        };
        masses(self) == masses(other)
    }
}

/// Resolution: the subdivision detail Avian's insertion hook scales a collider with.
const SCALE_DETAIL: u32 = 10;

/// `collider` at the scale Avian's insertion hook gives it on a body turned by `rotation`: the
/// world pose's rotation-only scale, quaternion-to-matrix rounding included.
pub(crate) fn posed(mut collider: Collider, rotation: Quat) -> Collider {
    let scale = GlobalTransform::from(Transform::from_rotation(rotation)).scale();
    collider.set_scale(scale, SCALE_DETAIL);
    // `set_scale` logs and keeps the old scale when it fails.
    assert_eq!(
        collider.scale(),
        scale,
        "a convex hull rescales by its rotation's rounding"
    );
    collider
}

impl Hulls {
    /// Refuses a joint on no face of its chunks or a contact hull that cannot be built; the
    /// caller names the asset file.
    pub(crate) fn new(blocks: &[Block], structure: &Structure) -> Result<Self, String> {
        let mut incident = vec![Vec::new(); blocks.len()];
        for (j, joint) in structure.joints.iter().enumerate() {
            incident[joint.a].push(j);
            if let Side::Chunk(b) = joint.b {
                incident[b].push(j);
            }
        }
        let mut contact = Vec::with_capacity(blocks.len());
        let mut occupied = Vec::with_capacity(blocks.len());
        for (c, (block, chunk)) in blocks.iter().zip(&structure.chunks).enumerate() {
            let points: Vec<Vec3> = (block.points.iter())
                .map(|point| *point - chunk.centroid)
                .collect();
            let centred = Collider::convex_hull(&points)
                .expect("admitted chunk hulls stay solid about any origin");
            let props = centred.mass_properties(chunk.material.density);
            occupied.push((centred, props));
            let joints = incident[c].iter().map(|&j| (j, &structure.joints[j]));
            let faces = super::breakage::joint_faces(block, joints).map_err(|j| {
                format!(
                    "joint {j} lies on no face of chunk {c}, so freed the chunk would meet its \
                     neighbour there with no contact gap: bake the construction again so each \
                     joint between chunks lies on a face of both"
                )
            })?;
            let hull = super::breakage::contact_collider(block, &faces, chunk.centroid)
                .ok_or_else(|| {
                    format!(
                        "chunk {c} leaves no solid once its joint faces stand {} m back for \
                         contact: author it thicker across its joints",
                        super::breakage::LOOSE_GAP_M
                    )
                })?;
            contact.push(hull);
        }
        Ok(Self { contact, occupied })
    }

    pub(crate) fn contact(&self, chunk: usize) -> &Collider {
        &self.contact[chunk]
    }

    /// Chunk `chunk`'s occupied hull and its mass properties, in the centroid frame.
    pub(crate) fn occupied(&self, chunk: usize) -> &(Collider, MassProperties3d) {
        &self.occupied[chunk]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::construction::fracture::{Bond, Chunk, Joint, Material};

    /// Fixture: a row of unit-section brick slabs along x, `widths` m thick, each joined to the
    /// next on their shared face; `off_face` moves the joint after slab `a` to `at`.
    fn slab_row(widths: &[f32], off_face: Option<(usize, Vec3)>) -> (Vec<Block>, Structure) {
        let material = Material::of(
            crate::substances::SubstanceRegistry::shipped_static(),
            "Brick",
        )
        .unwrap()
        .unwrap();
        let (mut blocks, mut structure, mut x) = (Vec::new(), Structure::default(), 0.0);
        for (c, &width) in widths.iter().enumerate() {
            let points = [x, x + width]
                .into_iter()
                .flat_map(|x| {
                    [-0.5, 0.5]
                        .into_iter()
                        .flat_map(move |y| [-0.5, 0.5].map(move |z| Vec3::new(x, y, z)))
                })
                .collect::<Vec<_>>();
            blocks.push(Block::hull(points).unwrap());
            structure.chunks.push(Chunk {
                volume: width,
                mass: width * material.density,
                centroid: Vec3::new(x + width / 2.0, 0.0, 0.0),
                material,
                buried: false,
            });
            x += width;
            if c > 0 {
                let centroid = match off_face {
                    Some((a, at)) if a + 1 == c => at,
                    _ => Vec3::new(x - width, 0.0, 0.0),
                };
                structure.joints.push(Joint {
                    a: c - 1,
                    b: Side::Chunk(c),
                    area: 1.0,
                    centroid,
                    normal: Vec3::X,
                    bond: Bond::Bonded(material.seam),
                });
            }
        }
        structure.part_of = (0..widths.len()).collect();
        (blocks, structure)
    }

    /// The game's environment destruction: the load gate refuses a joint between chunks that lies
    /// on no face of one of them, naming the joint and the chunk, since its contact hull could not
    /// stand back there; and a chunk thinner than its two setbacks, naming the chunk and the fix.
    /// Mutants: silently skip the unmatched joint, or build no contact hull for the sliver.
    #[test]
    fn the_load_gate_refuses_a_joint_off_its_faces_and_a_chunk_its_setbacks_consume() {
        let (blocks, structure) = slab_row(&[0.5, 0.5], None);
        Hulls::new(&blocks, &structure).expect("joints on shared faces are admitted");
        // Fixture: the joint's centroid inside slab 0, on no face of either slab.
        let (blocks, structure) = slab_row(&[0.5, 0.5], Some((0, Vec3::new(0.25, 0.0, 0.0))));
        let refusal = Hulls::new(&blocks, &structure).unwrap_err();
        assert!(
            refusal.contains("joint 0") && refusal.contains("chunk 0") && refusal.contains("bake"),
            "{refusal}"
        );
        // Fixture: a sliver between jointed slabs, thinner than its two face setbacks.
        let sliver = 1.5 * super::super::breakage::LOOSE_GAP_M;
        assert!(sliver < 2.0 * super::super::breakage::LOOSE_GAP_M);
        let (blocks, structure) = slab_row(&[0.5, sliver, 0.5], None);
        let refusal = Hulls::new(&blocks, &structure).unwrap_err();
        assert!(
            refusal.contains("chunk 1") && refusal.contains("thicker"),
            "{refusal}"
        );
    }
}
