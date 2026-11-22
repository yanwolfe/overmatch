//! The wall generator: a [`Wall`] spec (`tools/constructions/<id>.wall.ron`) to its
//! [`Construction`]. A wall is runs: each a right box along a straight centre line in plan, its
//! thickness across the line, between two heights. A free-standing leaf, a second leaf beside it,
//! a return at a corner, a post and a board are all runs; what each is made of and how it is cut
//! is its part's data, and what holds two runs of different components is the spec's joins.
//!
//! The construction frame of [`overmatch::construction::schema`]: `+Y` up, the origin on the
//! ground point. A planar cut's depth is the run's thickness axis, the horizontal normal of its
//! centre line.

use std::collections::BTreeMap;

use glam::{DVec2, DVec3};
use serde::Deserialize;

use super::fracture::{Pattern, Planar};
use super::geom::{Frame, Prism};
use super::spec::{self, Construction, Join, Part, PartCut, Piece};

/// One wall (the shipped files' own comments carry each number's source).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Wall {
    pub id: String,
    /// The foundation plane's height, m: every run standing on it is held there.
    pub foundation_y_m: f64,
    pub foundation_attachment: String,
    /// What each part is made of, how its surface runs and how it is cut, by part name.
    pub parts: BTreeMap<String, Part>,
    pub runs: Vec<Run>,
    pub joins: Vec<Join>,
}

/// One run: a box `thickness_m` across the plan line `from`..`to`, between the heights `y_m`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Run {
    /// Unique within the wall; seeds its cut's draws.
    pub name: String,
    /// A key of [`Wall::parts`], and the name the joins hold it by.
    pub part: String,
    pub component: String,
    /// The centre line's ends in plan, `(x, z)`, m.
    pub from: (f64, f64),
    pub to: (f64, f64),
    pub thickness_m: f64,
    /// Its foot and its top, m.
    pub y_m: (f64, f64),
}

impl Wall {
    /// Parse a wall's text and lay it out, refusing what [`Self::construction`] refuses.
    pub fn from_ron(text: &str) -> Result<Construction, String> {
        ron::from_str::<Self>(text)
            .map_err(|err| err.to_string())?
            .construction()
    }

    /// The wall as a construction, its pieces in the runs' order. Refuses a part whose frame or
    /// cut is unusable, a run whose name repeats, whose part the wall does not state or whose
    /// numbers span no box, runs whose interiors meet, and an attachment kind the shared list
    /// does not hold.
    pub fn construction(self) -> Result<Construction, String> {
        spec::check_holds(&self.foundation_attachment, &self.joins)?;
        if !self.foundation_y_m.is_finite() {
            return Err(format!(
                "foundation_y_m {} is not finite",
                self.foundation_y_m
            ));
        }
        for (name, part) in &self.parts {
            spec::check_frame(&format!("parts[{name:?}]"), part.surface_frame)?;
            (part.cut.check()).map_err(|err| format!("parts[{name:?}].cut: {err}"))?;
        }
        let mut pieces: Vec<Piece> = Vec::with_capacity(self.runs.len());
        for (k, run) in self.runs.into_iter().enumerate() {
            let what = format!("runs[{k}] ({})", run.name);
            if pieces.iter().any(|other| other.name == run.name) {
                return Err(format!("{what}: its name is taken"));
            }
            let part = (self.parts.get(&run.part))
                .ok_or_else(|| format!("{what}.part: parts states no {:?}", run.part))?;
            let (from, to) = (DVec2::from(run.from), DVec2::from(run.to));
            let length = from.distance(to);
            let (foot, top) = run.y_m;
            if !([length, run.thickness_m, top - foot].iter()).all(|v| v.is_finite() && *v > 0.0) {
                return Err(format!(
                    "{what}: author distinct finite ends, a positive thickness_m and y_m as (foot, top)"
                ));
            }
            let along = (to - from) / length;
            let prism = Prism {
                frame: Frame {
                    origin: DVec3::new(from.x, 0.0, from.y),
                    u: DVec3::new(along.x, 0.0, along.y),
                    v: DVec3::Y,
                },
                profile: vec![
                    DVec2::new(0.0, foot),
                    DVec2::new(length, foot),
                    DVec2::new(length, top),
                    DVec2::new(0.0, top),
                ],
                near: -run.thickness_m / 2.0,
                far: run.thickness_m / 2.0,
            };
            pieces.push(Piece {
                name: run.name,
                part: run.part,
                component: run.component,
                material: part.material.clone(),
                surface_frame: part.surface_frame,
                solid: prism.solid(),
                pattern: match &part.cut {
                    PartCut::Whole => Pattern::Whole,
                    PartCut::Planar(rule) => Pattern::Planar(Planar {
                        depth: prism.frame.w(),
                        rule: rule.clone(),
                    }),
                    PartCut::Cells(cells) => Pattern::Cells(cells.clone()),
                },
            });
        }
        spec::check_disjoint(&pieces)?;
        Ok(Construction {
            id: self.id,
            foundation_y: self.foundation_y_m,
            foundation_attachment: self.foundation_attachment,
            pieces,
            joins: self.joins,
        })
    }
}
