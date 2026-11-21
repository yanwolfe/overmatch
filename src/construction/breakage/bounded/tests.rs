//! The contact law wired into the step, on generic wall fixtures (game state only).

use avian3d::prelude::*;

use super::super::tests::composed_kind;
use super::*;
use crate::construction::tests::wall_construction;

/// Fixture: a wall 6 m long and 2.4 m high, and a rigid box 2 m wide, 1.2 m high and 2 m long
/// driven at its middle along +z from 0.175 m short of a 0.25 m leaf.
const WALL: Vec3 = Vec3::new(6.0, 2.4, 0.25);
const BOX: Vec3 = Vec3::new(2.0, 1.2, 2.0);
const START: Vec3 = Vec3::new(0.0, 1.0, -1.3);

struct Ram {
    app: App,
    root: Entity,
    body: Entity,
}

struct Outcome {
    /// The body's speed along +z after the run, its least during it and its largest loss over
    /// one tick, m/s.
    end: f32,
    least: f32,
    worst: f32,
    /// Where it ended along z, m.
    reached: f32,
    freed: usize,
    standing: usize,
}

impl Ram {
    fn new(kind: &Arc<ConstructionKind>, mass: f32, speed: f32, force: f32) -> Self {
        let (mut app, root) = composed_kind(false, kind);
        for _ in 0..3 {
            app.update();
        }
        let body = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Position(START),
                Rotation::default(),
                Collider::cuboid(BOX.x, BOX.y, BOX.z),
                Mass(mass),
                NoAutoMass,
                piece_layers(),
                LinearVelocity(Vec3::Z * speed),
                ConstantForce(Vec3::Z * force),
                SleepingDisabled,
            ))
            .id();
        Self { app, root, body }
    }

    fn speed(&self) -> f32 {
        self.app
            .world()
            .get::<LinearVelocity>(self.body)
            .unwrap()
            .0
            .z
    }

    fn run(&mut self, ticks: usize) -> Outcome {
        let mut last = self.speed();
        let (mut least, mut worst, mut freed) = (last, 0.0_f32, 0);
        for _ in 0..ticks {
            self.app.update();
            let speed = self.speed();
            freed += self.app.world().resource::<BoundAccount>().freed.len();
            least = least.min(speed);
            worst = worst.max(last - speed);
            last = speed;
        }
        let owner = self.app.world().get::<Breakage>(self.root).unwrap();
        Outcome {
            end: last,
            least,
            worst,
            reached: self.app.world().get::<Position>(self.body).unwrap().0.z,
            freed,
            standing: (0..owner.live.structure.chunks.len())
                .filter(|&c| owner.live.state.standing(c))
                .count(),
        }
    }
}

#[test]
fn explore() {
    for (material, size, cells, mass, speed, force) in [
        ("Brick", WALL, [6, 4, 1], 57_000.0, 4.5, 0.0),
        ("Brick", WALL, [12, 8, 1], 57_000.0, 4.5, 0.0),
        ("Brick", WALL.with_z(0.5), [6, 4, 2], 57_000.0, 4.5, 0.0),
        ("Brick", WALL, [6, 4, 1], 57_000.0, 2.8, 0.0),
        ("Brick", WALL, [12, 8, 1], 57_000.0, 2.8, 0.0),
        ("Brick", WALL, [6, 4, 1], 57_000.0, 9.4, 0.0),
        ("Brick", WALL, [12, 8, 1], 57_000.0, 9.4, 0.0),
        ("Brick", WALL, [6, 4, 1], 57_000.0, 0.0, 300_000.0),
        ("Brick", WALL, [6, 4, 1], 57_000.0, 0.0, 30_000.0),
        ("Brick", WALL, [6, 4, 1], 20.0, 4.5, 0.0),
        ("Brick", WALL, [6, 4, 1], 500.0, 4.5, 0.0),
    ] {
        let kind = wall_construction(size, cells, material);
        let mut ram = Ram::new(&kind, mass, speed, force);
        let o = ram.run(160);
        println!(
            "ROW {material} {size} {cells:?} mass {mass} speed {speed} force {force}: end {:.3} least {:.3} worst tick {:.3} reached {:.2} freed {} standing {}",
            o.end, o.least, o.worst, o.reached, o.freed, o.standing
        );
    }
}
