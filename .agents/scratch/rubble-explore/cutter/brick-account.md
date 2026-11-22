# Brick account: checkpoints 1-2

Subject: the game's building model. All MEASURED, release build, governed contact, 3 runs per arm (identical except junction after: bodies 540-542), plus 3-4 runs at 20 km/h. Ten cores: macOS cannot pin two, so `cpu` (summed CPU per tick, p95) bounds a two-core tick between cpu/2 and cpu. Budget 15.6 ms. "House" is `ram-house-face.json` (house_0046 long wall); `ram.json`'s house_0001 is end-on.

| case, arm | cells freed | bodies born | awake peak | tick p50 / p95 ms | cpu ms | crossing s | least fwd m/s | in place | sank | below ground at end |
|---|---|---|---|---|---|---|---|---|---|---|
| house, floor | 368 (211 decoration) | 13 | 13 | 2.57 / 5.69 | 9.9 | 1.45 | 10.44 | 0.62 | 0 | 0 |
| house, none | 448 | 540 | 381 | 6.24 / 8.71 | 21.3 | 1.44 | 9.18 | 0.54 | 2 | 14 |
| hall wall, floor | 283 (39) | 96 | 86 | 1.66 / 7.74 | 14.0 | 2.33 | 5.12 | 0.91 | 2 | 0 |
| hall wall, none | 297 | 244 | 193 | 6.02 / 10.86 | 18.6 | 3.34 | 4.15 | 0.82 | 16 | 1 |
| junction, floor | 372 (52) | 152 | 117 | 5.05 / 7.27 | 15.9 | never | -0.44 | 0.67 | 2 | 2 |
| junction, none | 462 | 542 | 397 | 8.30 / 9.29 | 24.3 | 7.98 | 0.42 | 0.56 | 31 | 6 |
| wall, floor | 175 (78) | 5 | 5 | 2.12 / 4.38 | 6.3 | 0.91 | 9.31 | 0.57 | 0 | 0 |
| wall, none | 190 | 220 | 190 | 3.18 / 5.63 | 10.0 | 0.97 | 9.08 | 0.56 | 2 | 6 |

Stage means, house, µs: physics 1125 to 4827, bound 306 to 912, structure 150 to 362, crumble 31 to 248. Deepest hull overlap 0.25-0.28 m to 0.24-0.33 m; fastest piece 12.8-14.9 to 15.0-18.1 m/s against a 10.3-10.8 m/s hull; hull worst-tick change -0.10/-0.72/-1.71/-0.30 to -0.13/-0.37/-0.65/-0.30 m/s.

## Deleted, kept

Deleted: `route`'s size split, `server_rubble_min_size_m` and its row, `merge_fragments`, `Live::rubble_joints`, `debris::decor` with `WorldShed`, its channel, refusal and look rows (route was their only producer). `PROTOCOL_REV` 108, hashes re-pinned. Kept: `Fate::Dust`, which named laws still use. One test holds the invariant; the mutant (a size floor in `evaluate_owner`) turns it red. Backlog destruction-04/05/07/08/10 stay open: design-level.

## Faults

- Single cells fall through the terrain (50 m/s, 78-126 m below): 0-2 with the floor, 2/16/31/2 without. Pre-existing, now common.
- Two-core tick: house cpu 21.3 ms and junction 24.3 ms straddle the budget. Clip runs (a rendering client beside the server) went over budget on 17 and 29 ticks.
- Nearly everything ends as a single cell: house 405 of 420 bodies, wall 188 of 188. Releases bear larger pieces (below); the landing split reduces them (103 landings, 396 children).
- Bodies do not settle: house 264 of 420 awake at 10 s, 47 moving.
- Hall wall at 20 km/h: crossing 4.36 to 7.09 s, least 0.46 m/s.

## Clips (20 km/h, one run per arm; such runs do not repeat)

- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-brick/house-exit-side.mp4`: left, a clean breach and three lumps; right, a fan of about a hundred identical brick cubes and the roof timbers down inside.
- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-brick/house-raised.mp4`: left, roof intact, two lumps outside; right, a brick trail along the path, roof fallen in.
- `/Users/Yan/github/vikng-dev/overmatch/.agents/scratch/rubble-explore/clips-brick/junction-raised.mp4`: both hulls held; right sits deeper under a rafter and brick heap.

## Replay (after arm, one run each)

| window | house: fragments / singles / ≥4 / ≥11 | hall wall |
|---|---|---|
| born | 135 / 59 / 51% / 22% | 61 / 44 / 27% / 0% |
| 1 | 110 / 42 / 63% / 32% | 50 / 31 / 34% / 14% |
| 2 | 108 / 39 / 63% / 32% | 47 / 29 / 42% / 15% |
| 4 | 99 / 35 / 66% / 40% | 41 / 24 / 51% / 18% |
| 8 | 96 / 33 / 67% / 41% | 38 / 23 / 54% / 26% |

Largest 38 and 25 cells. Ignoring which body pressed, window 8 gives 85% and 61% in ≥11, largest 158 and 39.

Reading: re-joining under one body is a modest gain, and the landing split would undo it; that split is the larger cause. Without a window, one push is the connected cells a body keeps yielding by `Give`, released when a tick adds none. DERIVED risks: pieces born up to 22 ticks later, so deeper inside the hull; a hull sliding along a wall never has a tick that adds none; a 158-cell release takes its unsupported storeys with it. Not built.
