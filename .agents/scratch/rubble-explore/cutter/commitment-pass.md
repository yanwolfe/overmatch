# Commitment pass: report

Subject: the game's building model (game state only). `lane/bounded`, on merge `b9e4f6ef`.

**Verdict: ready to review.** fmt, CI clippy and the server check are green; the lib suite passes (2,773 tests, MEASURED) after the two fixes below.

## What went
MEASURED against the merge: 9,544 lines out, 966 in, 46 files.
- The hull sweep, whole, with its cost stages and tests. `construction::hulls` keeps the per-chunk hulls and load-gate refusals.
- The relay readings (`relay.rs`, `OVERMATCH_RELAY`, `wanting`/`used`, press verdicts): no tracked script read them.
- Dead with them: `Assessment::adopt`, `Material::bearing`, `peak_speed`, `cancel_staged`, the contact-modify hook on construction colliders.
- Checkpoint 5 ran with only the log on, so no instrument word became code: give is forgotten, the shed is Coulomb (`StandIn`, sideways share), push-out is the engine's own.
- Fork: 29 lines for the bound (MEASURED), recorded as a divergence with its seam in `avian_ext::reaction_bound`.

## Behaviour did not move
Same merged tree before and after the pass, dev build. Five governed cells (the three below, `ram-house-face`, plain brick wall) match bit for bit: hull pose and velocity from 100 ticks before contact to the end, and the end structure (MEASURED). Checkpoint 5 used the lane's old pass format, so its column is a reference, not a pair.

| Pass | Checkpoint 5: crossing, least m/s, in place | After |
|---|---|---|
| Junction 10 km/h | never, -0.25, 83 % | never, -0.88, 86 % |
| Junction 20 km/h | never, -0.38, 73 % | never, -0.22, 64 % |
| Junction governed | never, -0.60, 65 % | never, -0.44, 67 % |
| Hall wall 10 km/h | 6.89 s, 0.72, 92 % | 9.83 s, 0.38, 92 % |
| Hall wall 20 km/h | 5.13 s, 2.41, 82 % | 4.23 s, 3.75, 92 % |
| Hall wall governed | 3.13 s, 4.18, 79 % | 2.33 s, 5.12, 91 % |
| `house_0001` face governed | 1.67 s, 8.21, 59 % | 3.34 s, 2.19, 33 % |

Held-speed cells do not repeat: the baseline binary run twice gave hall wall 20 km/h crossing in 4.14 s, then never. Battery: 60 cells, 59 complete; table at `target/commit-pass/proving/table.md` in the bay.

## Ordinary sessions
Release, default features, no instrument, one hidden scripted client into `house_0001`'s face (MEASURED). The wall gives at each speed.

| Contact m/s | Crossing | Least forward m/s | Departed chunks of 1,077 |
|---|---|---|---|
| 2.83 | 4.08 s | 2.68 | 46 |
| 4.33 | 2.83 s | 4.25 | 109 |
| 5.79 | 2.20 s | 5.69 | 101 |

## Found and fixed
- Two `track::sim` tests failed at schedule build: the seam's `deal` had no order against their systems. They now run before it.
- A rest contact read as contact on the first tick; the instrument now reads the first impulse the target returns. Impact ticks moved by a tick or two.
- Backlog row destruction-03 is closed.

## Found and left
- `ram` governed never crosses (least -0.03 m/s); only the junction is listed as accepted.
- `ram-house-face` rest is invalid three runs of three: the client is judged not keeping up.
- Unwatched since the sweep's tests went: a frozen piece thaws when its support is freed; pricing is independent of worker assignment; the belt cap.
- Grain's along-strength has no law reader.
- Backlog rows net-13 and net-15 stay, as this pass promised no behaviour change.
